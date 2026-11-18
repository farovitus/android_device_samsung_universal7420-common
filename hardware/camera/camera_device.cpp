#define LOG_TAG "s6cam_device"
#include "camera_device.h"

#include <aidl/android/hardware/camera/common/Status.h>
#include <aidl/android/hardware/camera/device/BufferStatus.h>
#include <aidl/android/hardware/camera/device/CaptureResult.h>
#include <aidl/android/hardware/camera/device/ErrorCode.h>
#include <aidl/android/hardware/camera/device/ErrorMsg.h>
#include <aidl/android/hardware/camera/device/NotifyMsg.h>
#include <aidl/android/hardware/camera/device/ShutterMsg.h>
#include <aidl/android/hardware/camera/device/StreamType.h>
#include <aidl/android/hardware/graphics/common/BufferUsage.h>
#include <aidl/android/hardware/graphics/common/Dataspace.h>
#include <aidl/android/hardware/graphics/common/PixelFormat.h>
#include <fcntl.h>
#include <hardware/gralloc.h>
#include <log/log.h>
#include <poll.h>
#include <sys/system_properties.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

namespace s6cam {

namespace {

using CamStatus = ::aidl::android::hardware::camera::common::Status;
using ::aidl::android::hardware::camera::device::BufferStatus;
using ::aidl::android::hardware::camera::device::ErrorCode;
using ::aidl::android::hardware::camera::device::ErrorMsg;
using ::aidl::android::hardware::camera::device::NotifyMsg;
using ::aidl::android::hardware::camera::device::ShutterMsg;
using ::aidl::android::hardware::camera::device::StreamType;
using ::aidl::android::hardware::graphics::common::BufferUsage;
using ::aidl::android::hardware::graphics::common::Dataspace;
using ::aidl::android::hardware::graphics::common::PixelFormat;

ScopedAStatus serviceErr(CamStatus s, const char* msg = "") {
    return ScopedAStatus::fromServiceSpecificErrorWithMessage(static_cast<int32_t>(s), msg);
}

int64_t monotonicNowNs() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

bool writeSysfs(const char* path, const char* val) {
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return false;
    ssize_t n = write(fd, val, std::strlen(val));
    int err = errno;
    close(fd);
    if (n < 0) {
        ALOGW("writeSysfs(%s, %s) failed: %s", path, val, strerror(err));
        return false;
    }
    return true;
}

bool setTorchSysfs(bool on, int32_t level = 6) {
    if (!on) {
        bool ok = writeSysfs("/sys/class/camera/flash/rear_torch_flash", "0");
        (void)writeSysfs("/sys/class/camera/flash/rear_flash", "0");
        return ok;
    }
    int clamped = std::clamp(level, 1, 15);
    int val = (clamped == 1) ? 1001 : clamped;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d", val);
    return writeSysfs("/sys/class/camera/flash/rear_torch_flash", buf);
}

bool isValidStreamConfig(const CameraStaticConfig& cfg, const StreamConfiguration& sc) {
    if (sc.streams.empty()) return false;
    int blobCount = 0;
    for (const auto& s : sc.streams) {
        if (s.streamType != StreamType::OUTPUT) return false;
        if (s.width <= 0 || s.height <= 0 || s.width > cfg.activeWidth || s.height > cfg.activeHeight) {
            // Note: BLOB width/height matches image size in ConfigureStreams
            return false;
        }
        if (s.format == PixelFormat::BLOB) {
            if (++blobCount > 1) return false;
        } else if (s.format != PixelFormat::IMPLEMENTATION_DEFINED &&
                   s.format != PixelFormat::YCBCR_420_888 &&
                   s.format != PixelFormat::YV12 &&
                   s.format != PixelFormat::YCRCB_420_SP &&
                   s.format != PixelFormat::RGBA_8888 &&
                   s.format != PixelFormat::RGBX_8888) {
            return false;
        }
    }
    return true;
}

}  // namespace

AidlCameraDeviceSession::AidlCameraDeviceSession(const V4l2DeviceNodes& nodes,
                                                 const CameraStaticConfig& staticCfg,
                                                 std::shared_ptr<ICameraDeviceCallback> callback,
                                                 std::function<void()> onCloseCb)
    : nodes_(nodes),
      staticCfg_(staticCfg),
      callback_(std::move(callback)),
      onCloseCb_(std::move(onCloseCb)),
      pipeline_(nodes),
      aeAwb_(nodes.pattern) {}

AidlCameraDeviceSession::~AidlCameraDeviceSession() {
    close();
}

bool AidlCameraDeviceSession::initialize() {
    if (!gralloc_.init()) {
        ALOGE("Failed to initialize GrallocBridge");
        return false;
    }
    if (!pipeline_.openNodes()) {
        ALOGE("Failed to open V4L2 nodes for camera %d", nodes_.cameraId);
        return false;
    }

    if (staticCfg_.hasAutofocus && pipeline_.getFocusRange(&af_.lensMin, &af_.lensMax)) {
        af_.enabled = true;
        af_.curPos = std::clamp(180, af_.lensMin, af_.lensMax);
        af_.bestPos = af_.curPos;
        af_.afState = ANDROID_CONTROL_AF_STATE_INACTIVE;
        pipeline_.setFocusPos(af_.curPos);
        ALOGI("Camera %d AF initialized (range=%d..%d, initial=%d)",
              nodes_.cameraId, af_.lensMin, af_.lensMax, af_.curPos);
    }

    char memfdProp[PROP_VALUE_MAX] = {};
    if (__system_property_get("sys.use_memfd", memfdProp) > 0 && memfdProp[0] == '1') {
        constexpr size_t kFmqSize = 1024 * 1024;
        requestQueue_ = std::make_unique<MetadataQueue>(kFmqSize, false);
        resultQueue_ = std::make_unique<MetadataQueue>(kFmqSize, false);
        if (!requestQueue_->isValid() || !resultQueue_->isValid()) {
            ALOGI("Using inline metadata transport (FMQ unavailable)");
            requestQueue_.reset();
            resultQueue_.reset();
        }
    } else {
        ALOGI("Using inline metadata transport (sys.use_memfd disabled)");
    }

    stopWorker_ = false;
    workerThread_ = std::thread(&AidlCameraDeviceSession::workerLoop, this);
    ALOGI("Camera %d session opened", nodes_.cameraId);
    return true;
}

ScopedAStatus AidlCameraDeviceSession::close() {
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        if (closed_) return ScopedAStatus::ok();
        closed_ = true;
    }

    flushing_.store(true);
    {
        std::lock_guard<std::mutex> lk(reqMutex_);
        stopWorker_ = true;
    }
    reqCv_.notify_all();
    if (workerThread_.joinable()) {
        workerThread_.join();
    }

    if (staticCfg_.hasFlash && flashTorchActive_) {
        setTorchSysfs(false);
        flashTorchActive_ = false;
    }

    pipeline_.closeNodes();
    gralloc_.clearAll();

    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        activeStreams_.clear();
        lastSettingsBytes_.clear();
    }
    if (onCloseCb_) {
        onCloseCb_();
    }
    ALOGI("Camera %d session closed", nodes_.cameraId);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::configureStreams(const StreamConfiguration& in_reqCfg,
                                                      std::vector<HalStream>* _aidl_return) {
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        if (closed_) return serviceErr(CamStatus::INTERNAL_ERROR, "Session closed");
    }
    if (!isValidStreamConfig(staticCfg_, in_reqCfg)) {
        return serviceErr(CamStatus::ILLEGAL_ARGUMENT, "Unsupported stream configuration");
    }

    // Wait for any pending requests to drain before resetting active streams
    {
        std::unique_lock<std::mutex> lk(reqMutex_);
        idleCv_.wait(lk, [&]() { return reqDeque_.empty() && !busyWorker_; });
    }

    std::unordered_map<int32_t, ActiveStream> newStreams;
    std::vector<HalStream> halStreams;
    halStreams.reserve(in_reqCfg.streams.size());

    for (const auto& s : in_reqCfg.streams) {
        HalStream hs{};
        hs.id = s.id;
        hs.consumerUsage = BufferUsage(0);
        hs.producerUsage = BufferUsage::CPU_WRITE_OFTEN;
        hs.overrideDataSpace = s.dataSpace;
        hs.physicalCameraId = "";

        if (s.format == PixelFormat::IMPLEMENTATION_DEFINED) {
            const int64_t u = static_cast<int64_t>(s.usage);
            const bool wantYuv =
                (u & static_cast<int64_t>(BufferUsage::VIDEO_ENCODER)) != 0 ||
                (u & static_cast<int64_t>(BufferUsage::CPU_READ_MASK)) != 0;
            hs.overrideFormat = wantYuv ? PixelFormat::YCBCR_420_888 : PixelFormat::RGBA_8888;
            hs.maxBuffers = 4;
        } else {
            hs.overrideFormat = s.format;
            hs.maxBuffers = (s.format == PixelFormat::BLOB) ? 2 : 4;
        }

        ActiveStream as{};
        as.id = s.id;
        as.width = s.width;
        as.height = s.height;
        as.format = static_cast<int32_t>(hs.overrideFormat);
        newStreams.emplace(s.id, as);
        halStreams.push_back(hs);

        ALOGI("configureStreams cam%d: id=%d %dx%d reqFmt=0x%x -> halFmt=0x%x maxBuf=%d",
              nodes_.cameraId, s.id, s.width, s.height,
              static_cast<int32_t>(s.format), as.format, hs.maxBuffers);
    }

    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        for (const auto& kv : activeStreams_) {
            if (newStreams.find(kv.first) == newStreams.end()) {
                gralloc_.removeStream(kv.first);
            }
        }
        activeStreams_ = std::move(newStreams);
        lastSettingsBytes_.clear();
    }

    // Pre-start viewfinder V4L2 stream immediately for minimal shutter latency
    if (!pipeline_.startStream(nodes_.previewWidth, nodes_.previewHeight, 4)) {
        return serviceErr(CamStatus::INTERNAL_ERROR, "Failed to start V4L2 viewfinder stream");
    }
    sensorLimits_ = pipeline_.querySensorLimits();
    aeAwb_.reset(sensorLimits_);

    *_aidl_return = std::move(halStreams);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::constructDefaultRequestSettings(
    RequestTemplate in_type, AidlCameraMetadata* _aidl_return) {
    std::lock_guard<std::mutex> lk(stateMutex_);
    if (closed_) return serviceErr(CamStatus::INTERNAL_ERROR, "Session closed");
    *_aidl_return = buildDefaultRequest(staticCfg_, in_type);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::flush() {
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        if (closed_) return serviceErr(CamStatus::INTERNAL_ERROR, "Session closed");
    }

    flushing_.store(true);
    {
        std::unique_lock<std::mutex> lk(reqMutex_);
        idleCv_.wait(lk, [&]() { return reqDeque_.empty() && !busyWorker_; });
    }
    flushing_.store(false);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::getCaptureRequestMetadataQueue(
    MQDescriptor<int8_t, SynchronizedReadWrite>* _aidl_return) {
    if (requestQueue_ && requestQueue_->isValid()) {
        *_aidl_return = requestQueue_->dupeDesc();
    }
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::getCaptureResultMetadataQueue(
    MQDescriptor<int8_t, SynchronizedReadWrite>* _aidl_return) {
    if (resultQueue_ && resultQueue_->isValid()) {
        *_aidl_return = resultQueue_->dupeDesc();
    }
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::isReconfigurationRequired(
    const AidlCameraMetadata& /*in_old*/, const AidlCameraMetadata& /*in_new*/, bool* _aidl_return) {
    *_aidl_return = false;
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::processCaptureRequest(
    const std::vector<CaptureRequest>& in_requests,
    const std::vector<BufferCache>& in_cachesToRemove,
    int32_t* _aidl_return) {
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        if (closed_) return serviceErr(CamStatus::INTERNAL_ERROR, "Session closed");
    }

    for (const auto& rm : in_cachesToRemove) {
        gralloc_.removeCached(rm.streamId, rm.bufferId);
    }

    int32_t accepted = 0;
    for (const auto& req : in_requests) {
        QueuedReq q{};
        q.frameNumber = req.frameNumber;

        if (req.fmqSettingsSize > 0 && requestQueue_ && requestQueue_->isValid()) {
            q.settingsBytes.resize(static_cast<size_t>(req.fmqSettingsSize));
            if (!requestQueue_->read(reinterpret_cast<int8_t*>(q.settingsBytes.data()),
                                     q.settingsBytes.size())) {
                ALOGE("Failed reading %lld bytes from request FMQ",
                      static_cast<long long>(req.fmqSettingsSize));
                q.settingsBytes.clear();
            }
        } else if (!req.settings.metadata.empty()) {
            q.settingsBytes = req.settings.metadata;
        }

        {
            std::lock_guard<std::mutex> lk(stateMutex_);
            if (q.settingsBytes.empty()) {
                q.settingsBytes = lastSettingsBytes_;
            } else {
                lastSettingsBytes_ = q.settingsBytes;
            }
        }

        for (const auto& ob : req.outputBuffers) {
            native_handle_t* hnd = gralloc_.getOrImportLocked(ob.streamId, ob.bufferId, ob.buffer);
            int fenceFd = -1;
            if (!ob.acquireFence.fds.empty() && ob.acquireFence.fds[0].get() >= 0) {
                fenceFd = fcntl(ob.acquireFence.fds[0].get(), F_DUPFD_CLOEXEC, 0);
            }
            q.outputs.push_back(QueuedBuf{ob.streamId, ob.bufferId, hnd, fenceFd});
        }

        {
            std::lock_guard<std::mutex> lk(reqMutex_);
            reqDeque_.push_back(std::move(q));
        }
        reqCv_.notify_one();
        ++accepted;
    }

    *_aidl_return = accepted;
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::signalStreamFlush(const std::vector<int32_t>& /*ids*/,
                                                         int32_t /*counter*/) {
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::switchToOffline(
    const std::vector<int32_t>& /*keep*/, CameraOfflineSessionInfo* /*info*/,
    std::shared_ptr<ICameraOfflineSession>* _aidl_return) {
    *_aidl_return = nullptr;
    return serviceErr(CamStatus::OPERATION_NOT_SUPPORTED, "Offline session not supported");
}

ScopedAStatus AidlCameraDeviceSession::repeatingRequestEnd(int32_t /*frame*/,
                                                           const std::vector<int32_t>& /*ids*/) {
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDeviceSession::configureStreamsV2(const StreamConfiguration& /*cfg*/,
                                                          ConfigureStreamsRet* /*ret*/) {
    return serviceErr(CamStatus::OPERATION_NOT_SUPPORTED, "Use configureStreams");
}

void AidlCameraDeviceSession::workerLoop() {
    while (true) {
        QueuedReq req;
        {
            std::unique_lock<std::mutex> lk(reqMutex_);
            reqCv_.wait(lk, [&]() { return stopWorker_ || !reqDeque_.empty(); });
            if (stopWorker_ && reqDeque_.empty()) {
                busyWorker_ = false;
                idleCv_.notify_all();
                return;
            }
            req = std::move(reqDeque_.front());
            reqDeque_.pop_front();
            busyWorker_ = true;
        }

        const bool errFlush = stopWorker_ || flushing_.load();
        if (errFlush) {
            returnQueuedAsError(req);
        } else {
            processSingleRequest(req, false);
        }

        {
            std::lock_guard<std::mutex> lk(reqMutex_);
            busyWorker_ = false;
            if (reqDeque_.empty()) {
                idleCv_.notify_all();
            }
        }
    }
}

void AidlCameraDeviceSession::returnQueuedAsError(const QueuedReq& req) {
    for (const auto& qb : req.outputs) {
        if (qb.acquireFenceFd >= 0) ::close(qb.acquireFenceFd);
    }

    if (callback_) {
        ErrorMsg em{};
        em.frameNumber = req.frameNumber;
        em.errorStreamId = -1;
        em.errorCode = ErrorCode::ERROR_REQUEST;
        NotifyMsg nm;
        nm.set<NotifyMsg::Tag::error>(em);
        std::lock_guard<std::mutex> cbLk(callbackMutex_);
        callback_->notify({nm});

        CaptureResult res{};
        res.frameNumber = req.frameNumber;
        res.fmqResultSize = 0;
        res.partialResult = 0;
        res.inputBuffer.streamId = -1;
        for (const auto& qb : req.outputs) {
            StreamBuffer sb{};
            sb.streamId = qb.streamId;
            sb.bufferId = qb.bufferId;
            sb.status = BufferStatus::ERROR;
            res.outputBuffers.push_back(std::move(sb));
        }
        std::vector<CaptureResult> results;
        results.push_back(std::move(res));
        callback_->processCaptureResult(results);
    }
}

void AidlCameraDeviceSession::processSingleRequest(const QueuedReq& req, bool /*isErrorFlush*/) {
    IspRequestParams params = parseRequestParams(req.settingsBytes);
    params.focalLengthMm = staticCfg_.focalLengthMm;

    // Wait on buffer acquire fences before writing
    for (const auto& qb : req.outputs) {
        if (qb.acquireFenceFd >= 0) {
            struct pollfd pfd{};
            pfd.fd = qb.acquireFenceFd;
            pfd.events = POLLIN;
            poll(&pfd, 1, 1500);
            ::close(qb.acquireFenceFd);
        }
    }

    // Check if high-resolution full-sensor mode (e.g. 5328x3000 16MP JPEG) is needed for this request
    bool needFullMode = false;
    std::vector<ActiveStream> targetStreams(req.outputs.size());
    {
        std::lock_guard<std::mutex> lk(stateMutex_);
        for (size_t i = 0; i < req.outputs.size(); ++i) {
            auto it = activeStreams_.find(req.outputs[i].streamId);
            if (it != activeStreams_.end()) {
                targetStreams[i] = it->second;
                if (it->second.width > nodes_.previewWidth || it->second.height > nodes_.previewHeight) {
                    needFullMode = true;
                }
            }
        }
    }

    const bool switchedToFull =
        needFullMode && (nodes_.fullWidth > nodes_.previewWidth || nodes_.fullHeight > nodes_.previewHeight);

    bool captureFlashFired = false;
    bool hasFlashAe = false;
    AeAwbResult flashAeRes{};
    if (staticCfg_.hasFlash) {
        const AeAwbResult curAe = aeAwb_.current();
        const bool wantTorch = (params.flashMode == ANDROID_FLASH_MODE_TORCH);
        const bool wantCaptureStrobe =
            (switchedToFull && !wantTorch) &&
            (params.flashMode == ANDROID_FLASH_MODE_SINGLE ||
             params.aeMode == ANDROID_CONTROL_AE_MODE_ON_ALWAYS_FLASH ||
             (params.aeMode == ANDROID_CONTROL_AE_MODE_ON_AUTO_FLASH &&
              curAe.appliedExposureNs >= 22000000LL));

        if (wantCaptureStrobe) {
            if (!pipeline_.isStreaming()) {
                pipeline_.startStream(nodes_.previewWidth, nodes_.previewHeight, 4);
                sensorLimits_ = pipeline_.querySensorLimits();
                aeAwb_.reset(sensorLimits_);
                pipeline_.applyExposureGain(sensorLimits_.expoDef, sensorLimits_.againDef,
                                            sensorLimits_.dgainDef);
            }
            setTorchSysfs(true, 12);  // 240 mA continuous assist avoids rolling-shutter flash banding
            flashTorchActive_ = true;
            captureFlashFired = true;

            // Drain pre-torch queued frames, then meter twice with fast damping (~230ms total)
            // so full-res capture converges accurately under flash illumination.
            AeAwbController flashAe = aeAwb_;
            for (int k = 0; k < 7; ++k) {
                RawFrameView mframe{};
                if (pipeline_.dequeueFrame(&mframe, 400)) {
                    const uint16_t* staged = (k == 2 || k == 5)
                        ? isp_.stageRaw(mframe.data, mframe.width, mframe.height, mframe.strideBytes)
                        : nullptr;
                    pipeline_.requeueFrame(mframe.bufferIndex);
                    if (staged) {
                        flashAeRes = flashAe.updateFromRaw(staged, mframe.width, mframe.height,
                                                           isp_.stagedStrideBytes(), params, true);
                        pipeline_.applyExposureGain(flashAeRes.exposureLines, flashAeRes.analogGain,
                                                    flashAeRes.digitalGain);
                        hasFlashAe = true;
                    }
                }
            }
        } else if (wantTorch != flashTorchActive_) {
            setTorchSysfs(wantTorch, 4);
            flashTorchActive_ = wantTorch;
        }
    }

    if (switchedToFull) {
        // Carry over current (or flash-adapted) AE gain/exposure into 16MP (REQBUFS=3 fits 160MB cam_cma)
        AeAwbResult prevAe = hasFlashAe ? flashAeRes : aeAwb_.current();
        if (pipeline_.startStream(nodes_.fullWidth, nodes_.fullHeight, 3)) {
            SensorCtrlLimits fullLim = pipeline_.querySensorLimits();
            int32_t scaledExpo = static_cast<int32_t>(
                std::llround(prevAe.appliedExposureNs / std::max(1.0, fullLim.lineDurationNs)));
            pipeline_.applyExposureGain(
                std::clamp(scaledExpo, fullLim.expoMin, fullLim.expoMax),
                std::clamp(prevAe.analogGain, fullLim.againMin, fullLim.againMax),
                std::clamp(prevAe.digitalGain, fullLim.dgainMin, fullLim.dgainMax));
            // Discard 3 transition frames so new exposure & PLL latch cleanly
            RawFrameView skip{};
            for (int k = 0; k < 3; ++k) {
                if (pipeline_.dequeueFrame(&skip)) {
                    pipeline_.requeueFrame(skip.bufferIndex);
                }
            }
        } else {
            ALOGW("Full 16MP mode switch failed, falling back to %dx%d",
                  nodes_.previewWidth, nodes_.previewHeight);
        }
    } else if (!pipeline_.isStreaming()) {
        pipeline_.startStream(nodes_.previewWidth, nodes_.previewHeight, 4);
        sensorLimits_ = pipeline_.querySensorLimits();
        aeAwb_.reset(sensorLimits_);
        pipeline_.applyExposureGain(sensorLimits_.expoDef, sensorLimits_.againDef,
                                    sensorLimits_.dgainDef);
    }

    RawFrameView frame{};
    bool gotFrame = pipeline_.dequeueFrame(&frame, 2000);
    int64_t tsNs = (gotFrame && frame.timestampNs > 0) ? frame.timestampNs : monotonicNowNs();

    if (captureFlashFired) {
        if (params.flashMode == ANDROID_FLASH_MODE_TORCH) {
            setTorchSysfs(true, 4);
            flashTorchActive_ = true;
        } else {
            setTorchSysfs(false);
            flashTorchActive_ = false;
        }
    }
    if (!staticCfg_.hasFlash) {
        lastFlashState_ = ANDROID_FLASH_STATE_UNAVAILABLE;
    } else if (captureFlashFired || flashTorchActive_) {
        lastFlashState_ = ANDROID_FLASH_STATE_FIRED;
    } else {
        lastFlashState_ = ANDROID_FLASH_STATE_READY;
    }

    // Copy uncached WC DMA-BUF once into CPU-cached staging buffer, then release kernel VB2 buffer immediately
    RawFrameView cachedFrame = frame;
    if (gotFrame) {
        const uint16_t* staged = isp_.stageRaw(frame.data, frame.width, frame.height, frame.strideBytes);
        pipeline_.requeueFrame(frame.bufferIndex);
        if (staged) {
            cachedFrame.data = staged;
            cachedFrame.strideBytes = isp_.stagedStrideBytes();
        } else {
            gotFrame = false;
        }
    }

    AeAwbResult aeRes = hasFlashAe ? flashAeRes : aeAwb_.current();
    if (gotFrame && !switchedToFull) {
        aeRes = aeAwb_.updateFromRaw(cachedFrame.data, cachedFrame.width, cachedFrame.height,
                                     cachedFrame.strideBytes, params);
        pipeline_.applyExposureGain(aeRes.exposureLines, aeRes.analogGain, aeRes.digitalGain);
        stepAutofocus(params, cachedFrame);
    }

    // Dispatch SHUTTER notify first (required before buffers are consumed by framework)
    if (callback_) {
        ShutterMsg sm{};
        sm.frameNumber = req.frameNumber;
        sm.timestamp = tsNs;
        sm.readoutTimestamp = tsNs + 15000000LL;
        NotifyMsg nm;
        nm.set<NotifyMsg::Tag::shutter>(sm);
        callback_->notify({nm});
    }

    std::vector<StreamBuffer> outBuffers;
    outBuffers.reserve(req.outputs.size());
    for (size_t i = 0; i < req.outputs.size(); ++i) {
        const auto& qb = req.outputs[i];
        const auto& st = targetStreams[i];
        StreamBuffer sb{};
        sb.streamId = qb.streamId;
        sb.bufferId = qb.bufferId;
        sb.status = BufferStatus::ERROR;

        LockedOutput lo{};
        if (gotFrame && qb.handle && st.id >= 0 &&
            gralloc_.lockForWrite(qb.handle, st.width, st.height, st.format, &lo)) {
            int32_t blobBytes = 0;
            bool ok = isp_.processOutput(cachedFrame.data, cachedFrame.width, cachedFrame.height,
                                         cachedFrame.strideBytes, staticCfg_.activeWidth,
                                         staticCfg_.activeHeight, nodes_.pattern, aeRes, params,
                                         lo, &blobBytes);
            gralloc_.unlock(qb.handle);
            if (ok) sb.status = BufferStatus::OK;
        }
        outBuffers.push_back(std::move(sb));
    }

    if (switchedToFull) {
        // Restore binned viewfinder stream cleanly for subsequent preview frames
        if (pipeline_.startStream(nodes_.previewWidth, nodes_.previewHeight, 4)) {
            sensorLimits_ = pipeline_.querySensorLimits();
            pipeline_.applyExposureGain(aeRes.exposureLines, aeRes.analogGain, aeRes.digitalGain);
        }
    }

    if (callback_) {
        AidlCameraMetadata resultMeta = buildResultMetadata(params, aeRes, tsNs);
        CaptureResult cr{};
        cr.frameNumber = req.frameNumber;
        cr.partialResult = 1;
        cr.inputBuffer.streamId = -1;
        cr.outputBuffers = std::move(outBuffers);

        bool wroteFmq = false;
        if (resultQueue_ && resultQueue_->isValid() && !resultMeta.metadata.empty()) {
            std::lock_guard<std::mutex> fmqLk(resultQueueMutex_);
            if (resultQueue_->availableToWrite() >= resultMeta.metadata.size()) {
                if (resultQueue_->write(reinterpret_cast<const int8_t*>(resultMeta.metadata.data()),
                                        resultMeta.metadata.size())) {
                    cr.fmqResultSize = static_cast<int64_t>(resultMeta.metadata.size());
                    wroteFmq = true;
                }
            }
        }
        if (!wroteFmq) {
            cr.fmqResultSize = 0;
            cr.result = std::move(resultMeta);
        }

        std::vector<CaptureResult> results;
        results.push_back(std::move(cr));
        std::lock_guard<std::mutex> cbLk(callbackMutex_);
        callback_->processCaptureResult(results);
    }
}

IspRequestParams AidlCameraDeviceSession::parseRequestParams(const std::vector<uint8_t>& bytes) const {
    IspRequestParams p{};
    MetaReader r(bytes);
    if (!r.valid()) return p;

    r.getU8(ANDROID_CONTROL_MODE, &p.controlMode);
    r.getU8(ANDROID_CONTROL_AE_MODE, &p.aeMode);
    uint8_t u8v = 0;
    if (r.getU8(ANDROID_CONTROL_AE_LOCK, &u8v)) p.aeLock = (u8v == ANDROID_CONTROL_AE_LOCK_ON);
    r.getI32(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &p.aeExposureComp);
    r.getU8(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, &p.aePrecaptureTrigger);
    r.getU8(ANDROID_CONTROL_AWB_MODE, &p.awbMode);
    if (r.getU8(ANDROID_CONTROL_AWB_LOCK, &u8v)) p.awbLock = (u8v == ANDROID_CONTROL_AWB_LOCK_ON);
    r.getU8(ANDROID_CONTROL_AF_MODE, &p.afMode);
    r.getU8(ANDROID_CONTROL_AF_TRIGGER, &p.afTrigger);
    r.getU8(ANDROID_FLASH_MODE, &p.flashMode);
    r.getF32(ANDROID_LENS_FOCUS_DISTANCE, &p.focusDistance);
    int32_t afReg[5] = {0, 0, 0, 0, 0};
    if (r.getI32(ANDROID_CONTROL_AF_REGIONS, afReg, 5) && afReg[4] > 0 && afReg[2] > afReg[0] && afReg[3] > afReg[1]) {
        p.hasAfRegion = true;
        p.afX = afReg[0];
        p.afY = afReg[1];
        p.afW = afReg[2] - afReg[0];
        p.afH = afReg[3] - afReg[1];
    }
    r.getF32(ANDROID_CONTROL_ZOOM_RATIO, &p.zoomRatio);
    int32_t rect[4] = {0, 0, 0, 0};
    if (r.getI32(ANDROID_SCALER_CROP_REGION, rect, 4) && rect[2] > 0 && rect[3] > 0) {
        p.hasCropRegion = true;
        p.cropX = rect[0]; p.cropY = rect[1];
        p.cropW = rect[2]; p.cropH = rect[3];
    }
    r.getI64(ANDROID_SENSOR_EXPOSURE_TIME, &p.manualExposureNs);
    r.getI32(ANDROID_SENSOR_SENSITIVITY, &p.manualSensitivity);
    r.getU8(ANDROID_JPEG_QUALITY, &p.jpegQuality);
    r.getI32(ANDROID_JPEG_ORIENTATION, &p.jpegOrientation);
    return p;
}

void AidlCameraDeviceSession::stepAutofocus(const IspRequestParams& p, const RawFrameView& frame) {
    if (!af_.enabled) {
        af_.moving = false;
        af_.afState = ANDROID_CONTROL_AF_STATE_INACTIVE;
        return;
    }

    ++af_.frameCounter;
    const bool isCaf = (p.afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE ||
                        p.afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO);
    const bool isTriggerMode = (p.afMode == ANDROID_CONTROL_AF_MODE_AUTO ||
                                p.afMode == ANDROID_CONTROL_AF_MODE_MACRO);

    if (p.afMode == ANDROID_CONTROL_AF_MODE_OFF) {
        af_.scanning = false;
        af_.moving = false;
        af_.locked = false;
        af_.activeTrigger = false;
        af_.afState = ANDROID_CONTROL_AF_STATE_INACTIVE;
        float maxD = std::max(1.0f, staticCfg_.minFocusDistance);
        int32_t target = af_.lensMin + static_cast<int32_t>(
            std::lround(std::clamp(p.focusDistance / maxD, 0.0f, 1.0f) * (af_.lensMax - af_.lensMin)));
        if (target != af_.curPos && pipeline_.setFocusPos(target)) {
            af_.curPos = target;
            af_.moving = true;
        }
        af_.prevTrigger = p.afTrigger;
        return;
    }

    if (p.afTrigger == ANDROID_CONTROL_AF_TRIGGER_CANCEL &&
        af_.prevTrigger != ANDROID_CONTROL_AF_TRIGGER_CANCEL) {
        af_.scanning = false;
        af_.moving = false;
        af_.locked = false;
        af_.activeTrigger = false;
        af_.pendingPos = -1;
        af_.afState = isCaf ? ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED : ANDROID_CONTROL_AF_STATE_INACTIVE;
        af_.prevTrigger = p.afTrigger;
        return;
    }

    auto startSweep = [&](bool activeTrig) {
        af_.scanning = true;
        af_.activeTrigger = activeTrig;
        af_.locked = false;
        af_.finePass = false;
        af_.bestScore = -1.0;
        af_.bestPos = af_.curPos;
        af_.scanIdx = 0;
        af_.sceneShiftCount = 0;
        static constexpr int kCoarse[8] = {35, 95, 155, 215, 275, 335, 405, 475};
        af_.candidates.clear();
        for (int pos : kCoarse) {
            af_.candidates.push_back(std::clamp(pos, af_.lensMin, af_.lensMax));
        }
        int32_t firstPos = af_.candidates[0];
        pipeline_.setFocusPos(firstPos);
        af_.curPos = firstPos;
        af_.pendingPos = firstPos;
        af_.settleHold = 1;
        af_.moving = true;
        af_.afState = activeTrig ? ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN
                                 : ANDROID_CONTROL_AF_STATE_PASSIVE_SCAN;
    };

    const bool triggerStarted = (p.afTrigger == ANDROID_CONTROL_AF_TRIGGER_START &&
                                 af_.prevTrigger != ANDROID_CONTROL_AF_TRIGGER_START);
    af_.prevTrigger = p.afTrigger;

    if (triggerStarted && (isTriggerMode || isCaf)) {
        if (isCaf && !af_.scanning && af_.initSweepDone && !p.hasAfRegion) {
            af_.locked = true;
            af_.moving = false;
            af_.afState = ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
            return;
        }
        startSweep(true);
        return;
    }

    if (!af_.scanning && isCaf && !af_.locked && !af_.initSweepDone && af_.frameCounter >= 12) {
        af_.initSweepDone = true;
        startSweep(false);
        return;
    }

    const double score = computeFocusSharpness(
        frame.data, frame.width, frame.height, frame.strideBytes,
        staticCfg_.activeWidth, staticCfg_.activeHeight, nodes_.pattern, p);

    if (af_.scanning) {
        if (af_.settleHold > 0) {
            --af_.settleHold;
            return;
        }
        if (score > af_.bestScore) {
            af_.bestScore = score;
            af_.bestPos = af_.pendingPos;
        }
        ++af_.scanIdx;
        if (af_.scanIdx < af_.candidates.size()) {
            int32_t nextPos = af_.candidates[af_.scanIdx];
            pipeline_.setFocusPos(nextPos);
            af_.curPos = nextPos;
            af_.pendingPos = nextPos;
            af_.settleHold = 1;
            af_.moving = true;
            af_.afState = af_.activeTrigger ? ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN
                                            : ANDROID_CONTROL_AF_STATE_PASSIVE_SCAN;
            return;
        }
        if (!af_.finePass) {
            af_.finePass = true;
            af_.candidates = {
                std::clamp(af_.bestPos - 28, af_.lensMin, af_.lensMax),
                std::clamp(af_.bestPos + 28, af_.lensMin, af_.lensMax),
            };
            af_.scanIdx = 0;
            int32_t nextPos = af_.candidates[0];
            pipeline_.setFocusPos(nextPos);
            af_.curPos = nextPos;
            af_.pendingPos = nextPos;
            af_.settleHold = 1;
            af_.moving = true;
            return;
        }

        // Peak found; move lens cleanly to best position and lock/converge
        pipeline_.setFocusPos(af_.bestPos);
        af_.curPos = af_.bestPos;
        af_.scanning = false;
        af_.moving = false;
        af_.initSweepDone = true;
        af_.refScore = std::max(af_.bestScore, 1e-6);
        af_.postLockSettle = 2;
        af_.sceneShiftCount = 0;
        af_.locked = af_.activeTrigger;
        af_.afState = af_.activeTrigger ? ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED
                                        : ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED;
        ALOGI("Camera %d AF converged: pos=%d score=%.4f (active=%d)",
              nodes_.cameraId, af_.bestPos, af_.bestScore, af_.activeTrigger ? 1 : 0);
        return;
    }

    af_.moving = false;
    if (af_.postLockSettle > 0) {
        if (--af_.postLockSettle == 0) {
            af_.refScore = std::max(score, 1e-6);
            af_.sceneShiftCount = 0;
        }
    } else if (isCaf && !af_.locked && af_.initSweepDone && af_.refScore > 0.0) {
        if (score < af_.refScore * 0.65 || score > af_.refScore * 1.55) {
            if (++af_.sceneShiftCount >= 10) {
                startSweep(false);
                return;
            }
        } else {
            af_.sceneShiftCount = 0;
        }
        af_.afState = ANDROID_CONTROL_AF_STATE_PASSIVE_FOCUSED;
    } else if (af_.locked) {
        af_.afState = ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
    } else if (isTriggerMode) {
        af_.afState = af_.initSweepDone ? ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED
                                        : ANDROID_CONTROL_AF_STATE_INACTIVE;
    }
}

AidlCameraMetadata AidlCameraDeviceSession::buildResultMetadata(const IspRequestParams& p,
                                                                const AeAwbResult& ae,
                                                                int64_t timestampNs) const {
    MetaBuilder b(48, 512);
    b.addU8(ANDROID_COLOR_CORRECTION_ABERRATION_MODE, ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF);
    b.addU8(ANDROID_CONTROL_AE_ANTIBANDING_MODE, ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO);
    b.addI32(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, p.aeExposureComp);
    b.addU8(ANDROID_CONTROL_AE_LOCK,
            p.aeLock ? ANDROID_CONTROL_AE_LOCK_ON : ANDROID_CONTROL_AE_LOCK_OFF);
    b.addU8(ANDROID_CONTROL_AE_MODE, p.aeMode);
    b.addU8(ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER, ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE);
    b.addU8(ANDROID_CONTROL_AE_STATE, ae.aeState);
    const int32_t fpsRange[] = {15, 30};
    b.add(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange, 2);

    b.addU8(ANDROID_CONTROL_AF_MODE, staticCfg_.hasAutofocus ? p.afMode : ANDROID_CONTROL_AF_MODE_OFF);
    b.addU8(ANDROID_CONTROL_AF_STATE, staticCfg_.hasAutofocus ? af_.afState : ANDROID_CONTROL_AF_STATE_INACTIVE);
    b.addU8(ANDROID_CONTROL_AF_TRIGGER, ANDROID_CONTROL_AF_TRIGGER_IDLE);
    b.addU8(ANDROID_CONTROL_AWB_LOCK,
            p.awbLock ? ANDROID_CONTROL_AWB_LOCK_ON : ANDROID_CONTROL_AWB_LOCK_OFF);
    b.addU8(ANDROID_CONTROL_AWB_MODE, p.awbMode);
    b.addU8(ANDROID_CONTROL_AWB_STATE, ae.awbState);
    b.addU8(ANDROID_CONTROL_CAPTURE_INTENT, ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW);
    b.addU8(ANDROID_CONTROL_EFFECT_MODE, ANDROID_CONTROL_EFFECT_MODE_OFF);
    b.addU8(ANDROID_CONTROL_MODE, p.controlMode);
    b.addU8(ANDROID_CONTROL_SCENE_MODE, ANDROID_CONTROL_SCENE_MODE_DISABLED);
    b.addU8(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE, ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF);
    b.addF32(ANDROID_CONTROL_ZOOM_RATIO, p.zoomRatio);

    b.addU8(ANDROID_EDGE_MODE, ANDROID_EDGE_MODE_FAST);
    b.addU8(ANDROID_FLASH_MODE, staticCfg_.hasFlash ? p.flashMode : ANDROID_FLASH_MODE_OFF);
    b.addU8(ANDROID_FLASH_STATE, staticCfg_.hasFlash ? lastFlashState_ : ANDROID_FLASH_STATE_UNAVAILABLE);
    b.addI32(ANDROID_JPEG_ORIENTATION, p.jpegOrientation);
    b.addU8(ANDROID_JPEG_QUALITY, p.jpegQuality);

    float focusDiopters = 0.0f;
    if (staticCfg_.hasAutofocus && af_.lensMax > af_.lensMin) {
        focusDiopters = staticCfg_.minFocusDistance *
                        static_cast<float>(af_.curPos - af_.lensMin) /
                        static_cast<float>(af_.lensMax - af_.lensMin);
    }
    b.addF32(ANDROID_LENS_APERTURE, staticCfg_.apertureF);
    b.addF32(ANDROID_LENS_FOCAL_LENGTH, staticCfg_.focalLengthMm);
    b.addF32(ANDROID_LENS_FOCUS_DISTANCE, focusDiopters);
    b.addU8(ANDROID_LENS_STATE, (staticCfg_.hasAutofocus && af_.moving)
                                    ? ANDROID_LENS_STATE_MOVING
                                    : ANDROID_LENS_STATE_STATIONARY);
    b.addU8(ANDROID_NOISE_REDUCTION_MODE, ANDROID_NOISE_REDUCTION_MODE_FAST);

    const int32_t crop[] = {
        p.hasCropRegion ? p.cropX : 0,
        p.hasCropRegion ? p.cropY : 0,
        p.hasCropRegion ? p.cropW : staticCfg_.activeWidth,
        p.hasCropRegion ? p.cropH : staticCfg_.activeHeight,
    };
    b.add(ANDROID_SCALER_CROP_REGION, crop, 4);

    b.addI64(ANDROID_SENSOR_EXPOSURE_TIME, ae.appliedExposureNs);
    b.addI64(ANDROID_SENSOR_FRAME_DURATION, 33333333LL);
    b.addI32(ANDROID_SENSOR_SENSITIVITY, ae.appliedSensitivity);
    b.addI64(ANDROID_SENSOR_TIMESTAMP, timestampNs);
    b.addU8(ANDROID_STATISTICS_FACE_DETECT_MODE, ANDROID_STATISTICS_FACE_DETECT_MODE_OFF);
    b.addU8(ANDROID_STATISTICS_SCENE_FLICKER, ANDROID_STATISTICS_SCENE_FLICKER_NONE);
    return b.buildAidl();
}

// ---------------- AidlCameraDevice ----------------

AidlCameraDevice::AidlCameraDevice(const V4l2DeviceNodes& nodes)
    : nodes_(nodes), staticCfg_(makeStaticConfig(nodes)) {}

std::string AidlCameraDevice::deviceName() const {
    return "device@1.3/internal/" + std::to_string(nodes_.cameraId);
}

CameraStaticConfig AidlCameraDevice::makeStaticConfig(const V4l2DeviceNodes& n) {
    CameraStaticConfig c{};
    c.cameraId = n.cameraId;
    c.isFront = n.isFront;
    c.activeWidth = n.fullWidth;
    c.activeHeight = n.fullHeight;
    c.sensorOrientation = n.isFront ? 270 : 90;
    c.physicalWidthMm = n.isFront ? 3.49f : 5.96f;
    c.physicalHeightMm = n.isFront ? 2.63f : 3.36f;
    c.focalLengthMm = n.isFront ? 2.2f : 4.3f;
    c.apertureF = 1.9f;
    c.maxJpegSize = n.isFront ? (4 * 1024 * 1024) : (12 * 1024 * 1024);
    c.hasFlash = !n.isFront && (access("/sys/class/camera/flash/rear_torch_flash", F_OK) == 0);
    c.hasAutofocus = !n.isFront && !n.lensSubdev.empty();
    c.minFocusDistance = c.hasAutofocus ? 10.0f : 0.0f;

    auto addFmtSet = [&](int32_t w, int32_t h, bool jpegOnly) {
        if (w > c.activeWidth || h > c.activeHeight) return;
        c.streams.push_back({static_cast<int32_t>(PixelFormat::BLOB), w, h, 33333333LL, 200000000LL});
        if (!jpegOnly) {
            c.streams.push_back(
                {static_cast<int32_t>(PixelFormat::YCBCR_420_888), w, h, 33333333LL, 0LL});
            c.streams.push_back(
                {static_cast<int32_t>(PixelFormat::IMPLEMENTATION_DEFINED), w, h, 33333333LL, 0LL});
        }
    };

    if (n.isFront) {
        addFmtSet(2592, 1944, true);
        addFmtSet(2560, 1920, true);
        addFmtSet(1920, 1080, false);
        addFmtSet(1440, 1080, false);
        addFmtSet(1280, 720, false);
        addFmtSet(960, 720, false);
        addFmtSet(640, 480, false);
        addFmtSet(320, 240, false);
    } else {
        addFmtSet(5312, 2988, true);
        addFmtSet(4000, 3000, true);
        addFmtSet(3264, 2448, true);
        addFmtSet(2560, 1440, false);
        addFmtSet(1920, 1080, false);
        addFmtSet(1440, 1080, false);
        addFmtSet(1280, 720, false);
        addFmtSet(960, 720, false);
        addFmtSet(640, 480, false);
        addFmtSet(320, 240, false);
    }
    return c;
}

ScopedAStatus AidlCameraDevice::getCameraCharacteristics(AidlCameraMetadata* _aidl_return) {
    *_aidl_return = buildStaticCharacteristics(staticCfg_);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::getPhysicalCameraCharacteristics(const std::string& /*id*/,
                                                                 AidlCameraMetadata* /*out*/) {
    return serviceErr(CamStatus::ILLEGAL_ARGUMENT, "Not a logical multi-camera");
}

ScopedAStatus AidlCameraDevice::getResourceCost(CameraResourceCost* _aidl_return) {
    CameraResourceCost rc{};
    rc.resourceCost = 100;
    *_aidl_return = std::move(rc);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::isStreamCombinationSupported(const StreamConfiguration& in_streams,
                                                             bool* _aidl_return) {
    *_aidl_return = isValidStreamConfig(staticCfg_, in_streams);
    return ScopedAStatus::ok();
}

void AidlCameraDevice::setTorchStatusCallback(TorchStatusCb cb) {
    std::lock_guard<std::mutex> lk(torchMutex_);
    torchCb_ = std::move(cb);
}

void AidlCameraDevice::onSessionClosed() {
    TorchStatusCb cb;
    {
        std::lock_guard<std::mutex> lk(torchMutex_);
        if (!staticCfg_.hasFlash) return;
        torchOn_ = false;
        cb = torchCb_;
    }
    if (cb) cb(deviceName(), true, false);
}

ScopedAStatus AidlCameraDevice::open(const std::shared_ptr<ICameraDeviceCallback>& in_callback,
                                     std::shared_ptr<ICameraDeviceSession>* _aidl_return) {
    if (!in_callback) {
        return serviceErr(CamStatus::ILLEGAL_ARGUMENT, "Null callback");
    }
    std::lock_guard<std::mutex> lk(openMutex_);
    if (auto existing = activeSession_.lock()) {
        existing->close();
        activeSession_.reset();
    }

    if (staticCfg_.hasFlash) {
        TorchStatusCb cb;
        {
            std::lock_guard<std::mutex> tlk(torchMutex_);
            if (torchOn_) {
                setTorchSysfs(false);
                torchOn_ = false;
            }
            cb = torchCb_;
        }
        if (cb) cb(deviceName(), false, false);
    }

    auto session = ::ndk::SharedRefBase::make<AidlCameraDeviceSession>(
        nodes_, staticCfg_, in_callback, [this]() { onSessionClosed(); });
    if (!session->initialize()) {
        onSessionClosed();
        return serviceErr(CamStatus::INTERNAL_ERROR, "Failed to initialize V4L2 camera session");
    }
    activeSession_ = session;
    *_aidl_return = session;
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::openInjectionSession(
    const std::shared_ptr<ICameraDeviceCallback>& /*cb*/,
    std::shared_ptr<ICameraInjectionSession>* _aidl_return) {
    *_aidl_return = nullptr;
    return serviceErr(CamStatus::OPERATION_NOT_SUPPORTED, "Injection not supported");
}

ScopedAStatus AidlCameraDevice::setTorchMode(bool on) {
    if (!staticCfg_.hasFlash) {
        return serviceErr(CamStatus::OPERATION_NOT_SUPPORTED, "Camera has no flash unit");
    }
    {
        std::lock_guard<std::mutex> olk(openMutex_);
        if (activeSession_.lock()) {
            return serviceErr(CamStatus::CAMERA_IN_USE, "Camera device in use");
        }
    }
    TorchStatusCb cb;
    {
        std::lock_guard<std::mutex> tlk(torchMutex_);
        if (!setTorchSysfs(on, on ? torchLevel_ : 0)) {
            return serviceErr(CamStatus::INTERNAL_ERROR, "Failed to control flash sysfs");
        }
        torchOn_ = on;
        cb = torchCb_;
    }
    if (cb) cb(deviceName(), true, on);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::turnOnTorchWithStrengthLevel(int32_t level) {
    if (!staticCfg_.hasFlash) {
        return serviceErr(CamStatus::OPERATION_NOT_SUPPORTED, "Camera has no flash unit");
    }
    if (level < 1 || level > 15) {
        return serviceErr(CamStatus::ILLEGAL_ARGUMENT, "Invalid torch strength level");
    }
    {
        std::lock_guard<std::mutex> olk(openMutex_);
        if (activeSession_.lock()) {
            return serviceErr(CamStatus::CAMERA_IN_USE, "Camera device in use");
        }
    }
    TorchStatusCb cb;
    {
        std::lock_guard<std::mutex> tlk(torchMutex_);
        if (!setTorchSysfs(true, level)) {
            return serviceErr(CamStatus::INTERNAL_ERROR, "Failed to control flash sysfs");
        }
        torchLevel_ = level;
        torchOn_ = true;
        cb = torchCb_;
    }
    if (cb) cb(deviceName(), true, true);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::getTorchStrengthLevel(int32_t* _aidl_return) {
    if (!staticCfg_.hasFlash) {
        *_aidl_return = 0;
        return serviceErr(CamStatus::OPERATION_NOT_SUPPORTED, "Camera has no flash unit");
    }
    std::lock_guard<std::mutex> tlk(torchMutex_);
    *_aidl_return = torchOn_ ? torchLevel_ : 0;
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::constructDefaultRequestSettings(RequestTemplate in_type,
                                                                AidlCameraMetadata* _aidl_return) {
    *_aidl_return = buildDefaultRequest(staticCfg_, in_type);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::isStreamCombinationWithSettingsSupported(
    const StreamConfiguration& in_streams, bool* _aidl_return) {
    *_aidl_return = isValidStreamConfig(staticCfg_, in_streams);
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraDevice::getSessionCharacteristics(
    const StreamConfiguration& in_sessionConfig, AidlCameraMetadata* _aidl_return) {
    if (!isValidStreamConfig(staticCfg_, in_sessionConfig)) {
        return serviceErr(CamStatus::ILLEGAL_ARGUMENT, "Invalid session stream configuration");
    }
    *_aidl_return = buildSessionCharacteristics(staticCfg_);
    return ScopedAStatus::ok();
}

}  // namespace s6cam
