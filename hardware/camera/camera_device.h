#pragma once

#include "camera_metadata_helper.h"
#include "gralloc_bridge.h"
#include "soft_isp.h"
#include "v4l2_pipeline.h"

#include <aidl/android/hardware/camera/device/BnCameraDevice.h>
#include <aidl/android/hardware/camera/device/BnCameraDeviceSession.h>
#include <aidl/android/hardware/camera/device/ICameraDeviceCallback.h>
#include <fmq/AidlMessageQueue.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace s6cam {

using ::aidl::android::hardware::camera::common::CameraResourceCost;
using ::aidl::android::hardware::camera::device::BnCameraDevice;
using ::aidl::android::hardware::camera::device::BnCameraDeviceSession;
using ::aidl::android::hardware::camera::device::BufferCache;
using ::aidl::android::hardware::camera::device::CameraOfflineSessionInfo;
using ::aidl::android::hardware::camera::device::CaptureRequest;
using ::aidl::android::hardware::camera::device::CaptureResult;
using ::aidl::android::hardware::camera::device::ConfigureStreamsRet;
using ::aidl::android::hardware::camera::device::HalStream;
using ::aidl::android::hardware::camera::device::ICameraDeviceCallback;
using ::aidl::android::hardware::camera::device::ICameraDeviceSession;
using ::aidl::android::hardware::camera::device::ICameraInjectionSession;
using ::aidl::android::hardware::camera::device::ICameraOfflineSession;
using ::aidl::android::hardware::camera::device::Stream;
using ::aidl::android::hardware::camera::device::StreamConfiguration;
using ::aidl::android::hardware::common::fmq::MQDescriptor;
using ::aidl::android::hardware::common::fmq::SynchronizedReadWrite;
using ::ndk::ScopedAStatus;

using MetadataQueue = ::android::AidlMessageQueue<int8_t, SynchronizedReadWrite>;

class AidlCameraDeviceSession : public BnCameraDeviceSession {
public:
    AidlCameraDeviceSession(const V4l2DeviceNodes& nodes,
                            const CameraStaticConfig& staticCfg,
                            std::shared_ptr<ICameraDeviceCallback> callback,
                            std::function<void()> onCloseCb = nullptr);
    ~AidlCameraDeviceSession() override;

    bool initialize();

    ScopedAStatus close() override;
    ScopedAStatus configureStreams(const StreamConfiguration& in_requestedConfiguration,
                                   std::vector<HalStream>* _aidl_return) override;
    ScopedAStatus constructDefaultRequestSettings(RequestTemplate in_type,
                                                  AidlCameraMetadata* _aidl_return) override;
    ScopedAStatus flush() override;
    ScopedAStatus getCaptureRequestMetadataQueue(
        MQDescriptor<int8_t, SynchronizedReadWrite>* _aidl_return) override;
    ScopedAStatus getCaptureResultMetadataQueue(
        MQDescriptor<int8_t, SynchronizedReadWrite>* _aidl_return) override;
    ScopedAStatus isReconfigurationRequired(const AidlCameraMetadata& in_oldSessionParams,
                                            const AidlCameraMetadata& in_newSessionParams,
                                            bool* _aidl_return) override;
    ScopedAStatus processCaptureRequest(const std::vector<CaptureRequest>& in_requests,
                                        const std::vector<BufferCache>& in_cachesToRemove,
                                        int32_t* _aidl_return) override;
    ScopedAStatus signalStreamFlush(const std::vector<int32_t>& in_streamIds,
                                    int32_t in_streamConfigCounter) override;
    ScopedAStatus switchToOffline(const std::vector<int32_t>& in_streamsToKeep,
                                  CameraOfflineSessionInfo* out_offlineSessionInfo,
                                  std::shared_ptr<ICameraOfflineSession>* _aidl_return) override;
    ScopedAStatus repeatingRequestEnd(int32_t in_frameNumber,
                                      const std::vector<int32_t>& in_streamIds) override;
    ScopedAStatus configureStreamsV2(const StreamConfiguration& in_requestedConfiguration,
                                     ConfigureStreamsRet* _aidl_return) override;

private:
    struct ActiveStream {
        int32_t id = -1;
        int32_t width = 0;
        int32_t height = 0;
        int32_t format = 0;  // Effective HAL_PIXEL_FORMAT_*
    };

    struct QueuedBuf {
        int32_t streamId = -1;
        int64_t bufferId = 0;
        native_handle_t* handle = nullptr;
        int acquireFenceFd = -1;
    };

    struct QueuedReq {
        int32_t frameNumber = 0;
        std::vector<uint8_t> settingsBytes;
        std::vector<QueuedBuf> outputs;
    };

    struct AfState {
        bool enabled = false;
        int32_t lensMin = 0;
        int32_t lensMax = 511;
        int32_t curPos = 180;
        int32_t bestPos = 180;
        double bestScore = -1.0;
        bool scanning = false;
        bool activeTrigger = false;
        bool locked = false;
        bool finePass = false;
        bool initSweepDone = false;
        size_t scanIdx = 0;
        std::vector<int32_t> candidates;
        int32_t pendingPos = -1;
        int32_t settleHold = 0;
        double refScore = 0.0;
        int32_t postLockSettle = 0;
        int32_t sceneShiftCount = 0;
        int32_t frameCounter = 0;
        uint8_t afState = 0;
        uint8_t prevTrigger = 0;
        bool moving = false;
    };

    void workerLoop();
    void processSingleRequest(const QueuedReq& req, bool isErrorFlush);
    IspRequestParams parseRequestParams(const std::vector<uint8_t>& settingsBytes) const;
    void stepAutofocus(const IspRequestParams& params, const RawFrameView& frame);
    AidlCameraMetadata buildResultMetadata(const IspRequestParams& params,
                                           const AeAwbResult& aeAwb,
                                           int64_t timestampNs) const;
    void returnQueuedAsError(const QueuedReq& req);

    V4l2DeviceNodes nodes_;
    CameraStaticConfig staticCfg_;
    std::shared_ptr<ICameraDeviceCallback> callback_;
    std::function<void()> onCloseCb_;

    V4l2Pipeline pipeline_;
    GrallocBridge gralloc_;
    SoftIsp isp_;
    AeAwbController aeAwb_;
    SensorCtrlLimits sensorLimits_{};
    AfState af_{};
    bool flashTorchActive_ = false;
    uint8_t lastFlashState_ = 0;

    std::unique_ptr<MetadataQueue> requestQueue_;
    std::unique_ptr<MetadataQueue> resultQueue_;

    std::mutex stateMutex_;
    std::unordered_map<int32_t, ActiveStream> activeStreams_;
    std::vector<uint8_t> lastSettingsBytes_;
    bool closed_ = false;

    std::mutex reqMutex_;
    std::condition_variable reqCv_;
    std::condition_variable idleCv_;
    std::deque<QueuedReq> reqDeque_;
    bool stopWorker_ = false;
    bool busyWorker_ = false;
    std::atomic<bool> flushing_{false};
    std::thread workerThread_;

    std::mutex callbackMutex_;
    std::mutex resultQueueMutex_;
};

class AidlCameraDevice : public BnCameraDevice {
public:
    using TorchStatusCb = std::function<void(const std::string& name, bool available, bool on)>;

    explicit AidlCameraDevice(const V4l2DeviceNodes& nodes);
    ~AidlCameraDevice() override = default;

    std::string deviceName() const;
    const CameraStaticConfig& staticConfig() const { return staticCfg_; }
    void setTorchStatusCallback(TorchStatusCb cb);

    ScopedAStatus getCameraCharacteristics(AidlCameraMetadata* _aidl_return) override;
    ScopedAStatus getPhysicalCameraCharacteristics(const std::string& in_physicalCameraId,
                                                   AidlCameraMetadata* _aidl_return) override;
    ScopedAStatus getResourceCost(CameraResourceCost* _aidl_return) override;
    ScopedAStatus isStreamCombinationSupported(const StreamConfiguration& in_streams,
                                               bool* _aidl_return) override;
    ScopedAStatus open(const std::shared_ptr<ICameraDeviceCallback>& in_callback,
                       std::shared_ptr<ICameraDeviceSession>* _aidl_return) override;
    ScopedAStatus openInjectionSession(
        const std::shared_ptr<ICameraDeviceCallback>& in_callback,
        std::shared_ptr<ICameraInjectionSession>* _aidl_return) override;
    ScopedAStatus setTorchMode(bool in_on) override;
    ScopedAStatus turnOnTorchWithStrengthLevel(int32_t in_torchStrength) override;
    ScopedAStatus getTorchStrengthLevel(int32_t* _aidl_return) override;
    ScopedAStatus constructDefaultRequestSettings(RequestTemplate in_type,
                                                  AidlCameraMetadata* _aidl_return) override;
    ScopedAStatus isStreamCombinationWithSettingsSupported(const StreamConfiguration& in_streams,
                                                           bool* _aidl_return) override;
    ScopedAStatus getSessionCharacteristics(const StreamConfiguration& in_sessionConfig,
                                            AidlCameraMetadata* _aidl_return) override;

private:
    CameraStaticConfig makeStaticConfig(const V4l2DeviceNodes& nodes);
    void onSessionClosed();

    V4l2DeviceNodes nodes_;
    CameraStaticConfig staticCfg_;
    std::mutex openMutex_;
    std::weak_ptr<AidlCameraDeviceSession> activeSession_;
    std::mutex torchMutex_;
    TorchStatusCb torchCb_;
    bool torchOn_ = false;
    int32_t torchLevel_ = 6;
};

}  // namespace s6cam
