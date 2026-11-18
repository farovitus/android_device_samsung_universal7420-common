#define LOG_TAG "s6cam_provider"
#include "camera_provider.h"

#include <aidl/android/hardware/camera/common/Status.h>
#include <aidl/android/hardware/camera/common/TorchModeStatus.h>
#include <log/log.h>
#include <chrono>
#include <thread>

namespace s6cam {

namespace {

using CamStatus = ::aidl::android::hardware::camera::common::Status;

ScopedAStatus serviceErr(CamStatus s, const char* msg = "") {
    return ScopedAStatus::fromServiceSpecificErrorWithMessage(static_cast<int32_t>(s), msg);
}

}  // namespace

bool AidlCameraProvider::init() {
    // Kernel camera drivers (especially s5c73c2 companion firmware over SPI) finish
    // probing asynchronously relative to init class_start; wait up to 12s for both pipelines.
    std::vector<V4l2DeviceNodes> nodesList;
    for (int attempt = 0; attempt < 48; ++attempt) {
        nodesList = discoverV4l2Cameras();
        if (nodesList.size() >= 2) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    if (nodesList.empty()) {
        nodesList = discoverV4l2Cameras();
    }
    std::lock_guard<std::mutex> lk(mutex_);
    cameraNames_.clear();
    devices_.clear();

    for (const auto& n : nodesList) {
        auto dev = ::ndk::SharedRefBase::make<AidlCameraDevice>(n);
        std::string name = dev->deviceName();
        dev->setTorchStatusCallback([this](const std::string& dName, bool available, bool on) {
            using ::aidl::android::hardware::camera::common::TorchModeStatus;
            std::shared_ptr<ICameraProviderCallback> cb;
            {
                std::lock_guard<std::mutex> lk(mutex_);
                cb = callback_;
            }
            if (!cb) return;
            TorchModeStatus st = !available
                ? TorchModeStatus::NOT_AVAILABLE
                : (on ? TorchModeStatus::AVAILABLE_ON : TorchModeStatus::AVAILABLE_OFF);
            cb->torchModeStatusChange(dName, st);
        });
        cameraNames_.push_back(name);
        devices_.emplace(name, std::move(dev));
        ALOGI("Registered camera interface: %s (sensor=%s %dx%d)",
              name.c_str(), n.sensorName.c_str(), n.fullWidth, n.fullHeight);
    }

    if (devices_.empty()) {
        ALOGW("No V4L2 camera pipelines discovered yet");
    }
    return true;
}

ScopedAStatus AidlCameraProvider::setCallback(
    const std::shared_ptr<ICameraProviderCallback>& in_callback) {
    if (!in_callback) {
        return serviceErr(CamStatus::ILLEGAL_ARGUMENT, "Null ICameraProviderCallback");
    }
    std::lock_guard<std::mutex> lk(mutex_);
    callback_ = in_callback;
    for (const auto& [name, dev] : devices_) {
        if (dev->staticConfig().hasFlash) {
            using ::aidl::android::hardware::camera::common::TorchModeStatus;
            in_callback->torchModeStatusChange(name, TorchModeStatus::AVAILABLE_OFF);
        }
    }
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraProvider::getVendorTags(std::vector<VendorTagSection>* _aidl_return) {
    _aidl_return->clear();
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraProvider::getCameraIdList(std::vector<std::string>* _aidl_return) {
    std::lock_guard<std::mutex> lk(mutex_);
    *_aidl_return = cameraNames_;
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraProvider::getCameraDeviceInterface(
    const std::string& in_cameraDeviceName, std::shared_ptr<ICameraDevice>* _aidl_return) {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = devices_.find(in_cameraDeviceName);
    if (it == devices_.end()) {
        *_aidl_return = nullptr;
        return serviceErr(CamStatus::ILLEGAL_ARGUMENT, "Unknown camera device name");
    }
    *_aidl_return = it->second;
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraProvider::notifyDeviceStateChange(int64_t /*in_deviceState*/) {
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraProvider::getConcurrentCameraIds(
    std::vector<ConcurrentCameraIdCombination>* _aidl_return) {
    _aidl_return->clear();
    return ScopedAStatus::ok();
}

ScopedAStatus AidlCameraProvider::isConcurrentStreamCombinationSupported(
    const std::vector<CameraIdAndStreamCombination>& /*in_configs*/, bool* _aidl_return) {
    *_aidl_return = false;
    return ScopedAStatus::ok();
}

}  // namespace s6cam
