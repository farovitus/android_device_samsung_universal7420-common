#pragma once

#include "camera_device.h"

#include <aidl/android/hardware/camera/provider/BnCameraProvider.h>
#include <aidl/android/hardware/camera/provider/ICameraProviderCallback.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace s6cam {

using ::aidl::android::hardware::camera::common::VendorTagSection;
using ::aidl::android::hardware::camera::device::ICameraDevice;
using ::aidl::android::hardware::camera::provider::BnCameraProvider;
using ::aidl::android::hardware::camera::provider::CameraIdAndStreamCombination;
using ::aidl::android::hardware::camera::provider::ConcurrentCameraIdCombination;
using ::aidl::android::hardware::camera::provider::ICameraProviderCallback;
using ::ndk::ScopedAStatus;

class AidlCameraProvider : public BnCameraProvider {
public:
    AidlCameraProvider() = default;
    ~AidlCameraProvider() override = default;

    bool init();

    ScopedAStatus setCallback(const std::shared_ptr<ICameraProviderCallback>& in_callback) override;
    ScopedAStatus getVendorTags(std::vector<VendorTagSection>* _aidl_return) override;
    ScopedAStatus getCameraIdList(std::vector<std::string>* _aidl_return) override;
    ScopedAStatus getCameraDeviceInterface(const std::string& in_cameraDeviceName,
                                           std::shared_ptr<ICameraDevice>* _aidl_return) override;
    ScopedAStatus notifyDeviceStateChange(int64_t in_deviceState) override;
    ScopedAStatus getConcurrentCameraIds(
        std::vector<ConcurrentCameraIdCombination>* _aidl_return) override;
    ScopedAStatus isConcurrentStreamCombinationSupported(
        const std::vector<CameraIdAndStreamCombination>& in_configs,
        bool* _aidl_return) override;

private:
    std::mutex mutex_;
    std::shared_ptr<ICameraProviderCallback> callback_;
    std::vector<std::string> cameraNames_;
    std::unordered_map<std::string, std::shared_ptr<AidlCameraDevice>> devices_;
};

}  // namespace s6cam
