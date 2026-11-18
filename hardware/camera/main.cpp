#define LOG_TAG "android.hardware.camera.provider-V3-service.s6"

#include "camera_provider.h"

#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <android/binder_stability.h>
#include <log/log.h>
#include <unistd.h>

int main() {
    ALOGI("Starting Exynos7420 V4L2/SoftISP Camera Provider V3 service");

    if (!ABinderProcess_setThreadPoolMaxThreadCount(4)) {
        ALOGW("ABinderProcess_setThreadPoolMaxThreadCount(4) failed");
    }
    ABinderProcess_startThreadPool();

    auto provider = ::ndk::SharedRefBase::make<s6cam::AidlCameraProvider>();
    provider->init();

    ::ndk::SpAIBinder binder = provider->asBinder();
    AIBinder_markVintfStability(binder.get());

    const std::string instance =
        std::string(s6cam::AidlCameraProvider::descriptor) + "/internal/0";
    binder_status_t st = AServiceManager_addService(binder.get(), instance.c_str());
    if (st != STATUS_OK) {
        ALOGE("AServiceManager_addService(%s) failed: %d", instance.c_str(), st);
        return 1;
    }

    ALOGI("Registered AIDL CameraProvider service: %s", instance.c_str());
    ABinderProcess_joinThreadPool();
    return 0;
}
