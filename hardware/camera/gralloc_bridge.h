#pragma once

#include <aidl/android/hardware/camera/device/StreamBuffer.h>
#include <aidl/android/hardware/common/NativeHandle.h>
#include <cutils/native_handle.h>
#include <hardware/gralloc.h>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace s6cam {

using AidlNativeHandle = ::aidl::android::hardware::common::NativeHandle;
using ::aidl::android::hardware::camera::device::StreamBuffer;

struct LockedOutput {
    int32_t width = 0;
    int32_t height = 0;
    int32_t stride = 0;
    int32_t bufferSize = 0;
    int32_t format = 0;
    void* vaddr = nullptr;
    android_ycbcr ycbcr{};
    bool isYcbcr = false;
};

/**
 * Bridges AIDL NativeHandle buffers to Exynos 7420's s6_drm_gralloc module,
 * managing handle caching, registerBuffer/unregisterBuffer, fence waits, and lock/unlock.
 */
class GrallocBridge {
public:
    GrallocBridge();
    ~GrallocBridge();

    GrallocBridge(const GrallocBridge&) = delete;
    GrallocBridge& operator=(const GrallocBridge&) = delete;

    bool init();

    /**
     * Imports (dups) an AIDL buffer handle on first sight or retrieves the cached one.
     */
    native_handle_t* getOrImportLocked(int32_t streamId, int64_t bufferId,
                                        const AidlNativeHandle& aidlHandle);

    void removeCached(int32_t streamId, int64_t bufferId);
    void removeStream(int32_t streamId);
    void clearAll();

    static int waitAndCloseFence(const AidlNativeHandle& fence, int timeoutMs = 1500);

    bool lockForWrite(native_handle_t* hnd, int32_t width, int32_t height, int32_t format,
                      LockedOutput* out);
    void unlock(native_handle_t* hnd);

private:
    struct BufKey {
        int32_t streamId;
        int64_t bufferId;
        bool operator==(const BufKey& o) const {
            return streamId == o.streamId && bufferId == o.bufferId;
        }
    };
    struct BufKeyHash {
        size_t operator()(const BufKey& k) const {
            return std::hash<int64_t>{}((static_cast<int64_t>(k.streamId) << 32) ^ k.bufferId);
        }
    };

    void freeHandle(native_handle_t* hnd);

    void* dlHandle_ = nullptr;
    const gralloc_module_t* gralloc_ = nullptr;
    std::mutex mutex_;
    std::unordered_map<BufKey, native_handle_t*, BufKeyHash> cache_;
};

}  // namespace s6cam
