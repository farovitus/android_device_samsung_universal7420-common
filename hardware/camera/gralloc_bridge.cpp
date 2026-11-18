#define LOG_TAG "s6cam_gralloc"
#include "gralloc_bridge.h"

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <log/log.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>

namespace s6cam {

namespace {

// Mirrors s6_drm_gralloc.cpp layout for reading internal handle dimensions/size cleanly.
struct S6PrivateHandle {
    native_handle_t nativeHandle;
    int fd;
    int magic;
    int flags;
    int size;
    int offset;
    uint64_t base;
    int pid;
    int width;
    int height;
    int format;
    int stride;
};

constexpr int kS6GrallocMagic = 0x3141592;

}  // namespace

GrallocBridge::GrallocBridge() = default;

GrallocBridge::~GrallocBridge() {
    clearAll();
    if (dlHandle_) {
        dlclose(dlHandle_);
        dlHandle_ = nullptr;
        gralloc_ = nullptr;
    }
}

bool GrallocBridge::init() {
    if (gralloc_) return true;

    const char* paths[] = {
        "/vendor/lib64/hw/gralloc.samsungexynos7420.so",
        "/vendor/lib64/hw/gralloc.default.so",
    };
    for (const char* path : paths) {
        dlHandle_ = dlopen(path, RTLD_NOW | RTLD_LOCAL);
        if (!dlHandle_) {
            ALOGW("dlopen(%s) failed: %s", path, dlerror());
            continue;
        }
        auto* hmi = reinterpret_cast<const gralloc_module_t*>(dlsym(dlHandle_, HAL_MODULE_INFO_SYM_AS_STR));
        if (hmi && hmi->common.tag == HARDWARE_MODULE_TAG) {
            gralloc_ = hmi;
            ALOGI("Loaded gralloc module from %s (%s)", path,
                  hmi->common.name ? hmi->common.name : "unknown");
            return true;
        }
        dlclose(dlHandle_);
        dlHandle_ = nullptr;
    }
    ALOGE("Failed to load any vendor gralloc module");
    return false;
}

native_handle_t* GrallocBridge::getOrImportLocked(int32_t streamId, int64_t bufferId,
                                                  const AidlNativeHandle& aidlHandle) {
    if (bufferId == 0) return nullptr;
    std::lock_guard<std::mutex> lk(mutex_);
    BufKey key{streamId, bufferId};
    auto it = cache_.find(key);
    if (it != cache_.end()) {
        return it->second;
    }

    if (aidlHandle.fds.empty() && aidlHandle.ints.empty()) {
        ALOGE("Missing buffer handle on first use for stream=%d bufId=%lld",
              streamId, static_cast<long long>(bufferId));
        return nullptr;
    }

    const int nFds = static_cast<int>(aidlHandle.fds.size());
    const int nInts = static_cast<int>(aidlHandle.ints.size());
    native_handle_t* hnd = native_handle_create(nFds, nInts);
    if (!hnd) return nullptr;

    for (int i = 0; i < nFds; ++i) {
        int srcFd = aidlHandle.fds[i].get();
        int dupFd = (srcFd >= 0) ? fcntl(srcFd, F_DUPFD_CLOEXEC, 0) : -1;
        if (dupFd < 0) {
            ALOGE("fcntl(F_DUPFD_CLOEXEC) failed for fd=%d: %s", srcFd, strerror(errno));
            hnd->numFds = i;
            native_handle_close(hnd);
            native_handle_delete(hnd);
            return nullptr;
        }
        hnd->data[i] = dupFd;
    }
    if (nInts > 0) {
        std::memcpy(&hnd->data[nFds], aidlHandle.ints.data(), nInts * sizeof(int32_t));
    }

    // Clear stale PID/base pointer carried from allocator process before registerBuffer().
    if (nFds == 1 && nInts >= 10) {
        auto* s6h = reinterpret_cast<S6PrivateHandle*>(hnd);
        if (s6h->magic == kS6GrallocMagic) {
            s6h->base = 0;
            s6h->pid = 0;
        }
    }

    if (gralloc_ && gralloc_->registerBuffer) {
        int rc = gralloc_->registerBuffer(gralloc_, hnd);
        if (rc != 0) {
            ALOGE("registerBuffer failed for stream=%d bufId=%lld: %d",
                  streamId, static_cast<long long>(bufferId), rc);
            native_handle_close(hnd);
            native_handle_delete(hnd);
            return nullptr;
        }
    }

    cache_.emplace(key, hnd);
    return hnd;
}

void GrallocBridge::freeHandle(native_handle_t* hnd) {
    if (!hnd) return;
    if (gralloc_ && gralloc_->unregisterBuffer) {
        gralloc_->unregisterBuffer(gralloc_, hnd);
    }
    for (int i = 0; i < hnd->numFds; ++i) {
        if (hnd->data[i] >= 0) {
            close(hnd->data[i]);
            hnd->data[i] = -1;
        }
    }
    native_handle_delete(hnd);
}

void GrallocBridge::removeCached(int32_t streamId, int64_t bufferId) {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = cache_.find(BufKey{streamId, bufferId});
    if (it != cache_.end()) {
        freeHandle(it->second);
        cache_.erase(it);
    }
}

void GrallocBridge::removeStream(int32_t streamId) {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto it = cache_.begin(); it != cache_.end();) {
        if (it->first.streamId == streamId) {
            freeHandle(it->second);
            it = cache_.erase(it);
        } else {
            ++it;
        }
    }
}

void GrallocBridge::clearAll() {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& kv : cache_) {
        freeHandle(kv.second);
    }
    cache_.clear();
}

int GrallocBridge::waitAndCloseFence(const AidlNativeHandle& fence, int timeoutMs) {
    int ret = 0;
    for (const auto& sfd : fence.fds) {
        int fd = sfd.get();
        if (fd < 0) continue;
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, timeoutMs);
        if (pr <= 0) {
            ret = -ETIMEDOUT;
        }
    }
    return ret;
}

bool GrallocBridge::lockForWrite(native_handle_t* hnd, int32_t width, int32_t height, int32_t format,
                                 LockedOutput* out) {
    if (!gralloc_ || !hnd || !out) return false;
    *out = LockedOutput{};
    out->width = width;
    out->height = height;
    out->format = format;
    out->stride = width;

    if (hnd->numFds == 1 && hnd->numInts >= 10) {
        const auto* s6h = reinterpret_cast<const S6PrivateHandle*>(hnd);
        if (s6h->magic == kS6GrallocMagic) {
            out->bufferSize = s6h->size;
            if (s6h->stride > 0) out->stride = s6h->stride;
        }
    }

    const int usage = GRALLOC_USAGE_SW_WRITE_OFTEN;
    if (format == HAL_PIXEL_FORMAT_YV12 ||
        format == HAL_PIXEL_FORMAT_YCrCb_420_SP ||
        format == HAL_PIXEL_FORMAT_YCbCr_420_888) {
        if (!gralloc_->lock_ycbcr) return false;
        int rc = gralloc_->lock_ycbcr(gralloc_, hnd, usage, 0, 0, width, height, &out->ycbcr);
        if (rc != 0 || !out->ycbcr.y) {
            ALOGE("lock_ycbcr(fmt=0x%x %dx%d) failed: %d", format, width, height, rc);
            return false;
        }
        out->isYcbcr = true;
        out->stride = static_cast<int32_t>(out->ycbcr.ystride);
        return true;
    }

    if (!gralloc_->lock) return false;
    int32_t lockW = width;
    int32_t lockH = height;
    if (format == HAL_PIXEL_FORMAT_BLOB && hnd->numFds == 1 && hnd->numInts >= 10) {
        const auto* s6h = reinterpret_cast<const S6PrivateHandle*>(hnd);
        if (s6h->magic == kS6GrallocMagic && s6h->width > 0 && s6h->height > 0) {
            lockW = s6h->width;
            lockH = s6h->height;
        }
    }
    int rc = gralloc_->lock(gralloc_, hnd, usage, 0, 0, lockW, lockH, &out->vaddr);
    if (rc != 0 || !out->vaddr) {
        ALOGE("lock(fmt=0x%x %dx%d) failed: %d", format, lockW, lockH, rc);
        return false;
    }
    return true;
}

void GrallocBridge::unlock(native_handle_t* hnd) {
    if (gralloc_ && gralloc_->unlock && hnd) {
        gralloc_->unlock(gralloc_, hnd);
    }
}

}  // namespace s6cam
