#pragma once

#include <aidl/android/hardware/camera/device/CameraMetadata.h>
#include <aidl/android/hardware/camera/device/RequestTemplate.h>
#include <system/camera_metadata.h>
#include <cstdint>
#include <cstring>
#include <vector>

namespace s6cam {

using AidlCameraMetadata = ::aidl::android::hardware::camera::device::CameraMetadata;
using ::aidl::android::hardware::camera::device::RequestTemplate;

/**
 * RAII wrapper for constructing, compacting, and serializing AOSP camera_metadata_t packets.
 */
class MetaBuilder {
public:
    explicit MetaBuilder(size_t entryCapacity = 128, size_t dataCapacity = 4096) {
        meta_ = allocate_camera_metadata(entryCapacity, dataCapacity);
    }

    ~MetaBuilder() {
        if (meta_) {
            free_camera_metadata(meta_);
            meta_ = nullptr;
        }
    }

    MetaBuilder(const MetaBuilder&) = delete;
    MetaBuilder& operator=(const MetaBuilder&) = delete;

    template <typename T>
    bool add(uint32_t tag, const T* data, size_t count) {
        if (!meta_) return false;
        int rc = add_camera_metadata_entry(meta_, tag, data, count);
        if (rc == 0) return true;
        // Reallocate on overflow with 2x capacity
        size_t ec = get_camera_metadata_entry_capacity(meta_) * 2 + 16;
        size_t dc = get_camera_metadata_data_capacity(meta_) * 2 +
                    calculate_camera_metadata_entry_data_size(get_camera_metadata_tag_type(tag), count);
        camera_metadata_t* next = allocate_camera_metadata(ec, dc);
        if (!next) return false;
        append_camera_metadata(next, meta_);
        free_camera_metadata(meta_);
        meta_ = next;
        return add_camera_metadata_entry(meta_, tag, data, count) == 0;
    }

    template <typename T>
    bool addOne(uint32_t tag, const T& val) {
        return add(tag, &val, 1);
    }

    bool addU8(uint32_t tag, uint8_t val) { return addOne(tag, val); }
    bool addI32(uint32_t tag, int32_t val) { return addOne(tag, val); }
    bool addI64(uint32_t tag, int64_t val) { return addOne(tag, val); }
    bool addF32(uint32_t tag, float val) { return addOne(tag, val); }

    AidlCameraMetadata buildAidl() {
        AidlCameraMetadata out;
        if (!meta_) return out;
        sort_camera_metadata(meta_);
        size_t compactSize = get_camera_metadata_compact_size(meta_);
        out.metadata.resize(compactSize);
        camera_metadata_t* dst = reinterpret_cast<camera_metadata_t*>(out.metadata.data());
        copy_camera_metadata(dst, compactSize, meta_);
        return out;
    }

private:
    camera_metadata_t* meta_ = nullptr;
};

struct StreamSpec {
    int32_t format;
    int32_t width;
    int32_t height;
    int64_t minFrameDurationNs;
    int64_t stallDurationNs;
};

struct CameraStaticConfig {
    int32_t cameraId;
    bool isFront;
    int32_t activeWidth;
    int32_t activeHeight;
    int32_t sensorOrientation;
    float physicalWidthMm;
    float physicalHeightMm;
    float focalLengthMm;
    float apertureF;
    int32_t maxJpegSize;
    bool hasFlash;
    bool hasAutofocus;
    float minFocusDistance;
    std::vector<StreamSpec> streams;
};

/**
 * Helper for querying tags cleanly inside a validated camera_metadata_t buffer.
 */
class MetaReader {
public:
    explicit MetaReader(const std::vector<uint8_t>& bytes) {
        if (bytes.size() >= sizeof(uint32_t)) {
            auto* raw = reinterpret_cast<const camera_metadata_t*>(bytes.data());
            size_t sz = bytes.size();
            if (validate_camera_metadata_structure(raw, &sz) == 0) {
                meta_ = raw;
            }
        }
    }

    bool valid() const { return meta_ != nullptr; }

    bool getU8(uint32_t tag, uint8_t* out) const {
        camera_metadata_ro_entry_t e{};
        if (!meta_ || find_camera_metadata_ro_entry(meta_, tag, &e) != 0 || e.count < 1)
            return false;
        *out = e.data.u8[0];
        return true;
    }

    bool getI32(uint32_t tag, int32_t* out, size_t count = 1) const {
        camera_metadata_ro_entry_t e{};
        if (!meta_ || find_camera_metadata_ro_entry(meta_, tag, &e) != 0 || e.count < count)
            return false;
        std::memcpy(out, e.data.i32, count * sizeof(int32_t));
        return true;
    }

    bool getI64(uint32_t tag, int64_t* out) const {
        camera_metadata_ro_entry_t e{};
        if (!meta_ || find_camera_metadata_ro_entry(meta_, tag, &e) != 0 || e.count < 1)
            return false;
        *out = e.data.i64[0];
        return true;
    }

    bool getF32(uint32_t tag, float* out) const {
        camera_metadata_ro_entry_t e{};
        if (!meta_ || find_camera_metadata_ro_entry(meta_, tag, &e) != 0 || e.count < 1)
            return false;
        *out = e.data.f[0];
        return true;
    }

private:
    const camera_metadata_t* meta_ = nullptr;
};

AidlCameraMetadata buildStaticCharacteristics(const CameraStaticConfig& cfg);
AidlCameraMetadata buildSessionCharacteristics(const CameraStaticConfig& cfg);
AidlCameraMetadata buildDefaultRequest(const CameraStaticConfig& cfg, RequestTemplate tpl);

}  // namespace s6cam
