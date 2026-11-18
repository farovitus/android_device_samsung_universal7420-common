#pragma once

#include "gralloc_bridge.h"
#include <array>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace s6cam {

enum class BayerPattern {
    GRBG,  // Rear S5K2P2 (MEDIA_BUS_FMT_SGRBG10_1X10)
    BGGR,  // Front S5K4E6 native VFLIP=0 (MEDIA_BUS_FMT_SBGGR10_1X10)
    GBRG,  // S5K4E6 VFLIP=1 (MEDIA_BUS_FMT_SGBRG10_1X10)
    RGGB,  // MEDIA_BUS_FMT_SRGGB10_1X10
};

struct WbCcmQ10 {
    int32_t rGainQ8;
    int32_t gGainQ8;
    int32_t bGainQ8;
    int32_t ccm[9];
};

struct IspRequestParams {
    uint8_t controlMode = 1;       // AUTO
    uint8_t aeMode = 1;            // ON
    bool aeLock = false;
    int32_t aeExposureComp = 0;    // 1/3 EV steps
    uint8_t aePrecaptureTrigger = 0;
    uint8_t awbMode = 1;           // AUTO
    bool awbLock = false;
    uint8_t afMode = 0;
    uint8_t afTrigger = 0;
    bool hasAfRegion = false;
    int32_t afX = 0;
    int32_t afY = 0;
    int32_t afW = 0;
    int32_t afH = 0;
    uint8_t flashMode = 0;         // OFF
    float focusDistance = 0.0f;
    float zoomRatio = 1.0f;
    bool hasCropRegion = false;
    int32_t cropX = 0;
    int32_t cropY = 0;
    int32_t cropW = 0;
    int32_t cropH = 0;
    int64_t manualExposureNs = 16666666LL;
    int32_t manualSensitivity = 100;
    uint8_t jpegQuality = 92;
    int32_t jpegOrientation = 0;
    float focalLengthMm = 4.3f;
};

struct AeAwbResult {
    int32_t exposureLines = 400;
    int32_t analogGain = 64;
    int32_t digitalGain = 256;
    float swDgain = 1.0f;
    int64_t appliedExposureNs = 16666666LL;
    int32_t appliedSensitivity = 100;
    uint8_t aeState = 2;   // CONVERGED
    uint8_t awbState = 2;  // CONVERGED
    float rGain = 1.35f;
    float gGain = 1.00f;
    float bGain = 1.45f;
};

struct SensorCtrlLimits {
    int32_t expoMin = 4;
    int32_t expoMax = 3000;
    int32_t expoDef = 600;
    int32_t againMin = 32;
    int32_t againMax = 512;
    int32_t againDef = 64;
    int32_t dgainMin = 256;
    int32_t dgainMax = 1024;
    int32_t dgainDef = 256;
    double lineDurationNs = 20000.0;  // ~50 us per line fallback
};

class AeAwbController {
public:
    explicit AeAwbController(BayerPattern pattern);

    void reset(const SensorCtrlLimits& limits);
    AeAwbResult updateFromRaw(const uint16_t* raw, int32_t width, int32_t height,
                              int32_t strideBytes, const IspRequestParams& params,
                              bool fastConvergence = false);

    AeAwbResult current() const { return state_; }

private:
    BayerPattern pattern_;
    SensorCtrlLimits limits_{};
    AeAwbResult state_{};
    bool initialized_ = false;
    double exposureValue_ = 0.0;  // combined linear EV proxy = exposureLines * (again/32) * (dgain/256)
    int32_t aeHoldFrames_ = 0;
};

/**
 * Parameterized Bayer10 SoftISP + multi-format scaler + NEON EXIF JPEG encoder.
 */
class SoftIsp {
public:
    SoftIsp();
    ~SoftIsp();

    SoftIsp(const SoftIsp&) = delete;
    SoftIsp& operator=(const SoftIsp&) = delete;

    // Copies non-cacheable DMA-BUF RAW10 data into a cache-backed buffer once per frame.
    const uint16_t* stageRaw(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideBytes);
    int32_t stagedStrideBytes() const { return stagedW_ * 2; }

    bool processOutput(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideBytes,
                       int32_t activeW, int32_t activeH, BayerPattern pattern,
                       const AeAwbResult& gains, const IspRequestParams& params,
                       const LockedOutput& dst, int32_t* outBlobSizeBytes = nullptr);

private:
    void initGammaLut();
    void runParallel(int32_t height, const std::function<void(int32_t, int32_t)>& fn);

    bool writeRgba(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideU16,
                   BayerPattern pattern, float cropX, float cropY, float cropW, float cropH,
                   const WbCcmQ10& wc, const LockedOutput& dst);

    bool writeYuv(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideU16,
                  BayerPattern pattern, float cropX, float cropY, float cropW, float cropH,
                  const WbCcmQ10& wc, const LockedOutput& dst);

    bool writeJpeg(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideU16,
                   BayerPattern pattern, float cropX, float cropY, float cropW, float cropH,
                   const WbCcmQ10& wc, const IspRequestParams& params,
                   const AeAwbResult& gains, const LockedOutput& dst, int32_t* outBlobSizeBytes);

    std::array<uint8_t, 1024> gammaLut_{};
    std::vector<uint16_t> cachedRaw_;
    int32_t stagedW_ = 0;

    std::vector<std::thread> workers_;
    std::mutex poolMtx_;
    std::condition_variable poolCv_;
    std::condition_variable doneCv_;
    std::function<void(int32_t, int32_t)> jobFn_;
    int32_t jobHeight_ = 0;
    uint64_t jobEpoch_ = 0;
    size_t activeJobs_ = 0;
    bool poolStop_ = false;
};

/**
 * Computes a fast luma-normalized contrast focus metric over the AF ROI (or center 35%).
 */
double computeFocusSharpness(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t strideBytes,
                             int32_t activeW, int32_t activeH, BayerPattern pattern,
                             const IspRequestParams& params);

}  // namespace s6cam
