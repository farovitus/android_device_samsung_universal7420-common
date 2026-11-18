#define LOG_TAG "s6cam_isp"
#include "soft_isp.h"

#include <aidl/android/hardware/camera/device/CameraBlob.h>
#include <aidl/android/hardware/camera/device/CameraBlobId.h>
#include <hardware/gralloc.h>
#include <jpeglib.h>
#include <log/log.h>
#include <system/camera_metadata.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

namespace s6cam {

namespace {

constexpr int kBlackLevel = 64;
constexpr float kLinearTarget = 215.0f;  // Linear pre-gamma mean corresponding to comfortable mid-tone

struct BayerOffsets {
    int rx, ry;
    int g0x, g0y;
    int g1x, g1y;
    int bx, by;
};

constexpr BayerOffsets getBayerOffsets(BayerPattern p) {
    switch (p) {
        case BayerPattern::GRBG:
            return {1, 0, 0, 0, 1, 1, 0, 1};
        case BayerPattern::GBRG:
            return {0, 1, 0, 0, 1, 1, 1, 0};
        case BayerPattern::RGGB:
            return {0, 0, 1, 0, 0, 1, 1, 1};
        case BayerPattern::BGGR:
        default:
            return {1, 1, 1, 0, 0, 1, 0, 0};
    }
}

inline uint8_t clampU8(int v) {
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

inline int clamp10(int v) {
    return v < 0 ? 0 : (v > 1023 ? 1023 : v);
}

inline int subBlack(uint16_t px) {
    int v = static_cast<int>(px & 0x3FF) - kBlackLevel;
    return v > 0 ? v : 0;
}

struct BayerTap {
    int32_t c;
    int32_t w0;
    int32_t w1;
};

std::vector<BayerTap> buildTapLut(float cropStart, float cropLen, int32_t dstDim, int32_t rawDim) {
    std::vector<BayerTap> taps(static_cast<size_t>(dstDim));
    const float step = cropLen / std::max(1, dstDim);
    const int32_t maxC = std::max(0, rawDim - 4);
    for (int32_t d = 0; d < dstDim; ++d) {
        float s = cropStart + (d + 0.5f) * step;
        int32_t c = std::clamp(static_cast<int32_t>(s) & ~1, 0, maxC);
        float f = std::clamp((s - static_cast<float>(c)) * 0.5f, 0.0f, 1.0f);
        int32_t w1 = static_cast<int32_t>(f * 256.0f + 0.5f);
        taps[d] = {c, 256 - w1, w1};
    }
    return taps;
}

struct CachedBayerRows {
    const uint16_t* r0; const uint16_t* r1;
    const uint16_t* b0; const uint16_t* b1;
    const uint16_t* g0_0; const uint16_t* g0_1;
    const uint16_t* g1_0; const uint16_t* g1_1;
    int rx, bx, g0x, g1x;
    int wy0, wy1;
};

inline CachedBayerRows getCachedRows(const uint16_t* raw, int32_t strideU16,
                                     const BayerOffsets& bo, const BayerTap& yt) {
    const uint16_t* base = raw + static_cast<size_t>(yt.c) * strideU16;
    return {
        base + bo.ry * strideU16,
        base + (bo.ry + 2) * strideU16,
        base + bo.by * strideU16,
        base + (bo.by + 2) * strideU16,
        base + bo.g0y * strideU16,
        base + (bo.g0y + 2) * strideU16,
        base + bo.g1y * strideU16,
        base + (bo.g1y + 2) * strideU16,
        bo.rx, bo.bx, bo.g0x, bo.g1x,
        yt.w0, yt.w1,
    };
}

inline void sampleTapCell(const CachedBayerRows& yr, const BayerTap& xt,
                          int& outR, int& outG, int& outB) {
    const int cx = xt.c;
    const int w00 = (xt.w0 * yr.wy0) >> 8;
    const int w10 = (xt.w1 * yr.wy0) >> 8;
    const int w01 = (xt.w0 * yr.wy1) >> 8;
    const int w11 = 256 - w00 - w10 - w01;

    const int r00 = subBlack(yr.r0[cx + yr.rx]);
    const int r10 = subBlack(yr.r0[cx + 2 + yr.rx]);
    const int r01 = subBlack(yr.r1[cx + yr.rx]);
    const int r11 = subBlack(yr.r1[cx + 2 + yr.rx]);

    const int g00 = (subBlack(yr.g0_0[cx + yr.g0x]) + subBlack(yr.g1_0[cx + yr.g1x])) >> 1;
    const int g10 = (subBlack(yr.g0_0[cx + 2 + yr.g0x]) + subBlack(yr.g1_0[cx + 2 + yr.g1x])) >> 1;
    const int g01 = (subBlack(yr.g0_1[cx + yr.g0x]) + subBlack(yr.g1_1[cx + yr.g1x])) >> 1;
    const int g11 = (subBlack(yr.g0_1[cx + 2 + yr.g0x]) + subBlack(yr.g1_1[cx + 2 + yr.g1x])) >> 1;

    const int b00 = subBlack(yr.b0[cx + yr.bx]);
    const int b10 = subBlack(yr.b0[cx + 2 + yr.bx]);
    const int b01 = subBlack(yr.b1[cx + yr.bx]);
    const int b11 = subBlack(yr.b1[cx + 2 + yr.bx]);

    outR = (r00 * w00 + r10 * w10 + r01 * w01 + r11 * w11) >> 8;
    outG = (g00 * w00 + g10 * w10 + g01 * w01 + g11 * w11) >> 8;
    outB = (b00 * w00 + b10 * w10 + b01 * w01 + b11 * w11) >> 8;
}

void computeAspectSafeCrop(int32_t rawW, int32_t rawH, int32_t activeW, int32_t activeH,
                           int32_t dstW, int32_t dstH, const IspRequestParams& params,
                           float& outX, float& outY, float& outW, float& outH) {
    float normX = 0.0f, normY = 0.0f, normW = 1.0f, normH = 1.0f;
    if (params.hasCropRegion && params.cropW > 0 && params.cropH > 0 && activeW > 0 && activeH > 0) {
        normX = static_cast<float>(params.cropX) / activeW;
        normY = static_cast<float>(params.cropY) / activeH;
        normW = static_cast<float>(params.cropW) / activeW;
        normH = static_cast<float>(params.cropH) / activeH;
    }
    if (params.zoomRatio > 1.01f) {
        float invZ = 1.0f / std::clamp(params.zoomRatio, 1.0f, 8.0f);
        float cx = normX + normW * 0.5f;
        float cy = normY + normH * 0.5f;
        normW *= invZ;
        normH *= invZ;
        normX = cx - normW * 0.5f;
        normY = cy - normH * 0.5f;
    }

    float baseX = normX * rawW;
    float baseY = normY * rawH;
    float baseW = normW * rawW;
    float baseH = normH * rawH;

    // Preserve target aspect ratio inside base crop
    float dstAspect = static_cast<float>(dstW) / std::max(1, dstH);
    float srcAspect = baseW / std::max(1.0f, baseH);
    if (srcAspect > dstAspect * 1.01f) {
        float newW = baseH * dstAspect;
        baseX += (baseW - newW) * 0.5f;
        baseW = newW;
    } else if (dstAspect > srcAspect * 1.01f) {
        float newH = baseW / dstAspect;
        baseY += (baseH - newH) * 0.5f;
        baseH = newH;
    }

    outX = std::clamp(baseX, 0.0f, static_cast<float>(rawW - 4));
    outY = std::clamp(baseY, 0.0f, static_cast<float>(rawH - 4));
    outW = std::clamp(baseW, 4.0f, static_cast<float>(rawW) - outX);
    outH = std::clamp(baseH, 4.0f, static_cast<float>(rawH) - outY);
}

WbCcmQ10 buildWbCcm(const AeAwbResult& gains) {
    // Tuned mild Samsung CMOS D65 sRGB matrix; normalize and clamp WB BEFORE CCM so clipped pixels
    // saturate cleanly to neutral white rather than pink.
    static constexpr float kCcm[3][3] = {
        { 1.28f, -0.20f, -0.08f},
        {-0.12f,  1.22f, -0.10f},
        {-0.04f, -0.24f,  1.28f},
    };
    const float whiteScale = 256.0f * 1023.0f / (1023.0f - kBlackLevel);
    const float scale = std::clamp(gains.swDgain, 1.0f, 8.0f) * whiteScale;
    WbCcmQ10 wc{};
    wc.rGainQ8 = static_cast<int32_t>(std::lround(gains.rGain * scale));
    wc.gGainQ8 = static_cast<int32_t>(std::lround(gains.gGain * scale));
    wc.bGainQ8 = static_cast<int32_t>(std::lround(gains.bGain * scale));
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            wc.ccm[r * 3 + c] = static_cast<int32_t>(std::lround(kCcm[r][c] * 1024.0f));
        }
    }
    return wc;
}

inline void applyWbCcm(const WbCcmQ10& wc, int lr, int lg, int lb,
                       int& outR, int& outG, int& outB) {
    const int wr = std::min(1023, (lr * wc.rGainQ8) >> 8);
    const int wg = std::min(1023, (lg * wc.gGainQ8) >> 8);
    const int wb = std::min(1023, (lb * wc.bGainQ8) >> 8);
    const int* m = wc.ccm;
    outR = clamp10((m[0] * wr + m[1] * wg + m[2] * wb) >> 10);
    outG = clamp10((m[3] * wr + m[4] * wg + m[5] * wb) >> 10);
    outB = clamp10((m[6] * wr + m[7] * wg + m[8] * wb) >> 10);
}

int orientationToExif(int deg) {
    switch (((deg % 360) + 360) % 360) {
        case 90:  return 6;
        case 180: return 3;
        case 270: return 8;
        default:  return 1;
    }
}

std::vector<uint8_t> buildExifApp1(int32_t orientationDeg, int64_t expoNs, int32_t iso, float focalMm) {
    // Minimal clean big-endian TIFF EXIF APP1 header carrying Orientation + Camera Model
    std::vector<uint8_t> b;
    auto push16 = [&](uint16_t v) {
        b.push_back(static_cast<uint8_t>(v >> 8));
        b.push_back(static_cast<uint8_t>(v & 0xFF));
    };
    auto push32 = [&](uint32_t v) {
        b.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
        b.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
        b.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
        b.push_back(static_cast<uint8_t>(v & 0xFF));
    };
    auto patch32 = [&](size_t pos, uint32_t v) {
        b[pos + 0] = static_cast<uint8_t>((v >> 24) & 0xFF);
        b[pos + 1] = static_cast<uint8_t>((v >> 16) & 0xFF);
        b[pos + 2] = static_cast<uint8_t>((v >> 8) & 0xFF);
        b[pos + 3] = static_cast<uint8_t>(v & 0xFF);
    };

    const char hdr[] = {'E', 'x', 'i', 'f', 0, 0};
    b.insert(b.end(), hdr, hdr + 6);
    const size_t tiffStart = b.size();
    // TIFF header "MM" (big endian), 0x002A, offset to IFD0 = 8
    b.push_back('M'); b.push_back('M');
    push16(42);
    push32(8);

    // IFD0: 4 tags (Make, Model, Orientation, ExifIFD)
    push16(4);
    const size_t makeOffPos = b.size() + 8;
    push16(0x010F); push16(2); push32(8); push32(0);   // Make "samsung\0" (8)
    const size_t modelOffPos = b.size() + 8;
    push16(0x0110); push16(2); push32(9); push32(0);   // Model "SM-G920I\0" (9)
    push16(0x0112); push16(3); push32(1); push16(orientationToExif(orientationDeg)); push16(0);
    const size_t exifIfdOffPos = b.size() + 8;
    push16(0x8769); push16(4); push32(1); push32(0);
    push32(0);  // Next IFD = 0

    patch32(makeOffPos, static_cast<uint32_t>(b.size() - tiffStart));
    const char mk[] = "samsung";
    b.insert(b.end(), mk, mk + sizeof(mk));

    patch32(modelOffPos, static_cast<uint32_t>(b.size() - tiffStart));
    const char md[] = "SM-G920I";
    b.insert(b.end(), md, md + sizeof(md));
    if ((b.size() & 1) != 0) b.push_back(0);

    // Exif SubIFD: ExposureTime, ISOSpeedRatings, FocalLength
    patch32(exifIfdOffPos, static_cast<uint32_t>(b.size() - tiffStart));
    push16(3);
    const size_t expOffPos = b.size() + 8;
    push16(0x829A); push16(5); push32(1); push32(0);
    push16(0x8827); push16(3); push32(1); push16(static_cast<uint16_t>(std::clamp(iso, 50, 6400))); push16(0);
    const size_t focOffPos = b.size() + 8;
    push16(0x920A); push16(5); push32(1); push32(0);
    push32(0);

    patch32(expOffPos, static_cast<uint32_t>(b.size() - tiffStart));
    uint32_t expUs = static_cast<uint32_t>(std::max<int64_t>(100, expoNs / 1000LL));
    push32(expUs); push32(1000000U);

    patch32(focOffPos, static_cast<uint32_t>(b.size() - tiffStart));
    push32(static_cast<uint32_t>(std::lround(focalMm * 100.0f))); push32(100U);

    return b;
}

}  // namespace

AeAwbController::AeAwbController(BayerPattern pattern) : pattern_(pattern) {}

void AeAwbController::reset(const SensorCtrlLimits& limits) {
    limits_ = limits;
    state_.exposureLines = std::clamp(limits_.expoDef, limits_.expoMin, limits_.expoMax);
    state_.analogGain = std::clamp(limits_.againDef, limits_.againMin, limits_.againMax);
    state_.digitalGain = std::clamp(limits_.dgainDef, limits_.dgainMin, limits_.dgainMax);
    state_.swDgain = 1.0f;
    state_.appliedExposureNs = static_cast<int64_t>(state_.exposureLines * limits_.lineDurationNs);
    state_.appliedSensitivity = (state_.analogGain * 100) / std::max(1, limits_.againMin);
    state_.rGain = 1.40f;
    state_.gGain = 1.00f;
    state_.bGain = 1.50f;
    state_.aeState = ANDROID_CONTROL_AE_STATE_SEARCHING;
    state_.awbState = ANDROID_CONTROL_AWB_STATE_SEARCHING;
    exposureValue_ = state_.exposureLines * (state_.analogGain / 32.0) * (state_.digitalGain / 256.0);
    aeHoldFrames_ = 0;
    initialized_ = true;
}

AeAwbResult AeAwbController::updateFromRaw(const uint16_t* raw, int32_t width, int32_t height,
                                           int32_t strideBytes, const IspRequestParams& params,
                                           bool fastConvergence) {
    if (!initialized_) {
        reset(limits_);
    }

    if (params.aeMode == ANDROID_CONTROL_AE_MODE_OFF || params.controlMode == ANDROID_CONTROL_MODE_OFF) {
        int32_t reqLines = static_cast<int32_t>(
            std::llround(params.manualExposureNs / std::max(1.0, limits_.lineDurationNs)));
        state_.exposureLines = std::clamp(reqLines, limits_.expoMin, limits_.expoMax);
        int32_t reqAgain = std::clamp((params.manualSensitivity * limits_.againMin) / 100,
                                      limits_.againMin, limits_.againMax);
        state_.analogGain = reqAgain;
        state_.digitalGain = limits_.dgainDef;
        state_.swDgain = 1.0f;
        state_.appliedExposureNs = static_cast<int64_t>(state_.exposureLines * limits_.lineDurationNs);
        state_.appliedSensitivity = (state_.analogGain * 100) / std::max(1, limits_.againMin);
        state_.aeState = ANDROID_CONTROL_AE_STATE_INACTIVE;
    } else if (params.aeLock) {
        state_.aeState = ANDROID_CONTROL_AE_STATE_LOCKED;
    }

    if (params.awbMode == ANDROID_CONTROL_AWB_MODE_INCANDESCENT) {
        state_.rGain = 1.05f; state_.gGain = 1.0f; state_.bGain = 2.05f;
        state_.awbState = ANDROID_CONTROL_AWB_STATE_CONVERGED;
    } else if (params.awbMode == ANDROID_CONTROL_AWB_MODE_FLUORESCENT) {
        state_.rGain = 1.25f; state_.gGain = 1.0f; state_.bGain = 1.65f;
        state_.awbState = ANDROID_CONTROL_AWB_STATE_CONVERGED;
    } else if (params.awbMode == ANDROID_CONTROL_AWB_MODE_DAYLIGHT) {
        state_.rGain = 1.48f; state_.gGain = 1.0f; state_.bGain = 1.38f;
        state_.awbState = ANDROID_CONTROL_AWB_STATE_CONVERGED;
    } else if (params.awbMode == ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT) {
        state_.rGain = 1.60f; state_.gGain = 1.0f; state_.bGain = 1.28f;
        state_.awbState = ANDROID_CONTROL_AWB_STATE_CONVERGED;
    } else if (params.awbLock) {
        state_.awbState = ANDROID_CONTROL_AWB_STATE_LOCKED;
    }

    if (!raw || width < 16 || height < 16) return state_;

    const int32_t strideU16 = strideBytes / 2;
    const BayerOffsets bo = getBayerOffsets(pattern_);
    const int stepX = std::max(2, (width / 64) & ~1);
    const int stepY = std::max(2, (height / 48) & ~1);

    double sumLuma = 0.0;
    double countLuma = 0.0;
    double sumR = 0.0, sumG = 0.0, sumB = 0.0;
    double countWb = 0.0;

    const int marginX = width / 10;
    const int marginY = height / 10;
    for (int y = marginY; y < height - marginY - 1; y += stepY) {
        const int cy = y & ~1;
        for (int x = marginX; x < width - marginX - 1; x += stepX) {
            const int cx = x & ~1;
            int r = static_cast<int>(raw[(cy + bo.ry) * strideU16 + cx + bo.rx] & 0x3FF) - kBlackLevel;
            int g0 = static_cast<int>(raw[(cy + bo.g0y) * strideU16 + cx + bo.g0x] & 0x3FF) - kBlackLevel;
            int g1 = static_cast<int>(raw[(cy + bo.g1y) * strideU16 + cx + bo.g1x] & 0x3FF) - kBlackLevel;
            int b = static_cast<int>(raw[(cy + bo.by) * strideU16 + cx + bo.bx] & 0x3FF) - kBlackLevel;
            r = std::max(0, r);
            b = std::max(0, b);
            int g = std::max(0, (g0 + g1) >> 1);

            double yLin = 0.299 * r + 0.587 * g + 0.114 * b;
            sumLuma += yLin;
            countLuma += 1.0;

            int maxC = std::max({r, g, b});
            int minC = std::min({r, g, b});
            const double effGain = std::clamp(static_cast<double>(state_.swDgain), 1.0, 8.0);
            if (maxC < 880 && (static_cast<double>(minC) * effGain) > 20.0) {
                sumR += r;
                sumG += g;
                sumB += b;
                countWb += 1.0;
            }
        }
    }

    if (aeHoldFrames_ > 0 && !fastConvergence) {
        --aeHoldFrames_;
    } else if (countLuma > 0.0 &&
        params.aeMode != ANDROID_CONTROL_AE_MODE_OFF &&
        params.controlMode != ANDROID_CONTROL_MODE_OFF &&
        !params.aeLock) {
        aeHoldFrames_ = 0;
        double meanY = std::max(4.0, (sumLuma / countLuma) * static_cast<double>(state_.swDgain));
        double compMult = std::pow(2.0, params.aeExposureComp / 3.0);
        double target = kLinearTarget * compMult;
        double ratio = target / meanY;

        if (std::abs(ratio - 1.0) < 0.12) {
            state_.aeState = ANDROID_CONTROL_AE_STATE_CONVERGED;
        } else {
            state_.aeState = ANDROID_CONTROL_AE_STATE_SEARCHING;
            ratio = fastConvergence ? std::clamp(ratio, 0.03, 4.0) : std::clamp(ratio, 0.5, 2.0);
            const double damp = fastConvergence ? 1.00 : 0.30;
            double dampedRatio = 1.0 + damp * (ratio - 1.0);
            const int32_t totalDgainMax = 2048;  // <= 8.0x combined HW+SW digital gain
            const int32_t hwDgainMax = std::clamp(totalDgainMax, limits_.dgainMin, limits_.dgainMax);
            double maxEv = static_cast<double>(limits_.expoMax) *
                           (limits_.againMax / 32.0) * (totalDgainMax / 256.0);
            double minEv = static_cast<double>(limits_.expoMin);
            exposureValue_ = std::clamp(exposureValue_ * dampedRatio, minEv, maxEv);

            // Prefer shutter speed up to ~33 ms (or expoMax * 0.65), then analog gain, then digital gain
            double prefMaxLines = std::min<double>(limits_.expoMax,
                                                   33333333.0 / std::max(1.0, limits_.lineDurationNs));
            if (prefMaxLines < limits_.expoMin) prefMaxLines = limits_.expoMax;

            double lines = std::clamp(exposureValue_, static_cast<double>(limits_.expoMin), prefMaxLines);
            double rem = exposureValue_ / std::max(1.0, lines);
            double againF = std::clamp(rem * 32.0, static_cast<double>(limits_.againMin),
                                       static_cast<double>(limits_.againMax));
            double totalDgainF = std::clamp((exposureValue_ / std::max(1.0, lines * (againF / 32.0))) * 256.0,
                                            static_cast<double>(limits_.dgainMin),
                                            static_cast<double>(totalDgainMax));
            double hwDgainF = std::clamp(totalDgainF, static_cast<double>(limits_.dgainMin),
                                         static_cast<double>(hwDgainMax));
            float newSwDgain = static_cast<float>(
                std::clamp(totalDgainF / std::max(1.0, hwDgainF), 1.0, 8.0));

            int32_t newExpo = static_cast<int32_t>(std::lround(lines));
            int32_t newAgain = static_cast<int32_t>(std::lround(againF));
            int32_t newDgain = static_cast<int32_t>(std::lround(hwDgainF));
            if (newExpo != state_.exposureLines || newAgain != state_.analogGain ||
                newDgain != state_.digitalGain) {
                aeHoldFrames_ = 2;  // allow sensor I2C + 2-frame pipeline latency to settle before resampling
            }
            state_.exposureLines = newExpo;
            state_.analogGain = newAgain;
            state_.digitalGain = newDgain;
            state_.swDgain = newSwDgain;
            state_.appliedExposureNs = static_cast<int64_t>(state_.exposureLines * limits_.lineDurationNs);
            double totalIsoMult = (state_.analogGain / std::max(1.0, static_cast<double>(limits_.againMin))) *
                                  (state_.digitalGain / 256.0) * state_.swDgain;
            state_.appliedSensitivity = static_cast<int32_t>(std::llround(totalIsoMult * 100.0));
        }
    }

    if (countWb >= 32.0 && params.awbMode == ANDROID_CONTROL_AWB_MODE_AUTO && !params.awbLock) {
        double avgR = std::max(8.0, sumR / countWb);
        double avgG = std::max(8.0, sumG / countWb);
        double avgB = std::max(8.0, sumB / countWb);
        float targetR = static_cast<float>(std::clamp(avgG / avgR, 0.75, 2.8));
        float targetB = static_cast<float>(std::clamp(avgG / avgB, 0.75, 2.8));

        float errR = std::abs(targetR - state_.rGain);
        float errB = std::abs(targetB - state_.bGain);
        state_.rGain = state_.rGain * 0.80f + targetR * 0.20f;
        state_.gGain = 1.00f;
        state_.bGain = state_.bGain * 0.80f + targetB * 0.20f;
        state_.awbState = (errR < 0.04f && errB < 0.04f)
                              ? ANDROID_CONTROL_AWB_STATE_CONVERGED
                              : ANDROID_CONTROL_AWB_STATE_SEARCHING;
    }

    return state_;
}

SoftIsp::SoftIsp() {
    initGammaLut();
    constexpr size_t kNumWorkers = 3;
    workers_.reserve(kNumWorkers);
    for (size_t id = 0; id < kNumWorkers; ++id) {
        workers_.emplace_back([this, id]() {
            uint64_t seenEpoch = 0;
            while (true) {
                std::function<void(int32_t, int32_t)> fn;
                int32_t h = 0;
                {
                    std::unique_lock<std::mutex> lk(poolMtx_);
                    poolCv_.wait(lk, [&]() { return poolStop_ || jobEpoch_ > seenEpoch; });
                    if (poolStop_) return;
                    seenEpoch = jobEpoch_;
                    fn = jobFn_;
                    h = jobHeight_;
                }
                const int32_t total = 4;
                const int32_t chunk = (h + total - 1) / total;
                const int32_t y0 = static_cast<int32_t>(id) * chunk;
                const int32_t y1 = std::min(h, y0 + chunk);
                if (y0 < y1 && fn) {
                    fn(y0, y1);
                }
                {
                    std::lock_guard<std::mutex> lk(poolMtx_);
                    if (--activeJobs_ == 0) {
                        doneCv_.notify_all();
                    }
                }
            }
        });
    }
}

SoftIsp::~SoftIsp() {
    {
        std::lock_guard<std::mutex> lk(poolMtx_);
        poolStop_ = true;
    }
    poolCv_.notify_all();
    for (auto& w : workers_) {
        if (w.joinable()) w.join();
    }
}

void SoftIsp::runParallel(int32_t height, const std::function<void(int32_t, int32_t)>& fn) {
    if (height < 256 || workers_.empty()) {
        fn(0, height);
        return;
    }
    {
        std::lock_guard<std::mutex> lk(poolMtx_);
        jobFn_ = fn;
        jobHeight_ = height;
        activeJobs_ = workers_.size();
        ++jobEpoch_;
    }
    poolCv_.notify_all();

    const int32_t total = static_cast<int32_t>(workers_.size()) + 1;
    const int32_t chunk = (height + total - 1) / total;
    const int32_t yLast0 = static_cast<int32_t>(workers_.size()) * chunk;
    if (yLast0 < height) {
        fn(yLast0, height);
    }

    std::unique_lock<std::mutex> lk(poolMtx_);
    doneCv_.wait(lk, [&]() { return activeJobs_ == 0; });
    jobFn_ = nullptr;
}

const uint16_t* SoftIsp::stageRaw(const uint16_t* raw, int32_t rawW, int32_t rawH,
                                  int32_t rawStrideBytes) {
    if (!raw || rawW <= 0 || rawH <= 0 || rawStrideBytes < rawW * 2) return nullptr;
    const size_t need = static_cast<size_t>(rawW) * rawH;
    if (cachedRaw_.size() < need) {
        cachedRaw_.resize(need);
    }
    stagedW_ = rawW;
    if (rawStrideBytes == rawW * 2) {
        std::memcpy(cachedRaw_.data(), raw, need * sizeof(uint16_t));
    } else {
        const uint8_t* src = reinterpret_cast<const uint8_t*>(raw);
        for (int32_t y = 0; y < rawH; ++y) {
            std::memcpy(cachedRaw_.data() + static_cast<size_t>(y) * rawW,
                        src + static_cast<size_t>(y) * rawStrideBytes,
                        static_cast<size_t>(rawW) * sizeof(uint16_t));
        }
    }
    return cachedRaw_.data();
}

void SoftIsp::initGammaLut() {
    for (int i = 0; i < 1024; ++i) {
        double x = static_cast<double>(i) / 1023.0;
        // Standard sRGB transfer curve
        double srgb = (x <= 0.0031308) ? (x * 12.92) : (1.055 * std::pow(x, 1.0 / 2.2) - 0.055);
        int v = static_cast<int>(std::lround(srgb * 255.0));
        gammaLut_[i] = clampU8(v);
    }
}

bool SoftIsp::processOutput(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideBytes,
                            int32_t activeW, int32_t activeH, BayerPattern pattern,
                            const AeAwbResult& gains, const IspRequestParams& params,
                            const LockedOutput& dst, int32_t* outBlobSizeBytes) {
    if (!raw || rawW < 8 || rawH < 8 || rawStrideBytes < rawW * 2) return false;

    const int32_t rawStrideU16 = rawStrideBytes / 2;
    float cropX = 0.0f, cropY = 0.0f, cropW = 0.0f, cropH = 0.0f;
    computeAspectSafeCrop(rawW, rawH, activeW, activeH, dst.width, dst.height, params,
                          cropX, cropY, cropW, cropH);

    const WbCcmQ10 wc = buildWbCcm(gains);

    if (dst.format == HAL_PIXEL_FORMAT_BLOB) {
        return writeJpeg(raw, rawW, rawH, rawStrideU16, pattern, cropX, cropY, cropW, cropH,
                         wc, params, gains, dst, outBlobSizeBytes);
    }
    if (dst.isYcbcr) {
        return writeYuv(raw, rawW, rawH, rawStrideU16, pattern, cropX, cropY, cropW, cropH,
                        wc, dst);
    }
    return writeRgba(raw, rawW, rawH, rawStrideU16, pattern, cropX, cropY, cropW, cropH,
                     wc, dst);
}

bool SoftIsp::writeRgba(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideU16,
                        BayerPattern pattern, float cropX, float cropY, float cropW, float cropH,
                        const WbCcmQ10& wc, const LockedOutput& dst) {
    if (!dst.vaddr || dst.width <= 0 || dst.height <= 0) return false;
    uint8_t* base = static_cast<uint8_t*>(dst.vaddr);
    const int32_t stridePx = std::max(dst.stride, dst.width);
    const BayerOffsets bo = getBayerOffsets(pattern);
    const auto xTaps = buildTapLut(cropX, cropW, dst.width, rawW);
    const auto yTaps = buildTapLut(cropY, cropH, dst.height, rawH);

    const bool swapRB = (dst.format == HAL_PIXEL_FORMAT_BGRA_8888);

    runParallel(dst.height, [&](int32_t y0, int32_t y1) {
        std::vector<uint32_t> lineBuf(static_cast<size_t>(dst.width));
        for (int32_t dy = y0; dy < y1; ++dy) {
            const CachedBayerRows yr = getCachedRows(raw, rawStrideU16, bo, yTaps[dy]);
            uint32_t* out32 = lineBuf.data();
            for (int32_t dx = 0; dx < dst.width; ++dx) {
                int lr, lg, lb, cr, cg, cb;
                sampleTapCell(yr, xTaps[dx], lr, lg, lb);
                applyWbCcm(wc, lr, lg, lb, cr, cg, cb);

                const uint32_t ur = gammaLut_[cr];
                const uint32_t ug = gammaLut_[cg];
                const uint32_t ub = gammaLut_[cb];

                if (swapRB) {
                    out32[dx] = 0xFF000000u | (ur << 16) | (ug << 8) | ub;
                } else {
                    out32[dx] = 0xFF000000u | (ub << 16) | (ug << 8) | ur;
                }
            }
            uint8_t* rowPtr = base + static_cast<size_t>(dy) * stridePx * 4;
            std::memcpy(rowPtr, lineBuf.data(), static_cast<size_t>(dst.width) * 4);
        }
    });
    return true;
}

bool SoftIsp::writeYuv(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideU16,
                       BayerPattern pattern, float cropX, float cropY, float cropW, float cropH,
                       const WbCcmQ10& wc, const LockedOutput& dst) {
    if (!dst.ycbcr.y || !dst.ycbcr.cb || !dst.ycbcr.cr || dst.width <= 0 || dst.height <= 0)
        return false;

    uint8_t* yPlane = static_cast<uint8_t*>(dst.ycbcr.y);
    uint8_t* cbPlane = static_cast<uint8_t*>(dst.ycbcr.cb);
    uint8_t* crPlane = static_cast<uint8_t*>(dst.ycbcr.cr);
    const size_t yStride = dst.ycbcr.ystride;
    const size_t cStride = dst.ycbcr.cstride;
    const size_t cStep = dst.ycbcr.chroma_step;

    const BayerOffsets bo = getBayerOffsets(pattern);
    const auto xTaps = buildTapLut(cropX, cropW, dst.width, rawW);
    const auto yTaps = buildTapLut(cropY, cropH, dst.height, rawH);

    // Pair rows (0+1, 2+3, ...) cleanly so threads never share chroma lines
    const int32_t pairRows = (dst.height + 1) / 2;
    runParallel(pairRows, [&](int32_t py0, int32_t py1) {
        std::vector<uint8_t> yLine(static_cast<size_t>(dst.width));
        std::vector<int> rBuf(static_cast<size_t>(dst.width));
        std::vector<int> gBuf(static_cast<size_t>(dst.width));
        std::vector<int> bBuf(static_cast<size_t>(dst.width));

        for (int32_t py = py0; py < py1; ++py) {
            const int32_t yTop = py * 2;
            const int32_t yBot = yTop + 1;

            auto processRow = [&](int32_t dy, bool accumChroma) {
                const CachedBayerRows yr = getCachedRows(raw, rawStrideU16, bo, yTaps[dy]);
                for (int32_t dx = 0; dx < dst.width; ++dx) {
                    int lr, lg, lb, cr, cg, cb;
                    sampleTapCell(yr, xTaps[dx], lr, lg, lb);
                    applyWbCcm(wc, lr, lg, lb, cr, cg, cb);

                    uint8_t r = gammaLut_[cr];
                    uint8_t g = gammaLut_[cg];
                    uint8_t b = gammaLut_[cb];

                    // BT.601 full range JPEG/Camera2 standard
                    int yVal = (77 * r + 150 * g + 29 * b + 128) >> 8;
                    yLine[dx] = clampU8(yVal);
                    if (accumChroma) {
                        rBuf[dx] = r; gBuf[dx] = g; bBuf[dx] = b;
                    }
                }
                std::memcpy(yPlane + static_cast<size_t>(dy) * yStride, yLine.data(),
                            static_cast<size_t>(dst.width));
            };

            processRow(yTop, true);
            if (yBot < dst.height) {
                processRow(yBot, false);
            }

            uint8_t* cbRow = cbPlane + static_cast<size_t>(py) * cStride;
            uint8_t* crRow = crPlane + static_cast<size_t>(py) * cStride;
            const int32_t halfW = (dst.width + 1) / 2;
            for (int32_t px = 0; px < halfW; ++px) {
                const int32_t x = px * 2;
                int r = rBuf[x];
                int g = gBuf[x];
                int b = bBuf[x];
                if (x + 1 < dst.width) {
                    r = (r + rBuf[x + 1]) >> 1;
                    g = (g + gBuf[x + 1]) >> 1;
                    b = (b + bBuf[x + 1]) >> 1;
                }
                int cb = ((-43 * r - 85 * g + 128 * b + 128) >> 8) + 128;
                int cr = ((128 * r - 107 * g - 21 * b + 128) >> 8) + 128;
                cbRow[px * cStep] = clampU8(cb);
                crRow[px * cStep] = clampU8(cr);
            }
        }
    });
    return true;
}

bool SoftIsp::writeJpeg(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t rawStrideU16,
                        BayerPattern pattern, float cropX, float cropY, float cropW, float cropH,
                        const WbCcmQ10& wc, const IspRequestParams& params,
                        const AeAwbResult& gains, const LockedOutput& dst, int32_t* outBlobSizeBytes) {
    if (!dst.vaddr || dst.bufferSize <= static_cast<int32_t>(sizeof(::aidl::android::hardware::camera::device::CameraBlob)))
        return false;

    const int32_t w = dst.width;
    const int32_t h = dst.height;
    std::vector<uint8_t> rgb(static_cast<size_t>(w) * h * 3);
    const BayerOffsets bo = getBayerOffsets(pattern);
    const auto xTaps = buildTapLut(cropX, cropW, w, rawW);
    const auto yTaps = buildTapLut(cropY, cropH, h, rawH);

    runParallel(h, [&](int32_t y0, int32_t y1) {
        for (int32_t dy = y0; dy < y1; ++dy) {
            uint8_t* rowPtr = rgb.data() + static_cast<size_t>(dy) * w * 3;
            const CachedBayerRows yr = getCachedRows(raw, rawStrideU16, bo, yTaps[dy]);
            for (int32_t dx = 0; dx < w; ++dx) {
                int lr, lg, lb, cr, cg, cb;
                sampleTapCell(yr, xTaps[dx], lr, lg, lb);
                applyWbCcm(wc, lr, lg, lb, cr, cg, cb);

                rowPtr[dx * 3 + 0] = gammaLut_[cr];
                rowPtr[dx * 3 + 1] = gammaLut_[cg];
                rowPtr[dx * 3 + 2] = gammaLut_[cb];
            }
        }
    });

    struct jpeg_compress_struct cinfo{};
    struct jpeg_error_mgr jerr{};
    uint8_t* outMem = nullptr;
    unsigned long outSize = 0;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &outMem, &outSize);

    cinfo.image_width = static_cast<JDIMENSION>(w);
    cinfo.image_height = static_cast<JDIMENSION>(h);
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, std::clamp<int>(params.jpegQuality, 10, 100), TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    auto exif = buildExifApp1(params.jpegOrientation, gains.appliedExposureNs,
                              gains.appliedSensitivity, params.focalLengthMm);
    jpeg_write_marker(&cinfo, JPEG_APP0 + 1, exif.data(), static_cast<unsigned int>(exif.size()));

    while (cinfo.next_scanline < cinfo.image_height) {
        JSAMPROW row = rgb.data() + static_cast<size_t>(cinfo.next_scanline) * w * 3;
        jpeg_write_scanlines(&cinfo, &row, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    using ::aidl::android::hardware::camera::device::CameraBlob;
    using ::aidl::android::hardware::camera::device::CameraBlobId;
    const size_t maxPayload = static_cast<size_t>(dst.bufferSize) - sizeof(CameraBlob);
    if (!outMem || outSize == 0 || outSize > maxPayload) {
        ALOGE("Encoded JPEG (%lu bytes) exceeds BLOB buffer payload (%zu bytes)", outSize, maxPayload);
        free(outMem);
        return false;
    }

    std::memcpy(dst.vaddr, outMem, outSize);
    free(outMem);

    CameraBlob blobHdr{};
    blobHdr.blobId = CameraBlobId::JPEG;
    blobHdr.blobSizeBytes = static_cast<int32_t>(outSize);
    std::memcpy(static_cast<uint8_t*>(dst.vaddr) + maxPayload, &blobHdr, sizeof(CameraBlob));

    if (outBlobSizeBytes) *outBlobSizeBytes = static_cast<int32_t>(outSize);
    return true;
}

double computeFocusSharpness(const uint16_t* raw, int32_t rawW, int32_t rawH, int32_t strideBytes,
                             int32_t activeW, int32_t activeH, BayerPattern pattern,
                             const IspRequestParams& params) {
    if (!raw || rawW < 32 || rawH < 32) return 0.0;

    const int32_t strideU16 = strideBytes / 2;
    const BayerOffsets bo = getBayerOffsets(pattern);

    int rx0 = (rawW * 32) / 100;
    int rx1 = (rawW * 68) / 100;
    int ry0 = (rawH * 32) / 100;
    int ry1 = (rawH * 68) / 100;

    if (params.hasAfRegion && params.afW > 0 && params.afH > 0 && activeW > 0 && activeH > 0) {
        int mapX0 = static_cast<int>(static_cast<int64_t>(params.afX) * rawW / activeW);
        int mapY0 = static_cast<int>(static_cast<int64_t>(params.afY) * rawH / activeH);
        int mapX1 = static_cast<int>(static_cast<int64_t>(params.afX + params.afW) * rawW / activeW);
        int mapY1 = static_cast<int>(static_cast<int64_t>(params.afY + params.afH) * rawH / activeH);
        if (mapX1 - mapX0 >= 64 && mapY1 - mapY0 >= 64) {
            rx0 = mapX0;
            ry0 = mapY0;
            rx1 = mapX1;
            ry1 = mapY1;
        }
    }

    rx0 = std::clamp(rx0, 4, rawW - 12) & ~1;
    ry0 = std::clamp(ry0, 4, rawH - 12) & ~1;
    rx1 = std::clamp(rx1, rx0 + 8, rawW - 4);
    ry1 = std::clamp(ry1, ry0 + 8, rawH - 4);

    double gradSum = 0.0;
    double lumaSum = 0.0;
    int64_t samples = 0;

    for (int y = ry0; y < ry1 - 4; y += 4) {
        const int row0 = (y + bo.g0y) * strideU16 + bo.g0x;
        const int rowNext = (y + 2 + bo.g0y) * strideU16 + bo.g0x;
        for (int x = rx0; x < rx1 - 4; x += 4) {
            int g = static_cast<int>(raw[row0 + x] & 0x3FF) - kBlackLevel;
            int gx = static_cast<int>(raw[row0 + x + 2] & 0x3FF) - kBlackLevel;
            int gy = static_cast<int>(raw[rowNext + x] & 0x3FF) - kBlackLevel;
            if (g < 0) g = 0;
            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            double dx = static_cast<double>(gx - g);
            double dy = static_cast<double>(gy - g);
            gradSum += dx * dx + dy * dy;
            lumaSum += static_cast<double>(g);
            ++samples;
        }
    }

    if (samples <= 0) return 0.0;
    double mean = lumaSum / static_cast<double>(samples);
    return gradSum / ((mean * mean + 64.0) * static_cast<double>(samples));
}

}  // namespace s6cam
