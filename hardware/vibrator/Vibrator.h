/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/vibrator/BnVibrator.h>
#include <android-base/unique_fd.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

/*
 * Vibrator HAL for a kernel force-feedback input device. On the NK 6.6
 * kernel the MAX77843 haptic motor is the mainline max77693-haptic driver,
 * which plays FF_RUMBLE effects; amplitude is the rumble magnitude.
 */
class Vibrator : public BnVibrator {
  public:
    Vibrator();

    ndk::ScopedAStatus getCapabilities(int32_t* _aidl_return) override;
    ndk::ScopedAStatus off() override;
    ndk::ScopedAStatus on(int32_t timeoutMs,
                          const std::shared_ptr<IVibratorCallback>& callback) override;
    ndk::ScopedAStatus perform(Effect effect, EffectStrength strength,
                               const std::shared_ptr<IVibratorCallback>& callback,
                               int32_t* _aidl_return) override;
    ndk::ScopedAStatus getSupportedEffects(std::vector<Effect>* _aidl_return) override;
    ndk::ScopedAStatus setAmplitude(float amplitude) override;
    ndk::ScopedAStatus setExternalControl(bool enabled) override;
    ndk::ScopedAStatus getCompositionDelayMax(int32_t* maxDelayMs) override;
    ndk::ScopedAStatus getCompositionSizeMax(int32_t* maxSize) override;
    ndk::ScopedAStatus getSupportedPrimitives(std::vector<CompositePrimitive>* supported) override;
    ndk::ScopedAStatus getPrimitiveDuration(CompositePrimitive primitive,
                                            int32_t* durationMs) override;
    ndk::ScopedAStatus compose(const std::vector<CompositeEffect>& composite,
                               const std::shared_ptr<IVibratorCallback>& callback) override;
    ndk::ScopedAStatus getSupportedAlwaysOnEffects(std::vector<Effect>* _aidl_return) override;
    ndk::ScopedAStatus alwaysOnEnable(int32_t id, Effect effect, EffectStrength strength) override;
    ndk::ScopedAStatus alwaysOnDisable(int32_t id) override;
    ndk::ScopedAStatus getResonantFrequency(float* resonantFreqHz) override;
    ndk::ScopedAStatus getQFactor(float* qFactor) override;
    ndk::ScopedAStatus getFrequencyResolution(float* freqResolutionHz) override;
    ndk::ScopedAStatus getFrequencyMinimum(float* freqMinimumHz) override;
    ndk::ScopedAStatus getBandwidthAmplitudeMap(std::vector<float>* _aidl_return) override;
    ndk::ScopedAStatus getPwlePrimitiveDurationMax(int32_t* durationMs) override;
    ndk::ScopedAStatus getPwleCompositionSizeMax(int32_t* maxSize) override;
    ndk::ScopedAStatus getSupportedBraking(std::vector<Braking>* supported) override;
    ndk::ScopedAStatus composePwle(const std::vector<PrimitivePwle>& composite,
                                   const std::shared_ptr<IVibratorCallback>& callback) override;

  private:
    using Clock = std::chrono::steady_clock;

    ndk::ScopedAStatus play(int32_t durationMs, uint16_t magnitude,
                            const std::shared_ptr<IVibratorCallback>& callback);
    bool openLocked();
    bool uploadLocked(int32_t durationMs, uint16_t magnitude);
    bool startLocked(bool start);

    ::android::base::unique_fd mFd;
    std::mutex mLock;
    std::condition_variable mCv;
    int16_t mEffectId = -1;
    uint16_t mMagnitude = UINT16_MAX;
    bool mPlaying = false;
    Clock::time_point mStopAt;
    uint64_t mGeneration = 0;
};

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
