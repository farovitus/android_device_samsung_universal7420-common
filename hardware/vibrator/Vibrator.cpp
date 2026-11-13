/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "vibrator.universal7420"

#include "Vibrator.h"

#include <android-base/logging.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

namespace {

constexpr char kInputDir[] = "/dev/input";
// The mainline MAX77843/MAX77693 haptic driver's input device name.
constexpr char kHapticName[] = "max77693-haptic";
// ff_effect.replay.length is 16 bits wide.
constexpr int32_t kMaxDurationMs = UINT16_MAX;

bool testBit(const unsigned long* bits, int bit) {
    constexpr int kBitsPerLong = sizeof(unsigned long) * 8;
    return (bits[bit / kBitsPerLong] >> (bit % kBitsPerLong)) & 1;
}

int openHaptic() {
    std::unique_ptr<DIR, decltype(&closedir)> dir(opendir(kInputDir), closedir);
    if (!dir) {
        PLOG(ERROR) << "Cannot open " << kInputDir;
        return -1;
    }

    while (struct dirent* entry = readdir(dir.get())) {
        if (strncmp(entry->d_name, "event", 5) != 0) continue;

        std::string path = std::string(kInputDir) + "/" + entry->d_name;
        int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;

        char name[64] = {};
        unsigned long ffBits[(FF_CNT + sizeof(unsigned long) * 8 - 1) /
                             (sizeof(unsigned long) * 8)] = {};
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0 &&
            strcmp(name, kHapticName) == 0 &&
            ioctl(fd, EVIOCGBIT(EV_FF, sizeof(ffBits)), ffBits) >= 0 &&
            testBit(ffBits, FF_RUMBLE)) {
            LOG(INFO) << "Using " << path << " (" << name << ")";
            return fd;
        }
        close(fd);
    }

    LOG(ERROR) << "No " << kHapticName << " input device";
    return -1;
}

ndk::ScopedAStatus unsupported() {
    return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
}

}  // namespace

Vibrator::Vibrator() {
    std::lock_guard<std::mutex> lock(mLock);
    openLocked();
}

bool Vibrator::openLocked() {
    if (mFd.ok()) return true;
    mFd.reset(openHaptic());
    mEffectId = -1;
    return mFd.ok();
}

bool Vibrator::uploadLocked(int32_t durationMs, uint16_t magnitude) {
    struct ff_effect effect = {};
    effect.type = FF_RUMBLE;
    effect.id = mEffectId;
    effect.replay.length = static_cast<uint16_t>(std::clamp(durationMs, 1, kMaxDurationMs));
    effect.u.rumble.strong_magnitude = magnitude;

    if (ioctl(mFd.get(), EVIOCSFF, &effect) < 0) {
        PLOG(ERROR) << "Cannot upload the effect";
        return false;
    }
    mEffectId = effect.id;
    return true;
}

bool Vibrator::startLocked(bool start) {
    struct input_event event = {};
    event.type = EV_FF;
    event.code = static_cast<uint16_t>(mEffectId);
    event.value = start ? 1 : 0;

    if (write(mFd.get(), &event, sizeof(event)) != sizeof(event)) {
        PLOG(ERROR) << "Cannot " << (start ? "start" : "stop") << " the effect";
        return false;
    }
    return true;
}

ndk::ScopedAStatus Vibrator::play(int32_t durationMs, uint16_t magnitude,
                                  const std::shared_ptr<IVibratorCallback>& callback) {
    uint64_t generation;
    Clock::time_point stopAt;
    {
        std::lock_guard<std::mutex> lock(mLock);
        if (!openLocked() || !uploadLocked(durationMs, magnitude) || !startLocked(true)) {
            return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_STATE);
        }
        mPlaying = true;
        mStopAt = stopAt = Clock::now() + std::chrono::milliseconds(durationMs);
        generation = ++mGeneration;
    }
    mCv.notify_all();

    if (callback) {
        std::thread([this, callback, generation, stopAt] {
            std::unique_lock<std::mutex> lock(mLock);
            // A later on(), perform() or off() supersedes this request.
            if (mCv.wait_until(lock, stopAt, [&] { return mGeneration != generation; })) {
                return;
            }
            mPlaying = false;
            lock.unlock();
            callback->onComplete();
        }).detach();
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::getCapabilities(int32_t* _aidl_return) {
    *_aidl_return = IVibrator::CAP_ON_CALLBACK | IVibrator::CAP_PERFORM_CALLBACK |
                    IVibrator::CAP_AMPLITUDE_CONTROL;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::off() {
    {
        std::lock_guard<std::mutex> lock(mLock);
        if (mFd.ok() && mEffectId >= 0 && mPlaying) startLocked(false);
        mPlaying = false;
        ++mGeneration;
    }
    mCv.notify_all();
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::on(int32_t timeoutMs,
                                const std::shared_ptr<IVibratorCallback>& callback) {
    if (timeoutMs <= 0) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    uint16_t magnitude;
    {
        std::lock_guard<std::mutex> lock(mLock);
        magnitude = mMagnitude;
    }
    return play(std::min(timeoutMs, kMaxDurationMs), magnitude, callback);
}

ndk::ScopedAStatus Vibrator::perform(Effect effect, EffectStrength strength,
                                     const std::shared_ptr<IVibratorCallback>& callback,
                                     int32_t* _aidl_return) {
    int32_t durationMs;
    switch (effect) {
        case Effect::TEXTURE_TICK:
            durationMs = 8;
            break;
        case Effect::TICK:
            durationMs = 12;
            break;
        case Effect::CLICK:
            durationMs = 20;
            break;
        case Effect::HEAVY_CLICK:
            durationMs = 30;
            break;
        default:
            return unsupported();
    }

    // Stock TouchWiz touch feedback levels 2, 3 (its default) and 4:
    // 49%, 65% and 82% of the drive range.
    uint16_t magnitude;
    switch (strength) {
        case EffectStrength::LIGHT:
            magnitude = UINT16_MAX * 49 / 100;
            break;
        case EffectStrength::MEDIUM:
            magnitude = UINT16_MAX * 65 / 100;
            break;
        case EffectStrength::STRONG:
            magnitude = UINT16_MAX * 82 / 100;
            break;
        default:
            return unsupported();
    }

    ndk::ScopedAStatus status = play(durationMs, magnitude, callback);
    if (status.isOk()) *_aidl_return = durationMs;
    return status;
}

ndk::ScopedAStatus Vibrator::getSupportedEffects(std::vector<Effect>* _aidl_return) {
    *_aidl_return = {Effect::CLICK, Effect::TICK, Effect::HEAVY_CLICK, Effect::TEXTURE_TICK};
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setAmplitude(float amplitude) {
    if (!(amplitude > 0.0f && amplitude <= 1.0f)) {
        return ndk::ScopedAStatus::fromExceptionCode(EX_ILLEGAL_ARGUMENT);
    }

    std::lock_guard<std::mutex> lock(mLock);
    mMagnitude = static_cast<uint16_t>(std::lround(amplitude * UINT16_MAX));

    // Re-uploading a running effect restarts it with the new magnitude, so
    // give it only the time that is left.
    if (mPlaying && mFd.ok() && mEffectId >= 0) {
        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(mStopAt - Clock::now());
        if (left.count() > 0) uploadLocked(static_cast<int32_t>(left.count()), mMagnitude);
    }
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Vibrator::setExternalControl(bool /* enabled */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getCompositionDelayMax(int32_t* /* maxDelayMs */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getCompositionSizeMax(int32_t* /* maxSize */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getSupportedPrimitives(
        std::vector<CompositePrimitive>* /* supported */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getPrimitiveDuration(CompositePrimitive /* primitive */,
                                                  int32_t* /* durationMs */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::compose(const std::vector<CompositeEffect>& /* composite */,
                                     const std::shared_ptr<IVibratorCallback>& /* callback */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getSupportedAlwaysOnEffects(std::vector<Effect>* /* _aidl_return */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::alwaysOnEnable(int32_t /* id */, Effect /* effect */,
                                            EffectStrength /* strength */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::alwaysOnDisable(int32_t /* id */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getResonantFrequency(float* /* resonantFreqHz */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getQFactor(float* /* qFactor */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getFrequencyResolution(float* /* freqResolutionHz */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getFrequencyMinimum(float* /* freqMinimumHz */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getBandwidthAmplitudeMap(std::vector<float>* /* _aidl_return */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getPwlePrimitiveDurationMax(int32_t* /* durationMs */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getPwleCompositionSizeMax(int32_t* /* maxSize */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::getSupportedBraking(std::vector<Braking>* /* supported */) {
    return unsupported();
}

ndk::ScopedAStatus Vibrator::composePwle(const std::vector<PrimitivePwle>& /* composite */,
                                         const std::shared_ptr<IVibratorCallback>& /* callback */) {
    return unsupported();
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl
