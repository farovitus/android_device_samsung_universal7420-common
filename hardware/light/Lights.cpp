/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Lights for the NK 6.6 kernel:
 * - panel backlight: the mainline s6e3ha2 backlight device, 0..max_brightness;
 * - touchkey backlight: cypress-mbr-touchkey, any non-zero value is on;
 * - notification LED: the MAX77843 multicolor LED "rgb:status". Colour goes to
 *   multi_intensity ("R G B"), brightness 255 switches it on, and the timer
 *   trigger makes the chip blink by itself (delay_on/delay_off in ms, rounded
 *   by the driver to the chip's steps).
 */

#include "Lights.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>

#include <cstdlib>

using ::android::base::ReadFileToString;
using ::android::base::StringPrintf;
using ::android::base::Trim;
using ::android::base::WriteStringToFile;

namespace aidl {
namespace android {
namespace hardware {
namespace light {

namespace {

constexpr const char* kBacklight = "/sys/class/backlight/s6e3ha2/brightness";
constexpr const char* kBacklightMax = "/sys/class/backlight/s6e3ha2/max_brightness";
constexpr const char* kButtons = "/sys/class/sec/sec_touchkey/brightness";
constexpr const char* kLed = "/sys/class/leds/rgb:status/";

void writeNode(const std::string& path, const std::string& value) {
    if (!WriteStringToFile(value, path)) {
        PLOG(ERROR) << "write " << value << " to " << path;
    }
}

uint32_t rgbToBrightness(uint32_t color) {
    uint32_t r = (color >> 16) & 0xff;
    uint32_t g = (color >> 8) & 0xff;
    uint32_t b = color & 0xff;
    return (77 * r + 150 * g + 29 * b) >> 8;
}

bool isLit(const HwLightState& state) {
    return (state.color & 0x00ffffff) != 0;
}

}  // namespace

Lights::Lights() : mMaxBrightness(100) {
    std::string max;
    if (ReadFileToString(kBacklightMax, &max)) {
        int value = std::atoi(Trim(max).c_str());
        if (value > 0) mMaxBrightness = value;
    }
}

void Lights::setBacklight(const HwLightState& state) {
    uint32_t brightness = rgbToBrightness(state.color);
    uint32_t value = (brightness * mMaxBrightness + 127) / 255;
    writeNode(kBacklight, std::to_string(value));
}

void Lights::setButtons(const HwLightState& state) {
    writeNode(kButtons, isLit(state) ? "1" : "0");
}

void Lights::updateLed() {
    const HwLightState* state = nullptr;
    for (const HwLightState* s : {&mAttention, &mNotification, &mBattery}) {
        if (isLit(*s)) {
            state = s;
            break;
        }
    }

    const std::string led(kLed);
    if (!state) {
        // brightness 0 also ends a blink
        writeNode(led + "trigger", "none");
        writeNode(led + "brightness", "0");
        return;
    }

    // The alpha byte scales the colour, as the framework's battery and
    // notification colours are full-alpha but apps may pass less.
    uint32_t alpha = (state->color >> 24) & 0xff;
    if (alpha == 0) alpha = 0xff;
    auto channel = [&](int shift) { return ((state->color >> shift) & 0xff) * alpha / 0xff; };

    writeNode(led + "trigger", "none");
    writeNode(led + "multi_intensity",
              StringPrintf("%u %u %u", channel(16), channel(8), channel(0)));
    writeNode(led + "brightness", "255");

    if (state->flashMode == FlashMode::TIMED && state->flashOnMs > 0 && state->flashOffMs > 0) {
        writeNode(led + "trigger", "timer");
        writeNode(led + "delay_on", std::to_string(state->flashOnMs));
        writeNode(led + "delay_off", std::to_string(state->flashOffMs));
    }
}

ndk::ScopedAStatus Lights::setLightState(int32_t id, const HwLightState& state) {
    std::lock_guard<std::mutex> lock(mLock);

    switch (static_cast<LightType>(id)) {
        case LightType::BACKLIGHT:
            setBacklight(state);
            break;
        case LightType::BUTTONS:
            setButtons(state);
            break;
        case LightType::BATTERY:
            mBattery = state;
            updateLed();
            break;
        case LightType::NOTIFICATIONS:
            mNotification = state;
            updateLed();
            break;
        case LightType::ATTENTION:
            mAttention = state;
            updateLed();
            break;
        default:
            return ndk::ScopedAStatus::fromExceptionCode(EX_UNSUPPORTED_OPERATION);
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus Lights::getLights(std::vector<HwLight>* lights) {
    for (LightType type : {LightType::BACKLIGHT, LightType::BUTTONS, LightType::BATTERY,
                           LightType::NOTIFICATIONS, LightType::ATTENTION}) {
        HwLight light;
        light.id = static_cast<int32_t>(type);
        light.ordinal = 0;
        light.type = type;
        lights->push_back(light);
    }
    return ndk::ScopedAStatus::ok();
}

}  // namespace light
}  // namespace hardware
}  // namespace android
}  // namespace aidl
