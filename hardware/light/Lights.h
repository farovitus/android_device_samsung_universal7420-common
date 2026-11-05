/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <aidl/android/hardware/light/BnLights.h>

#include <mutex>

namespace aidl {
namespace android {
namespace hardware {
namespace light {

class Lights : public BnLights {
  public:
    Lights();

    ndk::ScopedAStatus setLightState(int32_t id, const HwLightState& state) override;
    ndk::ScopedAStatus getLights(std::vector<HwLight>* lights) override;

  private:
    void setBacklight(const HwLightState& state);
    void setButtons(const HwLightState& state);
    void updateLed();

    std::mutex mLock;
    int mMaxBrightness;
    HwLightState mBattery;
    HwLightState mNotification;
    HwLightState mAttention;
};

}  // namespace light
}  // namespace hardware
}  // namespace android
}  // namespace aidl
