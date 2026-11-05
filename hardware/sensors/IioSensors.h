/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <V2_1/SubHal.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace implementation {
namespace nk {

using ::android::sp;
using ::android::hardware::hidl_handle;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using V1_0::OperationMode;
using V1_0::RateLevel;
using V1_0::Result;
using V1_0::SharedMemInfo;

enum class Kind {
    Accel,
    Gyro,
    Magn,
    Pressure,
    Light,
    Proximity,
    RotationVector,
    GameRotationVector,
    StepCounter,
    StepDetector,
    SigMotion,
};

struct SensorDesc {
    const char* iioName;
    Kind kind;
    SensorType type;
    const char* name;
    const char* vendor;
    float maxRange;
    float resolution;
    float power;
    int32_t minDelayUs;
    int32_t maxDelayUs;
    uint32_t flags;
    /* IIO units per count when the driver exports no _scale attribute */
    double defaultScale;
    /* vendor property that overrides the scale, or nullptr */
    const char* scaleProp;
};

class SensorCallback {
  public:
    virtual ~SensorCallback() = default;
    virtual void postEvents(const std::vector<Event>& events, bool wakeup) = 0;
};

/* One IIO scan element, from scan_elements/<name>_{index,type}. */
struct IioChannel {
    std::string name;
    unsigned index = 0;
    bool isSigned = false;
    bool bigEndian = false;
    unsigned bits = 0;
    unsigned storageBits = 0;
    unsigned shift = 0;
    unsigned repeat = 1;
    size_t offset = 0;
    double scale = 1.0;
    double rawOffset = 0.0;

    size_t bytes() const { return storageBits / 8 * repeat; }
    int64_t raw(const uint8_t* record, unsigned element = 0) const;
    double value(const uint8_t* record, unsigned element = 0) const {
        return (raw(record, element) + rawOffset) * scale;
    }
};

class IioSensor {
  public:
    IioSensor(int32_t handle, const SensorDesc& desc, const std::string& sysfsDir, int devNum,
              SensorCallback* callback);
    ~IioSensor();

    bool init();
    const SensorInfo& info() const { return mInfo; }

    Result activate(bool enable);
    Result batch(int64_t samplingPeriodNs);
    Result flush();
    void reset();
    void dump(std::string* out) const;

  private:
    bool readScanElements();
    bool openDevice();
    bool writeAttr(const std::string& attr, const std::string& value) const;
    void applyRate();
    void run();
    void wake();
    void readRecords(int fd, std::vector<uint8_t>* buf);
    void convert(const uint8_t* record, std::vector<Event>* events);
    bool isOneShot() const;

    const SensorDesc& mDesc;
    const std::string mSysfsDir;
    const std::string mDevNode;
    SensorCallback* const mCallback;
    SensorInfo mInfo;

    std::vector<IioChannel> mChannels; /* data channels, by scan index */
    IioChannel mTimestamp;
    bool mHasTimestamp = false;
    size_t mRecordSize = 0;
    std::string mRateAttr; /* sysfs rate attribute, or empty */
    double mLightGa = 1.0;

    mutable std::mutex mLock;
    bool mEnabled = false;
    int64_t mPeriodNs;
    unsigned mFlushPending = 0;

    /* reader-thread state */
    bool mFirst = true;
    float mLast = 0;
    uint64_t mSteps = 0;
    uint64_t mRecords = 0;

    std::atomic<int> mFd{-1};
    int mWakeFd = -1;
    std::atomic<bool> mStop{false};
    std::thread mThread;
};

class IioSubHal : public ISensorsSubHal, public SensorCallback {
  public:
    IioSubHal();

    Return<void> getSensorsList_2_1(getSensorsList_2_1_cb callback) override;
    Return<Result> setOperationMode(OperationMode mode) override;
    Return<Result> activate(int32_t sensorHandle, bool enabled) override;
    Return<Result> batch(int32_t sensorHandle, int64_t samplingPeriodNs,
                         int64_t maxReportLatencyNs) override;
    Return<Result> flush(int32_t sensorHandle) override;
    Return<Result> injectSensorData_2_1(const Event& event) override;
    Return<void> registerDirectChannel(const SharedMemInfo& mem,
                                       registerDirectChannel_cb callback) override;
    Return<Result> unregisterDirectChannel(int32_t channelHandle) override;
    Return<void> configDirectReport(int32_t sensorHandle, int32_t channelHandle, RateLevel rate,
                                    configDirectReport_cb callback) override;
    Return<void> debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args) override;
    const std::string getName() override { return "universal7420-iio"; }
    Return<Result> initialize(const sp<IHalProxyCallback>& halProxyCallback) override;

    void postEvents(const std::vector<Event>& events, bool wakeup) override;

  private:
    IioSensor* find(int32_t sensorHandle);

    std::vector<std::unique_ptr<IioSensor>> mSensors;
    std::mutex mCallbackLock;
    sp<IHalProxyCallback> mCallback;
};

}  // namespace nk
}  // namespace implementation
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
