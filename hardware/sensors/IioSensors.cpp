/*
 * Copyright (C) 2026 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Sensors sub-HAL for the BCM4773 sensor hub on the NK 6.6 kernel.
 *
 * The kernel's ssp_sensors drivers expose each hub sensor as an IIO device
 * with a kfifo buffer. This sub-HAL finds them by name, turns every scan
 * element on, reads records from /dev/iio:deviceN and converts them to
 * Android units. The AOSP multihal service loads it through hals.conf and
 * owns the framework queues and wake locks.
 *
 * Record layout follows iio_compute_scan_bytes(): each element aligned to
 * its own size, the record padded to its largest element. Timestamps come
 * from the hub driver in CLOCK_BOOTTIME nanoseconds.
 */

#include "IioSensors.h"

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/parsedouble.h>
#include <android-base/parseint.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>
#include <android-base/strings.h>
#include <utils/SystemClock.h>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace implementation {
namespace nk {

using ::android::base::StringPrintf;

namespace {

constexpr uint32_t kContinuous = static_cast<uint32_t>(V1_0::SensorFlagBits::CONTINUOUS_MODE);
constexpr uint32_t kOnChange = static_cast<uint32_t>(V1_0::SensorFlagBits::ON_CHANGE_MODE);
constexpr uint32_t kOneShot = static_cast<uint32_t>(V1_0::SensorFlagBits::ONE_SHOT_MODE);
constexpr uint32_t kSpecial = static_cast<uint32_t>(V1_0::SensorFlagBits::SPECIAL_REPORTING_MODE);
constexpr uint32_t kWakeUp = static_cast<uint32_t>(V1_0::SensorFlagBits::WAKE_UP);
constexpr uint32_t kModeMask = static_cast<uint32_t>(V1_0::SensorFlagBits::MASK_REPORTING_MODE);

constexpr double kG = 9.80665;
constexpr double kDegToRad = M_PI / 180.0;

const char kIioDevices[] = "/sys/bus/iio/devices";

/*
 * Default scales are the stock chip settings: MPU6500 at +-4 g
 * (8192 LSB/g) and +-2000 dps (16.4 LSB/dps). The hub's magnetometer unit
 * is not known; set persist.vendor.sensors.nk.magn_scale (uT per count) after
 * checking the field magnitude. persist.vendor.sensors.nk.light_ga is the glass
 * attenuation factor of the lux formula.
 */
const SensorDesc kSensors[] = {
        {"ssp-accelerometer", Kind::Accel, SensorType::ACCELEROMETER, "MPU6500 Accelerometer",
         "InvenSense", static_cast<float>(4 * kG), static_cast<float>(kG / 8192), 0.4f, 10000,
         1000000, kContinuous, kG / 8192, nullptr},
        {"ssp-gyroscope", Kind::Gyro, SensorType::GYROSCOPE, "MPU6500 Gyroscope", "InvenSense",
         static_cast<float>(2000 * kDegToRad), static_cast<float>(kDegToRad / 16.4), 3.2f, 10000,
         1000000, kContinuous, kDegToRad / 16.4, nullptr},
        {"ssp-magnetometer", Kind::Magn, SensorType::MAGNETIC_FIELD, "YAS537 Magnetometer",
         "Yamaha", 2000.0f, 0.1f, 1.1f, 20000, 1000000, kContinuous, 0.1,
         "persist.vendor.sensors.nk.magn_scale"},
        {"ssp-pressure", Kind::Pressure, SensorType::PRESSURE, "LPS25H Barometer",
         "STMicroelectronics", 1260.0f, 1.0f / 4096, 0.025f, 100000, 1000000, kContinuous,
         1.0 / 40960, nullptr},
        {"ssp-light", Kind::Light, SensorType::LIGHT, "TMD4903 Light", "ams", 60000.0f, 1.0f, 0.25f,
         0, 0, kOnChange, 1.0, "persist.vendor.sensors.nk.light_ga"},
        {"ssp-proximity", Kind::Proximity, SensorType::PROXIMITY, "TMD4903 Proximity", "ams", 5.0f,
         5.0f, 0.25f, 0, 0, kOnChange | kWakeUp, 1.0, nullptr},
        {"ssp-rotation-vector", Kind::RotationVector, SensorType::ROTATION_VECTOR,
         "Rotation Vector", "Broadcom", 1.0f, 0.0001f, 4.7f, 10000, 1000000, kContinuous, 1.0,
         nullptr},
        {"ssp-game-rotvec", Kind::GameRotationVector, SensorType::GAME_ROTATION_VECTOR,
         "Game Rotation Vector", "Broadcom", 1.0f, 0.0001f, 3.6f, 10000, 1000000, kContinuous, 1.0,
         nullptr},
        {"ssp-step-counter", Kind::StepCounter, SensorType::STEP_COUNTER, "Step Counter",
         "Broadcom", 4294967295.0f, 1.0f, 0.4f, 0, 0, kOnChange, 1.0, nullptr},
        {"ssp-step-detector", Kind::StepDetector, SensorType::STEP_DETECTOR, "Step Detector",
         "Broadcom", 1.0f, 1.0f, 0.4f, 0, 0, kSpecial, 1.0, nullptr},
        {"ssp-sig-motion", Kind::SigMotion, SensorType::SIGNIFICANT_MOTION,
         "Significant Motion", "Broadcom", 1.0f, 1.0f, 0.4f, -1, 0, kOneShot | kWakeUp, 1.0,
         nullptr},
};

bool readTrimmed(const std::string& path, std::string* out) {
    if (!android::base::ReadFileToString(path, out)) return false;
    *out = android::base::Trim(*out);
    return true;
}

bool readDouble(const std::string& path, double* out) {
    std::string s;
    return readTrimmed(path, &s) && android::base::ParseDouble(s, out);
}

/* "le:s16/16>>0" or, with repeat, "le:s32/32X4>>0" */
bool parseType(const std::string& s, IioChannel* c) {
    char endian, sign;
    unsigned bits, storage, repeat = 1, shift;

    if (sscanf(s.c_str(), "%ce:%c%u/%uX%u>>%u", &endian, &sign, &bits, &storage, &repeat,
               &shift) != 6) {
        repeat = 1;
        if (sscanf(s.c_str(), "%ce:%c%u/%u>>%u", &endian, &sign, &bits, &storage, &shift) != 5)
            return false;
    }
    if (storage == 0 || storage > 64 || storage % 8 || bits == 0 || bits > storage || !repeat)
        return false;

    c->bigEndian = endian == 'b';
    c->isSigned = sign == 's';
    c->bits = bits;
    c->storageBits = storage;
    c->repeat = repeat;
    c->shift = shift;
    return true;
}

size_t alignUp(size_t x, size_t a) {
    return (x + a - 1) / a * a;
}

/*
 * ams DN40 lux for an RGBC sensor: remove the IR part, weight the
 * channels, divide by counts per lux. ATIME and AGAIN are the TMD4903
 * register codes the hub reports with each sample.
 */
float computeLux(double r, double g, double b, double c, unsigned atime, unsigned again,
                 double ga) {
    static const double kGain[] = {1, 4, 16, 64};
    constexpr double kDgf = 310, kRCoef = 0.136, kGCoef = 1.0, kBCoef = -0.444;

    const double atimeMs = (256 - (atime & 0xff)) * 2.78;
    const double cpl = atimeMs * kGain[again & 3] / (ga * kDgf);
    const double ir = std::max(0.0, (r + g + b - c) / 2);
    const double lux = (kRCoef * (r - ir) + kGCoef * (g - ir) + kBCoef * (b - ir)) / cpl;

    return static_cast<float>(std::max(0.0, lux));
}

}  // namespace

int64_t IioChannel::raw(const uint8_t* record, unsigned element) const {
    const unsigned size = storageBits / 8;
    const uint8_t* p = record + offset + static_cast<size_t>(element) * size;
    uint64_t v = 0;

    for (unsigned i = 0; i < size; i++) v = (v << 8) | p[bigEndian ? i : size - 1 - i];

    v >>= shift;
    if (bits < 64) {
        const uint64_t mask = (uint64_t(1) << bits) - 1;

        v &= mask;
        if (isSigned && (v & (uint64_t(1) << (bits - 1)))) v |= ~mask;
    }
    return static_cast<int64_t>(v);
}

IioSensor::IioSensor(int32_t handle, const SensorDesc& desc, const std::string& sysfsDir,
                     int devNum, SensorCallback* callback)
    : mDesc(desc),
      mSysfsDir(sysfsDir),
      mDevNode(StringPrintf("/dev/iio:device%d", devNum)),
      mCallback(callback),
      mPeriodNs(static_cast<int64_t>(desc.maxDelayUs) * 1000) {
    mInfo.sensorHandle = handle;
    mInfo.name = desc.name;
    mInfo.vendor = desc.vendor;
    mInfo.version = 1;
    mInfo.type = desc.type;
    mInfo.typeAsString = "";
    mInfo.maxRange = desc.maxRange;
    mInfo.resolution = desc.resolution;
    mInfo.power = desc.power;
    mInfo.minDelay = desc.minDelayUs;
    mInfo.fifoReservedEventCount = 0;
    mInfo.fifoMaxEventCount = 0;
    mInfo.requiredPermission = "";
    mInfo.maxDelay = desc.maxDelayUs;
    mInfo.flags = desc.flags;
}

IioSensor::~IioSensor() {
    mStop = true;
    wake();
    if (mThread.joinable()) mThread.join();
    if (mFd >= 0) close(mFd);
    if (mWakeFd >= 0) close(mWakeFd);
}

bool IioSensor::isOneShot() const {
    return (mInfo.flags & kModeMask) == kOneShot;
}

bool IioSensor::readScanElements() {
    const std::string dir = mSysfsDir + "/scan_elements";
    std::unique_ptr<DIR, decltype(&closedir)> d(opendir(dir.c_str()), closedir);
    std::vector<IioChannel> all;

    if (!d) {
        PLOG(ERROR) << "cannot open " << dir;
        return false;
    }

    while (struct dirent* e = readdir(d.get())) {
        const std::string f = e->d_name;
        IioChannel c;
        std::string s;

        if (!android::base::EndsWith(f, "_en")) continue;
        c.name = f.substr(0, f.size() - 3);
        if (!readTrimmed(dir + "/" + c.name + "_index", &s) ||
            !android::base::ParseUint(s, &c.index) ||
            !readTrimmed(dir + "/" + c.name + "_type", &s) || !parseType(s, &c)) {
            LOG(ERROR) << mDesc.iioName << ": bad scan element " << c.name;
            return false;
        }
        all.push_back(c);
    }

    std::sort(all.begin(), all.end(),
              [](const IioChannel& a, const IioChannel& b) { return a.index < b.index; });

    size_t offset = 0, largest = 1;
    for (auto& c : all) {
        c.offset = alignUp(offset, c.bytes());
        offset = c.offset + c.bytes();
        largest = std::max(largest, c.bytes());
    }
    mRecordSize = alignUp(offset, largest);

    for (auto& c : all) {
        if (c.name == "in_timestamp") {
            mTimestamp = c;
            mHasTimestamp = true;
            continue;
        }

        /* in_accel_x_scale, then the shared in_accel_scale */
        bool found = false;
        for (std::string base = c.name; !found;) {
            found = readDouble(mSysfsDir + "/" + base + "_scale", &c.scale);
            if (found) {
                readDouble(mSysfsDir + "/" + base + "_offset", &c.rawOffset);
                break;
            }
            const size_t p = base.rfind('_');
            if (p == std::string::npos || p <= 2) break;
            base.resize(p);
        }
        if (!found) c.scale = mDesc.defaultScale;
        mChannels.push_back(c);
    }

    return !mChannels.empty() && mRecordSize > 0;
}

bool IioSensor::init() {
    if (!readScanElements()) return false;

    /*
     * The rate is shared by all channels (sampling_frequency) on the hub
     * drivers, and by channel type (in_accel_sampling_frequency) on the
     * mainline accel and gyro drivers.
     */
    std::vector<std::string> rateAttrs{"sampling_frequency"};
    if (!mChannels.empty()) {
        const std::string& n = mChannels[0].name;
        const size_t cut = n.rfind('_');
        if (cut != std::string::npos && cut > 0)
            rateAttrs.push_back(n.substr(0, cut) + "_sampling_frequency");
    }
    for (const auto& a : rateAttrs) {
        if (access((mSysfsDir + "/" + a).c_str(), W_OK) == 0) {
            mRateAttr = a;
            break;
        }
    }

    if (mDesc.scaleProp) {
        double v;
        const std::string s = android::base::GetProperty(mDesc.scaleProp, "");

        if (!s.empty() && android::base::ParseDouble(s, &v) && v > 0) {
            if (mDesc.kind == Kind::Light) {
                mLightGa = v;
            } else {
                for (auto& c : mChannels) c.scale = v;
            }
            LOG(INFO) << mDesc.iioName << ": " << mDesc.scaleProp << " = " << v;
        }
    }

    mWakeFd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (mWakeFd < 0) {
        PLOG(ERROR) << "eventfd";
        return false;
    }
    mThread = std::thread(&IioSensor::run, this);
    return true;
}

bool IioSensor::writeAttr(const std::string& attr, const std::string& value) const {
    const std::string path = mSysfsDir + "/" + attr;

    if (!android::base::WriteStringToFile(value, path)) {
        PLOG(ERROR) << "cannot write " << value << " to " << path;
        return false;
    }
    return true;
}

/* Called with mLock held. */
bool IioSensor::openDevice() {
    if (mFd >= 0) return true;

    for (const auto& c : mChannels) writeAttr("scan_elements/" + c.name + "_en", "1");
    if (mHasTimestamp) writeAttr("scan_elements/in_timestamp_en", "1");
    writeAttr("buffer/length", "128");

    const int fd = open(mDevNode.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        PLOG(ERROR) << "cannot open " << mDevNode;
        return false;
    }
    mFd = fd;
    wake();
    return true;
}

/* Called with mLock held. The hub takes a new rate only while enabled. */
void IioSensor::applyRate() {
    if ((mInfo.flags & kModeMask) != kContinuous || mRateAttr.empty()) return;

    const int64_t minNs = static_cast<int64_t>(mDesc.minDelayUs) * 1000;
    const int64_t maxNs = static_cast<int64_t>(mDesc.maxDelayUs) * 1000;
    const int64_t period = std::clamp(mPeriodNs, minNs, maxNs);
    const int64_t uhz = 1000000000000000LL / period;

    writeAttr(mRateAttr,
              StringPrintf("%lld.%06lld", static_cast<long long>(uhz / 1000000),
                           static_cast<long long>(uhz % 1000000)));
}

Result IioSensor::activate(bool enable) {
    std::lock_guard<std::mutex> lock(mLock);

    if (enable == mEnabled) return Result::OK;

    if (enable) {
        if (!openDevice() || !writeAttr("buffer/enable", "1")) return Result::BAD_VALUE;
        mEnabled = true;
        mFirst = true;
        applyRate();
    } else {
        writeAttr("buffer/enable", "0");
        mEnabled = false;
        mFlushPending = 0;
    }
    return Result::OK;
}

Result IioSensor::batch(int64_t samplingPeriodNs) {
    std::lock_guard<std::mutex> lock(mLock);

    if (samplingPeriodNs < 0) return Result::BAD_VALUE;
    mPeriodNs = samplingPeriodNs;
    if (mEnabled) applyRate();
    return Result::OK;
}

Result IioSensor::flush() {
    std::lock_guard<std::mutex> lock(mLock);

    if (isOneShot() || !mEnabled) return Result::BAD_VALUE;
    mFlushPending++;
    wake();
    return Result::OK;
}

void IioSensor::reset() {
    activate(false);
}

void IioSensor::wake() {
    const uint64_t one = 1;

    if (mWakeFd >= 0) (void)!write(mWakeFd, &one, sizeof(one));
}

void IioSensor::run() {
    std::vector<uint8_t> buf(mRecordSize * 32);

    while (!mStop) {
        const int fd = mFd;
        struct pollfd fds[2] = {{mWakeFd, POLLIN, 0}, {fd, POLLIN, 0}};

        if (poll(fds, fd >= 0 ? 2 : 1, -1) < 0) {
            if (errno != EINTR) {
                PLOG(ERROR) << mDesc.iioName << ": poll";
                usleep(100000);
            }
            continue;
        }

        if (fds[0].revents & POLLIN) {
            uint64_t v;
            (void)!read(mWakeFd, &v, sizeof(v));
        }

        if (fd >= 0) {
            readRecords(fd, &buf);
            if (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                LOG(ERROR) << mDesc.iioName << ": " << mDevNode << " poll error";
                usleep(500000);
            }
        }

        unsigned flushes;
        {
            std::lock_guard<std::mutex> lock(mLock);
            flushes = mFlushPending;
            mFlushPending = 0;
        }
        if (flushes) {
            Event e{};

            e.sensorHandle = mInfo.sensorHandle;
            e.sensorType = SensorType::META_DATA;
            e.timestamp = 0;
            e.u.meta.what = V1_0::MetaDataEventType::META_DATA_FLUSH_COMPLETE;
            mCallback->postEvents(std::vector<Event>(flushes, e), false);
        }
    }
}

void IioSensor::readRecords(int fd, std::vector<uint8_t>* buf) {
    std::vector<Event> events;

    for (;;) {
        const ssize_t n = read(fd, buf->data(), buf->size());

        if (n < 0 && errno == EINTR) continue;
        if (n < 0) {
            if (errno != EAGAIN) PLOG(ERROR) << mDesc.iioName << ": read";
            break;
        }
        if (n == 0) break;

        std::lock_guard<std::mutex> lock(mLock);
        for (size_t off = 0; mEnabled && off + mRecordSize <= static_cast<size_t>(n);
             off += mRecordSize) {
            convert(buf->data() + off, &events);
            mRecords++;
        }
    }

    if (!events.empty()) mCallback->postEvents(events, mInfo.flags & kWakeUp);
}

/* Called with mLock held. */
void IioSensor::convert(const uint8_t* record, std::vector<Event>* events) {
    Event e{};
    auto value = [&](size_t i) { return static_cast<float>(mChannels[i].value(record)); };

    e.sensorHandle = mInfo.sensorHandle;
    e.sensorType = mInfo.type;
    e.timestamp = mHasTimestamp ? mTimestamp.raw(record) : android::elapsedRealtimeNano();

    switch (mDesc.kind) {
        case Kind::Accel:
        case Kind::Gyro:
        case Kind::Magn:
            if (mChannels.size() < 3) return;
            e.u.vec3.x = value(0);
            e.u.vec3.y = value(1);
            e.u.vec3.z = value(2);
            e.u.vec3.status = V1_0::SensorStatus::ACCURACY_HIGH;
            break;
        case Kind::Pressure:
            e.u.scalar = value(0) * 10.0f; /* kPa to hPa */
            break;
        case Kind::Light: {
            if (mChannels.size() < 6) return;
            const float lux = computeLux(
                    mChannels[0].raw(record), mChannels[1].raw(record), mChannels[2].raw(record),
                    mChannels[3].raw(record), mChannels[4].raw(record), mChannels[5].raw(record),
                    mLightGa);
            if (!mFirst && lux == mLast) return;
            mLast = e.u.scalar = lux;
            break;
        }
        case Kind::Proximity: {
            /* hub state 1 is near */
            const float distance = mChannels[0].raw(record) ? 0.0f : mInfo.maxRange;
            if (!mFirst && distance == mLast) return;
            mLast = e.u.scalar = distance;
            break;
        }
        case Kind::RotationVector:
        case Kind::GameRotationVector: {
            /* x, y, z, w in an arbitrary fixed-point unit: normalise */
            double q[4], norm = 0;

            if (mChannels[0].repeat < 4) return;
            for (unsigned i = 0; i < 4; i++) {
                q[i] = mChannels[0].raw(record, i);
                norm += q[i] * q[i];
            }
            norm = std::sqrt(norm);
            if (norm == 0) return;
            if (q[3] < 0) norm = -norm;
            for (auto& v : q) v /= norm;

            if (mDesc.kind == Kind::RotationVector) {
                for (unsigned i = 0; i < 4; i++) e.u.data[i] = static_cast<float>(q[i]);
                e.u.data[4] = -1.0f; /* heading accuracy unknown */
            } else {
                e.u.vec4.x = static_cast<float>(q[0]);
                e.u.vec4.y = static_cast<float>(q[1]);
                e.u.vec4.z = static_cast<float>(q[2]);
                e.u.vec4.w = static_cast<float>(q[3]);
            }
            break;
        }
        case Kind::StepCounter: {
            /* the hub reports steps since its previous sample */
            const uint64_t diff = static_cast<uint32_t>(mChannels[0].raw(record));

            mSteps += diff;
            if (!mFirst && !diff) return;
            e.u.stepCount = mSteps;
            break;
        }
        case Kind::StepDetector:
            e.u.scalar = 1.0f;
            break;
        case Kind::SigMotion:
            e.u.scalar = 1.0f;
            /* one-shot: disarm after the first trigger */
            writeAttr("buffer/enable", "0");
            mEnabled = false;
            break;
    }

    mFirst = false;
    events->push_back(e);
}

void IioSensor::dump(std::string* out) const {
    std::lock_guard<std::mutex> lock(mLock);

    *out += StringPrintf("%d %s (%s, %s): %s period %lld ns, %llu records, record %zu bytes\n",
                         mInfo.sensorHandle, mInfo.name.c_str(), mDesc.iioName, mDevNode.c_str(),
                         mEnabled ? "on" : "off", static_cast<long long>(mPeriodNs),
                         static_cast<unsigned long long>(mRecords), mRecordSize);
    for (const auto& c : mChannels)
        *out += StringPrintf("    %s: offset %zu %c%u/%ux%u scale %g offset %g\n", c.name.c_str(),
                             c.offset, c.isSigned ? 's' : 'u', c.bits, c.storageBits, c.repeat,
                             c.scale, c.rawOffset);
    if (mHasTimestamp) *out += StringPrintf("    in_timestamp: offset %zu\n", mTimestamp.offset);
}

IioSubHal::IioSubHal() {
    std::map<std::string, std::pair<std::string, int>> devices;

    /*
     * The modules load in early-init, but a deferred probe can finish
     * later: give the hub's devices a few seconds to register.
     */
    for (int tries = 0; tries < 50; tries++) {
        std::unique_ptr<DIR, decltype(&closedir)> d(opendir(kIioDevices), closedir);

        devices.clear();
        while (struct dirent* e = d ? readdir(d.get()) : nullptr) {
            int num;
            std::string name;

            if (sscanf(e->d_name, "iio:device%d", &num) != 1) continue;
            const std::string dir = std::string(kIioDevices) + "/" + e->d_name;
            if (readTrimmed(dir + "/name", &name)) devices[name] = {dir, num};
        }
        if (devices.count(kSensors[0].iioName)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    int32_t handle = 1;
    for (const auto& desc : kSensors) {
        const auto it = devices.find(desc.iioName);

        if (it == devices.end()) {
            LOG(WARNING) << "no IIO device " << desc.iioName;
            continue;
        }

        auto sensor = std::make_unique<IioSensor>(handle, desc, it->second.first,
                                                  it->second.second, this);
        if (!sensor->init()) {
            LOG(ERROR) << "cannot set up " << desc.iioName;
            continue;
        }
        LOG(INFO) << "sensor " << handle << ": " << desc.name << " on " << it->second.first;
        handle++;
        mSensors.push_back(std::move(sensor));
    }
}

IioSensor* IioSubHal::find(int32_t sensorHandle) {
    for (auto& s : mSensors)
        if (s->info().sensorHandle == sensorHandle) return s.get();
    return nullptr;
}

Return<void> IioSubHal::getSensorsList_2_1(getSensorsList_2_1_cb callback) {
    std::vector<SensorInfo> list;

    for (const auto& s : mSensors) list.push_back(s->info());
    callback(list);
    return Void();
}

Return<Result> IioSubHal::setOperationMode(OperationMode mode) {
    return mode == OperationMode::NORMAL ? Result::OK : Result::BAD_VALUE;
}

Return<Result> IioSubHal::activate(int32_t sensorHandle, bool enabled) {
    IioSensor* s = find(sensorHandle);

    return s ? s->activate(enabled) : Result::BAD_VALUE;
}

Return<Result> IioSubHal::batch(int32_t sensorHandle, int64_t samplingPeriodNs,
                                int64_t /* maxReportLatencyNs */) {
    IioSensor* s = find(sensorHandle);

    return s ? s->batch(samplingPeriodNs) : Result::BAD_VALUE;
}

Return<Result> IioSubHal::flush(int32_t sensorHandle) {
    IioSensor* s = find(sensorHandle);

    return s ? s->flush() : Result::BAD_VALUE;
}

Return<Result> IioSubHal::injectSensorData_2_1(const Event& /* event */) {
    return Result::INVALID_OPERATION;
}

Return<void> IioSubHal::registerDirectChannel(const SharedMemInfo& /* mem */,
                                              registerDirectChannel_cb callback) {
    callback(Result::INVALID_OPERATION, -1);
    return Void();
}

Return<Result> IioSubHal::unregisterDirectChannel(int32_t /* channelHandle */) {
    return Result::INVALID_OPERATION;
}

Return<void> IioSubHal::configDirectReport(int32_t /* sensorHandle */,
                                           int32_t /* channelHandle */, RateLevel /* rate */,
                                           configDirectReport_cb callback) {
    callback(Result::INVALID_OPERATION, 0);
    return Void();
}

Return<void> IioSubHal::debug(const hidl_handle& fd, const hidl_vec<hidl_string>& /* args */) {
    std::string out;

    if (fd.getNativeHandle() == nullptr || fd->numFds < 1) return Void();
    for (const auto& s : mSensors) s->dump(&out);
    android::base::WriteStringToFd(out, fd->data[0]);
    return Void();
}

Return<Result> IioSubHal::initialize(const sp<IHalProxyCallback>& halProxyCallback) {
    /* the framework restarted: start from all sensors off */
    for (auto& s : mSensors) s->reset();

    std::lock_guard<std::mutex> lock(mCallbackLock);
    mCallback = halProxyCallback;
    return Result::OK;
}

void IioSubHal::postEvents(const std::vector<Event>& events, bool wakeup) {
    sp<IHalProxyCallback> callback;

    {
        std::lock_guard<std::mutex> lock(mCallbackLock);
        callback = mCallback;
    }
    if (callback == nullptr) return;
    callback->postEvents(events, callback->createScopedWakelock(wakeup));
}

}  // namespace nk
}  // namespace implementation
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android

using ::android::hardware::sensors::V2_1::implementation::ISensorsSubHal;

ISensorsSubHal* sensorsHalGetSubHal_2_1(uint32_t* version) {
    static ::android::hardware::sensors::V2_1::implementation::nk::IioSubHal subHal;

    *version = SUB_HAL_2_1_VERSION;
    return &subHal;
}
