#pragma once

#include "soft_isp.h"
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace s6cam {

struct V4l2DeviceNodes {
    int32_t cameraId = 0;
    bool isFront = false;
    std::string sensorName;
    std::string mediaDev;
    std::string videoDev;
    std::string csisSubdev;
    std::string sensorSubdev;
    std::string lensSubdev;  // Optional AF VCM subdev (e.g., ak7345)
    BayerPattern pattern = BayerPattern::GRBG;
    uint32_t mbusCode = 0;
    uint32_t v4l2PixFmt = 0;
    int32_t previewWidth = 2664;
    int32_t previewHeight = 1500;
    int32_t fullWidth = 5328;
    int32_t fullHeight = 3000;
};

struct RawFrameView {
    const uint16_t* data = nullptr;
    int32_t width = 0;
    int32_t height = 0;
    int32_t strideBytes = 0;
    uint32_t bufferIndex = 0;
    int64_t timestampNs = 0;
};

/**
 * Discovers mainline Exynos7420 V4L2/MediaController camera pipelines dynamically.
 */
std::vector<V4l2DeviceNodes> discoverV4l2Cameras();

/**
 * Manages open FDs, media links, subdev pad formats, VB2 MMAP buffers, and V4L2 controls
 * for a single Exynos7420 FIMC-LITE camera pipeline.
 */
class V4l2Pipeline {
public:
    explicit V4l2Pipeline(const V4l2DeviceNodes& nodes);
    ~V4l2Pipeline();

    V4l2Pipeline(const V4l2Pipeline&) = delete;
    V4l2Pipeline& operator=(const V4l2Pipeline&) = delete;

    const V4l2DeviceNodes& nodes() const { return nodes_; }

    bool openNodes();
    void closeNodes();

    bool startStream(int32_t width, int32_t height, uint32_t reqBufCount);
    void stopStream();
    bool isStreaming() const { return streaming_; }
    int32_t currentWidth() const { return curW_; }
    int32_t currentHeight() const { return curH_; }

    SensorCtrlLimits querySensorLimits();
    void applyExposureGain(int32_t exposureLines, int32_t again, int32_t dgain);

    bool setFocusPos(int32_t focusPos);
    bool getFocusRange(int32_t* minVal, int32_t* maxVal);

    bool dequeueFrame(RawFrameView* outFrame, int timeoutMs = 2000);
    bool requeueFrame(uint32_t bufferIndex);

private:
    struct MmapBuf {
        void* ptr = nullptr;
        size_t length = 0;
    };

    bool enableMediaLinks();
    bool setSubdevPadFormat(int fd, uint32_t pad, int32_t width, int32_t height, uint32_t code);
    bool queryCtrl(int fd, uint32_t cid, int64_t* minV, int64_t* maxV, int64_t* defV, int64_t* curV);
    bool setCtrl(int fd, uint32_t cid, int32_t val);

    V4l2DeviceNodes nodes_;
    int mediaFd_ = -1;
    int videoFd_ = -1;
    int csisFd_ = -1;
    int sensorFd_ = -1;
    int lensFd_ = -1;

    bool streaming_ = false;
    bool oisActive_ = false;
    int32_t curW_ = 0;
    int32_t curH_ = 0;
    int32_t curStrideBytes_ = 0;
    std::vector<MmapBuf> buffers_;

    int32_t lastExpo_ = -1;
    int32_t lastAgain_ = -1;
    int32_t lastDgain_ = -1;
};

}  // namespace s6cam
