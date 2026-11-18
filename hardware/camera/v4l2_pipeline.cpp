#define LOG_TAG "s6cam_v4l2"
#include "v4l2_pipeline.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <linux/media.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <log/log.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <unordered_map>

namespace s6cam {

namespace {

bool writeOisSysfs(const char* val) {
    int fd = open("/sys/class/camera/ois/ois_power", O_WRONLY | O_CLOEXEC);
    if (fd < 0) return false;
    ssize_t len = static_cast<ssize_t>(std::strlen(val));
    ssize_t n = write(fd, val, len);
    close(fd);
    return n == len;
}

int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

std::unordered_map<uint64_t, std::string> scanDevNodesByPrefix(const char* prefix) {
    std::unordered_map<uint64_t, std::string> result;
    DIR* dir = opendir("/dev");
    if (!dir) return result;
    const size_t plen = std::strlen(prefix);
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (std::strncmp(ent->d_name, prefix, plen) != 0) continue;
        std::string full = std::string("/dev/") + ent->d_name;
        struct stat st{};
        if (stat(full.c_str(), &st) == 0 && S_ISCHR(st.st_mode)) {
            uint64_t key = (static_cast<uint64_t>(major(st.st_rdev)) << 32) | minor(st.st_rdev);
            result.emplace(key, full);
        }
    }
    closedir(dir);
    return result;
}

inline uint64_t devKey(uint32_t maj, uint32_t min) {
    return (static_cast<uint64_t>(maj) << 32) | min;
}

struct DiscoveredMedia {
    std::string mediaPath;
    bool hasRear = false;
    bool hasFront = false;
    V4l2DeviceNodes node;
};

bool inspectMediaDevice(const std::string& mediaPath,
                        const std::unordered_map<uint64_t, std::string>& videoNodes,
                        const std::unordered_map<uint64_t, std::string>& subdevNodes,
                        V4l2DeviceNodes* out) {
    int mfd = open(mediaPath.c_str(), O_RDWR | O_CLOEXEC);
    if (mfd < 0) return false;

    struct media_device_info minfo{};
    if (xioctl(mfd, MEDIA_IOC_DEVICE_INFO, &minfo) < 0) {
        close(mfd);
        return false;
    }

    V4l2DeviceNodes n{};
    n.mediaDev = mediaPath;
    bool foundSensor = false;
    bool foundCsis = false;
    bool foundVideo = false;

    uint32_t eid = 0;
    while (true) {
        struct media_entity_desc ent{};
        ent.id = eid | MEDIA_ENT_ID_FLAG_NEXT;
        if (xioctl(mfd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0) break;
        eid = ent.id;

        uint64_t dk = devKey(ent.dev.major, ent.dev.minor);
        if (std::strncmp(ent.name, "s5c73c2", 7) == 0) {
            auto it = subdevNodes.find(dk);
            if (it != subdevNodes.end()) {
                n.sensorSubdev = it->second;
                n.sensorName = ent.name;
                n.isFront = false;
                n.cameraId = 0;
                n.pattern = BayerPattern::GRBG;
                n.mbusCode = MEDIA_BUS_FMT_SGRBG10_1X10;
                n.v4l2PixFmt = V4L2_PIX_FMT_SGRBG10;
                n.previewWidth = 2664;
                n.previewHeight = 1500;
                n.fullWidth = 5328;
                n.fullHeight = 3000;
                foundSensor = true;
            }
        } else if (std::strncmp(ent.name, "s5k4e6", 6) == 0) {
            auto it = subdevNodes.find(dk);
            if (it != subdevNodes.end()) {
                n.sensorSubdev = it->second;
                n.sensorName = ent.name;
                n.isFront = true;
                n.cameraId = 1;
                n.pattern = BayerPattern::GRBG;
                n.mbusCode = MEDIA_BUS_FMT_SGRBG10_1X10;
                n.v4l2PixFmt = V4L2_PIX_FMT_SGRBG10;
                n.previewWidth = 2608;
                n.previewHeight = 1960;
                n.fullWidth = 2608;
                n.fullHeight = 1960;
                foundSensor = true;
            }
        } else if (std::strstr(ent.name, ".csis") != nullptr) {
            auto it = subdevNodes.find(dk);
            if (it != subdevNodes.end()) {
                n.csisSubdev = it->second;
                foundCsis = true;
            }
        } else if (std::strstr(ent.name, ".fimc-lite") != nullptr) {
            auto it = videoNodes.find(dk);
            if (it != videoNodes.end()) {
                n.videoDev = it->second;
                foundVideo = true;
            }
        } else if (ent.type == MEDIA_ENT_T_V4L2_SUBDEV_LENS ||
                   std::strncmp(ent.name, "ak73", 4) == 0) {
            auto it = subdevNodes.find(dk);
            if (it != subdevNodes.end()) {
                n.lensSubdev = it->second;
            }
        }
    }

    close(mfd);
    if (foundSensor && foundCsis && foundVideo) {
        *out = n;
        return true;
    }
    return false;
}

}  // namespace

std::vector<V4l2DeviceNodes> discoverV4l2Cameras() {
    auto videoNodes = scanDevNodesByPrefix("video");
    auto subdevNodes = scanDevNodesByPrefix("v4l-subdev");

    // Pick up standalone lens subdevs even if not yet linked to a specific media graph
    std::string fallbackLens;
    for (int i = 0; i < 16; ++i) {
        std::string sp = "/dev/v4l-subdev" + std::to_string(i);
        int fd = open(sp.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;
        struct v4l2_control c{};
        c.id = V4L2_CID_FOCUS_ABSOLUTE;
        if (xioctl(fd, VIDIOC_G_CTRL, &c) == 0) {
            fallbackLens = sp;
            close(fd);
            break;
        }
        close(fd);
    }

    std::vector<V4l2DeviceNodes> found;
    for (int i = 0; i < 8; ++i) {
        std::string mp = "/dev/media" + std::to_string(i);
        if (access(mp.c_str(), R_OK | W_OK) != 0) continue;
        V4l2DeviceNodes n;
        if (inspectMediaDevice(mp, videoNodes, subdevNodes, &n)) {
            if (!n.isFront && n.lensSubdev.empty() && !fallbackLens.empty()) {
                n.lensSubdev = fallbackLens;
            }
            ALOGI("Discovered camera %d (%s): media=%s sensor=%s csis=%s video=%s lens=%s",
                  n.cameraId, n.sensorName.c_str(), n.mediaDev.c_str(),
                  n.sensorSubdev.c_str(), n.csisSubdev.c_str(), n.videoDev.c_str(),
                  n.lensSubdev.empty() ? "none" : n.lensSubdev.c_str());
            found.push_back(n);
        }
    }

    std::sort(found.begin(), found.end(),
              [](const V4l2DeviceNodes& a, const V4l2DeviceNodes& b) {
                  return a.cameraId < b.cameraId;
              });
    return found;
}

V4l2Pipeline::V4l2Pipeline(const V4l2DeviceNodes& nodes) : nodes_(nodes) {}

V4l2Pipeline::~V4l2Pipeline() {
    closeNodes();
}

bool V4l2Pipeline::openNodes() {
    if (videoFd_ >= 0) return true;

    mediaFd_ = open(nodes_.mediaDev.c_str(), O_RDWR | O_CLOEXEC);
    sensorFd_ = open(nodes_.sensorSubdev.c_str(), O_RDWR | O_CLOEXEC);
    csisFd_ = open(nodes_.csisSubdev.c_str(), O_RDWR | O_CLOEXEC);
    videoFd_ = open(nodes_.videoDev.c_str(), O_RDWR | O_CLOEXEC);

    if (mediaFd_ < 0 || sensorFd_ < 0 || csisFd_ < 0 || videoFd_ < 0) {
        ALOGE("openNodes failed cam%d (media=%d sensor=%d csis=%d video=%d): %s",
              nodes_.cameraId, mediaFd_, sensorFd_, csisFd_, videoFd_, strerror(errno));
        closeNodes();
        return false;
    }

    if (!nodes_.lensSubdev.empty()) {
        lensFd_ = open(nodes_.lensSubdev.c_str(), O_RDWR | O_CLOEXEC);
        if (lensFd_ < 0) {
            ALOGW("Optional lens subdev %s open failed: %s", nodes_.lensSubdev.c_str(), strerror(errno));
        }
    }

    enableMediaLinks();
    lastExpo_ = -1;
    lastAgain_ = -1;
    lastDgain_ = -1;
    return true;
}

void V4l2Pipeline::closeNodes() {
    if (!nodes_.isFront && oisActive_) {
        writeOisSysfs("0");
        oisActive_ = false;
    }
    stopStream();
    auto closeFd = [](int& fd) {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    };
    closeFd(lensFd_);
    closeFd(videoFd_);
    closeFd(csisFd_);
    closeFd(sensorFd_);
    closeFd(mediaFd_);
}

bool V4l2Pipeline::enableMediaLinks() {
    if (mediaFd_ < 0) return false;
    uint32_t eid = 0;
    while (true) {
        struct media_entity_desc ent{};
        ent.id = eid | MEDIA_ENT_ID_FLAG_NEXT;
        if (xioctl(mediaFd_, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0) break;
        eid = ent.id;
        if (ent.links == 0) continue;

        std::vector<struct media_link_desc> links(ent.links);
        std::vector<struct media_pad_desc> pads(ent.pads);
        struct media_links_enum le{};
        le.entity = ent.id;
        le.pads = ent.pads ? pads.data() : nullptr;
        le.links = links.data();
        if (xioctl(mediaFd_, MEDIA_IOC_ENUM_LINKS, &le) < 0) continue;

        for (auto link : links) {
            if (!(link.flags & MEDIA_LNK_FL_IMMUTABLE) && !(link.flags & MEDIA_LNK_FL_ENABLED)) {
                link.flags |= MEDIA_LNK_FL_ENABLED;
                if (xioctl(mediaFd_, MEDIA_IOC_SETUP_LINK, &link) < 0) {
                    ALOGW("MEDIA_IOC_SETUP_LINK failed: %s", strerror(errno));
                }
            }
        }
    }
    return true;
}

bool V4l2Pipeline::setSubdevPadFormat(int fd, uint32_t pad, int32_t width, int32_t height, uint32_t code) {
    struct v4l2_subdev_format sfmt{};
    sfmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sfmt.pad = pad;
    sfmt.format.width = static_cast<uint32_t>(width);
    sfmt.format.height = static_cast<uint32_t>(height);
    sfmt.format.code = code;
    sfmt.format.field = V4L2_FIELD_NONE;
    if (xioctl(fd, VIDIOC_SUBDEV_S_FMT, &sfmt) < 0) {
        ALOGE("VIDIOC_SUBDEV_S_FMT(fd=%d pad=%u %dx%d code=0x%x) failed: %s",
              fd, pad, width, height, code, strerror(errno));
        return false;
    }
    return true;
}

bool V4l2Pipeline::startStream(int32_t width, int32_t height, uint32_t reqBufCount) {
    if (videoFd_ < 0 && !openNodes()) return false;
    if (streaming_ && curW_ == width && curH_ == height) return true;

    stopStream();
    enableMediaLinks();

    if (!setSubdevPadFormat(sensorFd_, 0, width, height, nodes_.mbusCode)) return false;
    if (!setSubdevPadFormat(csisFd_, 0, width, height, nodes_.mbusCode)) return false;
    if (!setSubdevPadFormat(csisFd_, 1, width, height, nodes_.mbusCode)) return false;

    struct v4l2_format vfmt{};
    vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vfmt.fmt.pix.width = static_cast<uint32_t>(width);
    vfmt.fmt.pix.height = static_cast<uint32_t>(height);
    vfmt.fmt.pix.pixelformat = nodes_.v4l2PixFmt;
    vfmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(videoFd_, VIDIOC_S_FMT, &vfmt) < 0) {
        ALOGE("VIDIOC_S_FMT(%dx%d fourcc=0x%x) failed: %s", width, height, nodes_.v4l2PixFmt, strerror(errno));
        return false;
    }

    curW_ = static_cast<int32_t>(vfmt.fmt.pix.width);
    curH_ = static_cast<int32_t>(vfmt.fmt.pix.height);
    curStrideBytes_ = static_cast<int32_t>(vfmt.fmt.pix.bytesperline);
    if (curStrideBytes_ < curW_ * 2) curStrideBytes_ = curW_ * 2;

    struct v4l2_requestbuffers req{};
    req.count = reqBufCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(videoFd_, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) {
        ALOGE("VIDIOC_REQBUFS(count=%u %dx%d) failed: %s", reqBufCount, curW_, curH_, strerror(errno));
        return false;
    }

    buffers_.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(videoFd_, VIDIOC_QUERYBUF, &buf) < 0) {
            ALOGE("VIDIOC_QUERYBUF[%u] failed: %s", i, strerror(errno));
            stopStream();
            return false;
        }
        void* p = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, videoFd_, buf.m.offset);
        if (p == MAP_FAILED) {
            ALOGE("mmap[%u] len=%u failed: %s", i, buf.length, strerror(errno));
            stopStream();
            return false;
        }
        buffers_[i] = {p, buf.length};
    }

    for (uint32_t i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(videoFd_, VIDIOC_QBUF, &buf) < 0) {
            ALOGE("VIDIOC_QBUF[%u] failed: %s", i, strerror(errno));
            stopStream();
            return false;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(videoFd_, VIDIOC_STREAMON, &type) < 0) {
        ALOGE("VIDIOC_STREAMON failed: %s", strerror(errno));
        stopStream();
        return false;
    }

    streaming_ = true;
    lastExpo_ = -1;
    lastAgain_ = -1;
    lastDgain_ = -1;
    if (!nodes_.isFront && !oisActive_) {
        // Companion/sensor MCLK & rails are live once STREAMON succeeds
        oisActive_ = writeOisSysfs("1");
    }
    ALOGI("Started stream cam%d %dx%d stride=%d bufs=%u", nodes_.cameraId, curW_, curH_,
          curStrideBytes_, req.count);
    return true;
}

void V4l2Pipeline::stopStream() {
    if (videoFd_ < 0) return;

    if (streaming_) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(videoFd_, VIDIOC_STREAMOFF, &type) < 0) {
            ALOGW("VIDIOC_STREAMOFF failed: %s", strerror(errno));
        }
        streaming_ = false;
    }

    for (auto& b : buffers_) {
        if (b.ptr && b.ptr != MAP_FAILED) {
            munmap(b.ptr, b.length);
        }
    }
    buffers_.clear();

    struct v4l2_requestbuffers req{};
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    xioctl(videoFd_, VIDIOC_REQBUFS, &req);
}

bool V4l2Pipeline::queryCtrl(int fd, uint32_t cid, int64_t* minV, int64_t* maxV,
                             int64_t* defV, int64_t* curV) {
    if (fd < 0) return false;
    struct v4l2_query_ext_ctrl qec{};
    qec.id = cid;
    if (xioctl(fd, VIDIOC_QUERY_EXT_CTRL, &qec) < 0) return false;
    if (minV) *minV = qec.minimum;
    if (maxV) *maxV = qec.maximum;
    if (defV) *defV = qec.default_value;
    if (curV) {
        struct v4l2_ext_control ec{};
        ec.id = cid;
        struct v4l2_ext_controls ecs{};
        ecs.count = 1;
        ecs.controls = &ec;
        if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &ecs) == 0) {
            *curV = (qec.type == V4L2_CTRL_TYPE_INTEGER64) ? ec.value64 : ec.value;
        } else {
            *curV = qec.default_value;
        }
    }
    return true;
}

bool V4l2Pipeline::setCtrl(int fd, uint32_t cid, int32_t val) {
    if (fd < 0) return false;
    struct v4l2_control c{};
    c.id = cid;
    c.value = val;
    return xioctl(fd, VIDIOC_S_CTRL, &c) == 0;
}

SensorCtrlLimits V4l2Pipeline::querySensorLimits() {
    SensorCtrlLimits lim{};
    int64_t minV = 0, maxV = 0, defV = 0;
    if (queryCtrl(sensorFd_, V4L2_CID_EXPOSURE, &minV, &maxV, &defV, nullptr)) {
        lim.expoMin = static_cast<int32_t>(minV);
        lim.expoMax = static_cast<int32_t>(maxV);
        lim.expoDef = static_cast<int32_t>(defV);
    }
    if (queryCtrl(sensorFd_, V4L2_CID_ANALOGUE_GAIN, &minV, &maxV, &defV, nullptr)) {
        lim.againMin = static_cast<int32_t>(minV);
        lim.againMax = static_cast<int32_t>(maxV);
        lim.againDef = static_cast<int32_t>(defV);
    }
    if (queryCtrl(sensorFd_, V4L2_CID_DIGITAL_GAIN, &minV, &maxV, &defV, nullptr)) {
        lim.dgainMin = static_cast<int32_t>(minV);
        lim.dgainMax = static_cast<int32_t>(maxV);
        lim.dgainDef = static_cast<int32_t>(defV);
    }

    int64_t pixRate = 0, hblank = 0;
    if (queryCtrl(sensorFd_, V4L2_CID_PIXEL_RATE, nullptr, nullptr, nullptr, &pixRate) && pixRate > 0) {
        queryCtrl(sensorFd_, V4L2_CID_HBLANK, nullptr, nullptr, nullptr, &hblank);
        double linePixels = static_cast<double>(std::max(1, curW_) + std::max<int64_t>(0, hblank));
        lim.lineDurationNs = (linePixels * 1e9) / static_cast<double>(pixRate);
    }
    return lim;
}

void V4l2Pipeline::applyExposureGain(int32_t exposureLines, int32_t again, int32_t dgain) {
    if (sensorFd_ < 0) return;
    if (exposureLines != lastExpo_) {
        if (setCtrl(sensorFd_, V4L2_CID_EXPOSURE, exposureLines)) lastExpo_ = exposureLines;
    }
    if (again != lastAgain_) {
        if (setCtrl(sensorFd_, V4L2_CID_ANALOGUE_GAIN, again)) lastAgain_ = again;
    }
    if (dgain != lastDgain_) {
        if (setCtrl(sensorFd_, V4L2_CID_DIGITAL_GAIN, dgain)) lastDgain_ = dgain;
    }
}

bool V4l2Pipeline::setFocusPos(int32_t focusPos) {
    if (lensFd_ < 0) return false;
    if (!setCtrl(lensFd_, V4L2_CID_FOCUS_ABSOLUTE, focusPos)) {
        ALOGW("setFocusPos(%d) ioctl failed: %s", focusPos, strerror(errno));
        return false;
    }
    return true;
}

bool V4l2Pipeline::getFocusRange(int32_t* minVal, int32_t* maxVal) {
    if (lensFd_ < 0) return false;
    int64_t lo = 0, hi = 0;
    if (!queryCtrl(lensFd_, V4L2_CID_FOCUS_ABSOLUTE, &lo, &hi, nullptr, nullptr)) return false;
    if (minVal) *minVal = static_cast<int32_t>(lo);
    if (maxVal) *maxVal = static_cast<int32_t>(hi);
    return true;
}

bool V4l2Pipeline::dequeueFrame(RawFrameView* outFrame, int timeoutMs) {
    if (!streaming_ || videoFd_ < 0 || !outFrame) return false;

    struct pollfd pfd{};
    pfd.fd = videoFd_;
    pfd.events = POLLIN;
    int pr = poll(&pfd, 1, timeoutMs);
    if (pr <= 0) {
        ALOGE("poll(videoFd=%d) %s", videoFd_, pr == 0 ? "timed out" : strerror(errno));
        return false;
    }

    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(videoFd_, VIDIOC_DQBUF, &buf) < 0) {
        ALOGE("VIDIOC_DQBUF failed: %s", strerror(errno));
        return false;
    }
    if (buf.index >= buffers_.size()) return false;

    outFrame->data = static_cast<const uint16_t*>(buffers_[buf.index].ptr);
    outFrame->width = curW_;
    outFrame->height = curH_;
    outFrame->strideBytes = curStrideBytes_;
    outFrame->bufferIndex = buf.index;
    outFrame->timestampNs = static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000000LL +
                            static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;
    return true;
}

bool V4l2Pipeline::requeueFrame(uint32_t bufferIndex) {
    if (videoFd_ < 0 || bufferIndex >= buffers_.size()) return false;
    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = bufferIndex;
    if (xioctl(videoFd_, VIDIOC_QBUF, &buf) < 0) {
        ALOGE("requeue VIDIOC_QBUF[%u] failed: %s", bufferIndex, strerror(errno));
        return false;
    }
    return true;
}

}  // namespace s6cam
