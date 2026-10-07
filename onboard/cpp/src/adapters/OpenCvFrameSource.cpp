#include "argus/adapters/OpenCvFrameSource.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <limits>
#include <poll.h>
#include <unistd.h>
namespace argus {
OpenCvFrameSource::~OpenCvFrameSource() = default;
OpenCvFrameSource::OpenCvFrameSource(const IClock &clock, const std::string &path, bool replay, unsigned w,
                                     unsigned h, unsigned fps, const std::string &fourcc)
    : clock_(clock), replay_(replay), name_((replay ? "replay:" : "ir_v4l2:") + path) {
    if (!w || !h || !fps || fourcc.size() != 4)
        throw std::invalid_argument("invalid camera format");
    bool index =
        !path.empty() && std::all_of(path.begin(), path.end(), [](char c) { return c >= '0' && c <= '9'; });
    if (replay && !std::filesystem::is_regular_file(path))
        throw std::invalid_argument("replay requires a video file");
    if (!replay && !index && path.rfind("/dev/", 0) != 0)
        throw std::invalid_argument("V4L2 requires a device; select replay for files");
    auto helper = std::filesystem::canonical("/proc/self/exe").parent_path() / "argus-frame-source";
    process_.start({helper.string(), replay ? "replay" : "ir_v4l2", path, std::to_string(w),
                    std::to_string(h), std::to_string(fps), fourcc},
                   true);
}
bool OpenCvFrameSource::read_exact(std::uint8_t *dst, std::size_t bytes) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    std::size_t offset{};
    while (offset < bytes) {
        if (stopping_)
            return false;
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("OpenCV capture response timed out");
        pollfd item{process_.output_fd(), POLLIN, 0};
        auto rc = poll(&item, 1, 100);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            throw std::runtime_error("OpenCV source poll failed");
        }
        if (!rc)
            continue;
        auto n = ::read(item.fd, dst + offset, bytes - offset);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            throw std::runtime_error("OpenCV source read failed");
        }
        if (!n)
            throw std::runtime_error("OpenCV capture exited unexpectedly/truncated frame");
        offset += n;
    }
    return true;
}
FrameRead OpenCvFrameSource::read(Frame &frame) {
    std::uint8_t header[40]{};
    if (!read_exact(header, 40))
        return FrameRead::EndOfStream;
    std::string magic(reinterpret_cast<char *>(header), 4);
    if (magic == "END1" && replay_)
        return FrameRead::EndOfStream;
    auto le = [&](unsigned start, unsigned n) {
        std::uint64_t v{};
        for (unsigned i = 0; i < n; ++i)
            v |= std::uint64_t(header[start + i]) << (8 * i);
        return v;
    };
    auto w = le(8, 4), h = le(12, 4), id = le(24, 8), stamp = le(32, 8);
    if (magic != "CVA1" || le(4, 2) != 1 || le(6, 2) != 40 || !w || !h || w > 8192 || h > 8192 ||
        le(16, 2) != 24 || le(18, 2) != 0 || le(20, 4) != w * h * 3 || w * h * 3 > 32 * 1024 * 1024 ||
        id != sequence_++)
        throw std::runtime_error("invalid OpenCV source header");
    auto bytes = std::make_shared<std::vector<std::uint8_t>>(w * h * 3);
    if (!read_exact(bytes->data(), bytes->size()))
        return FrameRead::EndOfStream;
    auto now = clock_.now();
    frame = {id,
             unsigned(w),
             unsigned(h),
             unsigned(w * 3),
             "BGR8",
             bytes,
             {now.unix_ns, now.monotonic_ns, std::nullopt, stamp, "host_monotonic"}};
    frame.validate();
    return FrameRead::Ready;
}
} // namespace argus
