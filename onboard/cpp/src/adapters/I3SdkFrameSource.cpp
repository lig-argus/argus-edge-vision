#include "argus/adapters/I3SdkFrameSource.hpp"
#include <cerrno>
#include <chrono>
#include <limits>
#include <poll.h>
#include <unistd.h>
namespace argus {
I3SdkFrameSource::I3SdkFrameSource(const IClock &clock, const std::string &executable, unsigned device,
                                   unsigned timeout)
    : clock_(clock), timeout_s_(timeout),
      name_("ir_sdk:" + executable + ":device=" + std::to_string(device)) {
    if (device > 31 || timeout < 1 || timeout > 300 || access(executable.c_str(), X_OK))
        throw std::invalid_argument("SDK executable/device/timeout unavailable");
    process_.start(
        {executable, "--device", std::to_string(device), "--timeout", std::to_string(timeout), "--fps", "0"},
        true);
}
bool I3SdkFrameSource::read_exact(std::uint8_t *dst, std::size_t bytes, bool header) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s_);
    std::size_t offset{};
    while (offset < bytes) {
        if (stopping_)
            return false;
        if (std::chrono::steady_clock::now() >= deadline)
            throw std::runtime_error("SDK frame receive timed out");
        pollfd item{process_.output_fd(), POLLIN, 0};
        auto rc = poll(&item, 1, 100);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            throw std::runtime_error("SDK poll failed");
        }
        if (!rc)
            continue;
        auto count = ::read(item.fd, dst + offset, bytes - offset);
        if (count < 0) {
            if (errno == EINTR)
                continue;
            throw std::runtime_error("SDK pipe read failed");
        }
        if (!count) {
            if (header && offset == 0)
                throw std::runtime_error("SDK stream ended unexpectedly");
            throw std::runtime_error("truncated SDK frame");
        }
        offset += count;
    }
    return true;
}
FrameRead I3SdkFrameSource::read(Frame &frame) {
    std::uint8_t header[40]{};
    if (!read_exact(header, sizeof(header), true))
        return FrameRead::EndOfStream;
    auto le = [&](unsigned offset, unsigned n) {
        std::uint64_t value{};
        for (unsigned i = 0; i < n; ++i)
            value |= std::uint64_t(header[offset + i]) << (8 * i);
        return value;
    };
    if (std::string(reinterpret_cast<char *>(header), 4) != "I3R4" || le(4, 2) != 1 || le(6, 2) != 40 ||
        le(8, 4) != 640 || le(12, 4) != 480 || le(16, 2) != 14 || le(18, 2) > 1 ||
        le(20, 4) != 640 * 480 * 2 || le(24, 8) <= last_sequence_ ||
        le(32, 8) > std::numeric_limits<std::uint64_t>::max() / 1000)
        throw std::runtime_error("invalid RAW14 header/sequence");
    auto payload = std::make_shared<std::vector<std::uint8_t>>(640 * 480 * 2);
    if (!read_exact(payload->data(), payload->size(), false))
        return FrameRead::EndOfStream;
    for (std::size_t i = 0; i < payload->size(); i += 2)
        if ((unsigned((*payload)[i]) | unsigned((*payload)[i + 1]) << 8) > 16383)
            throw std::runtime_error("RAW14 value exceeds range; masking is forbidden");
    auto stamp = clock_.now();
    last_sequence_ = le(24, 8);
    frame = {last_sequence_,
             640,
             480,
             1280,
             "RAW14_LE16",
             payload,
             {stamp.unix_ns, stamp.monotonic_ns, std::nullopt, le(32, 8) * 1000, "host_monotonic"}};
    frame.validate();
    return FrameRead::Ready;
}
} // namespace argus
