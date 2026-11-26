#pragma once
#include <array>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/common/include/i_clock.hpp"
namespace argus {
struct FrameTiming {
    std::uint64_t received_unix_ns{}, received_monotonic_ns{};
    // SDK 2.12.2 device_ms is camera uptime, not a verified exposure timestamp.
    // Do not populate exposure_monotonic_ns without event and host-clock mapping validation.
    std::optional<std::uint64_t> exposure_monotonic_ns, source_receive_monotonic_ns;
    std::string clock_domain{"host_monotonic"};
    // serial, camera boot milliseconds, device frame counter, SDK host receipt ns.
    // Camera uptime is not certified exposure time.
    std::optional<std::array<std::uint64_t, 4>> camera_clock;
    void validate() const {
        if (clock_domain != "host_monotonic") {
            throw std::invalid_argument("unmapped frame clock");
        }
        if (exposure_monotonic_ns && *exposure_monotonic_ns > received_monotonic_ns) {
            throw std::invalid_argument("exposure after receipt");
        }
        if (camera_clock && (camera_clock->at(0) > UINT32_MAX || camera_clock->at(1) > UINT32_MAX ||
                             camera_clock->at(2) > UINT32_MAX || camera_clock->at(3) == 0 ||
                             camera_clock->at(3) > received_monotonic_ns)) {
            throw std::invalid_argument("invalid SDK frame clock metadata");
        }
    }
    std::uint64_t require_exposure_time() const {
        validate();
        if (!exposure_monotonic_ns) {
            throw std::invalid_argument("sensor exposure time unavailable");
        }
        return *exposure_monotonic_ns;
    }
};
struct Frame {
    std::uint64_t frame_id{};
    unsigned width{}, height{}, stride{};
    std::string pixel_format;  // BGR8 or RAW14_LE16; conversion belongs to Python Preprocessor.
    std::shared_ptr<const std::vector<std::uint8_t>> pixels;
    FrameTiming timing;
    void validate() const {
        timing.validate();
        unsigned channels = 0;
        if (pixel_format == "BGR8") {
            channels = 3;
        } else if (pixel_format == "RAW14_LE16") {
            channels = 2;
        }
        if (channels == 0 || width == 0 || height == 0 || width > 8192 || height > 8192 ||
            stride != width * channels || !pixels ||
            pixels->size() != static_cast<std::size_t>(stride) * height) {
            throw std::invalid_argument("invalid frame format or payload");
        }
    }
};
}  // namespace argus
