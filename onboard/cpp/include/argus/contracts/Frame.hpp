#pragma once
#include "argus/ports/IClock.hpp"
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
namespace argus {
struct FrameTiming {
    std::uint64_t received_unix_ns{}, received_monotonic_ns{};
    std::optional<std::uint64_t> exposure_monotonic_ns, source_receive_monotonic_ns;
    std::string clock_domain{"host_monotonic"};
    void validate() const {
        if (clock_domain != "host_monotonic")
            throw std::invalid_argument("unmapped frame clock");
        if (exposure_monotonic_ns && *exposure_monotonic_ns > received_monotonic_ns)
            throw std::invalid_argument("exposure after receipt");
    }
    std::uint64_t require_exposure_time() const {
        validate();
        if (!exposure_monotonic_ns)
            throw std::invalid_argument("sensor exposure time unavailable");
        return *exposure_monotonic_ns;
    }
};
struct Frame {
    std::uint64_t frame_id{};
    unsigned width{}, height{}, stride{};
    std::string pixel_format; // BGR8 or RAW14_LE16; conversion belongs to Python Preprocessor.
    std::shared_ptr<const std::vector<std::uint8_t>> pixels;
    FrameTiming timing;
    void validate() const {
        timing.validate();
        auto channels = pixel_format == "BGR8" ? 3U : pixel_format == "RAW14_LE16" ? 2U : 0U;
        if (!channels || !width || !height || width > 8192 || height > 8192 || stride != width * channels ||
            !pixels || pixels->size() != std::size_t(stride) * height)
            throw std::invalid_argument("invalid frame format or payload");
    }
};
} // namespace argus
