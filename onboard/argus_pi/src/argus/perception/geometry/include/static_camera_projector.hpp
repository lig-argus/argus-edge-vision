#pragma once

#include "argus/perception/geometry/include/pi_ground_projector.hpp"

namespace follow {
// Original UCMC fixed-camera ground-plane mapping. Association coordinates only;
// never exported as FC/NED ground observations or an aligned vehicle pose.
class StaticCameraProjector final {
public:
    StaticCameraProjector(const std::string& camera_file, std::uint32_t width, std::uint32_t height,
                          NativeJson geometry = NativeJson::object());
    PiProjectedFrame project(const NativeJson& pixels, std::int64_t now_ns) const;

private:
    Eigen::Matrix3d inverse_;
    std::array<double, 4> k_{};
    std::array<double, 4> distortion_{};
    std::uint32_t width_, height_;
};
}  // namespace follow
