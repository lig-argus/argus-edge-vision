// 지면 기하 역할. worker의 contact-first와 유효 roof fallback을 보존한다.
#pragma once
#include <nlohmann/json.hpp>

#include "argus/common/time/include/frame_pose_alignment.hpp"
#include "argus/tracking/ucmc/include/metric_box_observation.hpp"
namespace follow {
class FrameTransform {
public:
    FrameTransform(std::array<double, 4> intrinsics, nlohmann::json config);
    Eigen::Vector2d uv_to_ground(double u, double v, const AlignedFCPose&, double plane_down) const;
    Eigen::Vector2d ground_to_uv(const Eigen::Vector2d&, const AlignedFCPose&) const;
    MetricBoxObservation measure(const std::array<double, 5>&, const AlignedFCPose&) const;
    std::array<double, 5> stable_box(const std::array<double, 5>&, const AlignedFCPose&) const;
    const nlohmann::json& config() const { return config_; }

private:
    Eigen::Matrix3d rotation(const AlignedFCPose&) const;
    Eigen::Vector2d project(double u, double v, double plane, const AlignedFCPose&) const;
    std::array<double, 4> k_;
    nlohmann::json config_;
    Eigen::Matrix3d camera_to_body_;
    Eigen::Vector3d offset_;
    double anchor_x_, anchor_y_, reference_down_;
    bool contact_, roof_;
};
}  // namespace follow
