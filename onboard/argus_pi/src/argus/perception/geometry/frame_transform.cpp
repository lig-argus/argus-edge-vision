#include "argus/perception/geometry/include/frame_transform.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "argus/perception/geometry/include/pinhole_distortion.hpp"
namespace follow {
FrameTransform::FrameTransform(std::array<double, 4> k, nlohmann::json config)
    : k_(k), config_(std::move(config)) {
    const double q = std::sqrt(.5);
    std::array<double, 9> matrix{0, -q, q, 1, 0, 0, 0, q, q};
    if (config_.contains("camera_to_body")) {
        matrix = config_["camera_to_body"].get<std::array<double, 9>>();
    }
    for (int i = 0; i < 9; ++i) {
        camera_to_body_(i / 3, i % 3) = matrix[i];
    }
    auto p = config_.value("camera_offset_body_m", std::array<double, 3>{.12, -.03, -.242});
    offset_ = Eigen::Vector3d(p[0], p[1], p[2]);
    if (!camera_to_body_.allFinite() || !offset_.allFinite() ||
        !(camera_to_body_.transpose() * camera_to_body_)
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6)) {
        throw std::invalid_argument("invalid camera mount calibration");
    }
    anchor_x_ = config_.value("box_anchor_x", .5);
    anchor_y_ = config_.value("box_anchor_y", 1.);
    double h = config_.value("target_anchor_height_above_ground_m", 0.);
    if (!std::isfinite(anchor_x_) || !std::isfinite(anchor_y_) || !std::isfinite(h) ||
        anchor_x_ < 0 || anchor_x_ > 1 || anchor_y_ < 0 || anchor_y_ > 1 || h < 0 || h > 3) {
        throw std::invalid_argument("invalid vehicle reference anchor/height");
    }
    for (double value : k_) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("invalid camera intrinsics");
        }
    }
    if (k_[0] <= 0 || k_[1] <= 0) {
        throw std::invalid_argument("invalid camera focal length");
    }
    reference_down_ = config_.value("ground_plane_down_m", 0.) - h;
    contact_ = config_.value("ground_contact_anchor", false);
    roof_ = config_.value("ground_roof_anchor", false);
    double radius = config_.value("ground_center_radius_m", 0.);
    if ((contact_ || roof_) && (!std::isfinite(radius) || radius <= 0 || radius > 2)) {
        throw std::invalid_argument("invalid reference radius");
    }
    if (contact_ && (anchor_y_ != 1 || h != 0)) {
        throw std::invalid_argument("invalid ground contact model");
    }
    if (roof_ && (anchor_y_ != 0 || h <= 0)) {
        throw std::invalid_argument("invalid roof reference model");
    }
}
Eigen::Matrix3d FrameTransform::rotation(const AlignedFCPose& p) const {
    return (Eigen::AngleAxisd(p.rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p.rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(p.rpy[0], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}
Eigen::Vector2d FrameTransform::uv_to_ground(double u, double v, const AlignedFCPose& p,
                                             double plane) const {
    auto r = rotation(p);
    const auto d = config_.value("distortion", Distortion4{});
    const auto uv = undistort_normalized((u - k_[2]) / k_[0], (v - k_[3]) / k_[1], d);
    Eigen::Vector3d ray = r * camera_to_body_ * Eigen::Vector3d(uv[0], uv[1], 1.);
    Eigen::Vector3d origin = p.position + r * offset_;
    if (ray[2] < config_.at("minimum_down_ray").get<double>()) {
        throw std::runtime_error("ground ray near horizon or above ground");
    }
    double distance = (plane - origin[2]) / ray[2];
    Eigen::Vector3d point = origin + distance * ray;
    if (distance <= 0 || !point.allFinite() ||
        (point - origin).head<2>().norm() > config_.at("max_ground_range_m").get<double>()) {
        throw std::runtime_error("ground intersection invalid/out of range");
    }
    return point.head<2>();
}
Eigen::Vector2d FrameTransform::ground_to_uv(const Eigen::Vector2d& point,
                                             const AlignedFCPose& p) const {
    auto r = rotation(p);
    Eigen::Vector3d optical =
        camera_to_body_.transpose() * r.transpose() *
        (Eigen::Vector3d(point[0], point[1], config_.value("ground_plane_down_m", 0.)) -
         p.position - r * offset_);
    if (optical[2] <= 0) {
        throw std::runtime_error("point behind camera");
    }
    const auto uv = distort_normalized(optical[0] / optical[2], optical[1] / optical[2],
                                       config_.value("distortion", Distortion4{}));
    return {k_[2] + k_[0] * uv[0], k_[3] + k_[1] * uv[1]};
}
Eigen::Vector2d FrameTransform::project(double u, double v, double plane,
                                        const AlignedFCPose& p) const {
    Eigen::Vector2d point = uv_to_ground(u, v, p, plane);
    if (contact_ || roof_) {
        Eigen::Vector2d delta = point - p.position.head<2>();
        double norm = delta.norm();
        if (norm <= 1e-6) {
            throw std::runtime_error("ground contact direction invalid");
        }
        point += (contact_ ? 1. : -1.) * config_.at("ground_center_radius_m").get<double>() *
                 delta / norm;
    }
    return point;
}
MetricBoxObservation FrameTransform::measure(const std::array<double, 5>& b,
                                             const AlignedFCPose& p) const {
    const auto [x1, y1, x2, y2, score] = b;
    double width = config_.value("image_width", 640.), height = config_.value("image_height", 480.);
    if (contact_ && (x1 < 2 || x2 > width - 2 || y2 > height - 2 || y2 < 2 || y2 - y1 < 8)) {
        throw std::runtime_error("ground contact not fully visible");
    }
    if (roof_ && (x1 < 2 || x2 > width - 2 || y1 < 2 || y1 > height - 8 || y2 - y1 < 8)) {
        throw std::runtime_error("roof reference not fully visible");
    }
    double u = x1 + anchor_x_ * (x2 - x1), v = y1 + anchor_y_ * (y2 - y1);
    MetricBoxObservation out;
    out.box = b;
    out.anchor = {anchor_y_, config_.value("target_anchor_height_above_ground_m", 0.)};
    out.point = project(u, v, reference_down_, p);
    Eigen::Matrix2d jac;
    jac.col(0) = project(u + .5, v, reference_down_, p) - project(u - .5, v, reference_down_, p);
    jac.col(1) = project(u, v + .5, reference_down_, p) - project(u, v - .5, reference_down_, p);
    Eigen::Matrix2d pixels = Eigen::Matrix2d::Zero();
    double sx = std::clamp(.05 * (y2 - y1), 2., 13.), sy = std::clamp(.05 * (y2 - y1), 2., 10.);
    pixels(0, 0) = sx * sx;
    pixels(1, 1) = sy * sy;
    Eigen::Vector2d plane =
        (project(u, v, reference_down_ + .01, p) - project(u, v, reference_down_ - .01, p)) / .02;
    out.covariance =
        jac * pixels * jac.transpose() +
        Eigen::Matrix2d::Identity() * std::pow(config_.at("ground_pose_std_m").get<double>(), 2) +
        plane * plane.transpose() * std::pow(config_.at("contact_height_std_m").get<double>(), 2);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(out.covariance);
    if (!out.covariance.allFinite() || eigen.info() != Eigen::Success ||
        eigen.eigenvalues().maxCoeff() >
            std::pow(config_.at("max_ground_std_m").get<double>(), 2)) {
        throw std::runtime_error("ground covariance gate");
    }
    return out;
}
std::array<double, 5> FrameTransform::stable_box(const std::array<double, 5>& b,
                                                 const AlignedFCPose& p) const {
    Eigen::Vector2d low = Eigen::Vector2d::Constant(INFINITY),
                    high = Eigen::Vector2d::Constant(-INFINITY);
    for (auto uv : std::array<std::array<double, 2>, 4>{
             {{b[0], b[1]}, {b[2], b[1]}, {b[0], b[3]}, {b[2], b[3]}}}) {
        Eigen::Vector2d point =
            Eigen::Vector2d::Constant(1000.) +
            20. * uv_to_ground(uv[0], uv[1], p, config_.value("ground_plane_down_m", 0.));
        low = low.cwiseMin(point);
        high = high.cwiseMax(point);
    }
    return {low[0], low[1], high[0], high[1], b[4]};
}
}  // namespace follow
