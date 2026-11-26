#include "argus/perception/geometry/include/static_camera_projector.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "argus/perception/geometry/include/pinhole_distortion.hpp"

namespace follow {
StaticCameraProjector::StaticCameraProjector(const std::string& file, std::uint32_t width,
                                             std::uint32_t height, NativeJson geometry)
    : width_(width), height_(height) {
    std::ifstream input(file);
    Eigen::Matrix3d rotation, intrinsic;
    Eigen::Vector3d translation;
    std::string section;
    if (!(input >> section) || section != "RotationMatrices") {
        throw std::invalid_argument("camera-only requires a UCMC R/T/K calibration file");
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            input >> rotation(r, c);
        }
    }
    input >> section;
    if (section != "TranslationVectors") {
        throw std::invalid_argument("missing UCMC translation");
    }
    input >> translation[0] >> translation[1] >> translation[2] >> section;
    if (section != "IntrinsicMatrix") {
        throw std::invalid_argument("missing UCMC intrinsics");
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            input >> intrinsic(r, c);
        }
    }
    if (!input || (input >> section) || width == 0 || height == 0 || !rotation.allFinite() ||
        !translation.allFinite() || !intrinsic.allFinite() ||
        !(rotation * rotation.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-4) ||
        std::abs(rotation.determinant() - 1.) > 1e-4 || intrinsic(0, 0) <= 0 ||
        intrinsic(1, 1) <= 0 || !intrinsic.row(2).isApprox(Eigen::RowVector3d(0, 0, 1), 1e-9)) {
        throw std::invalid_argument("invalid UCMC static calibration");
    }
    if (!geometry.empty()) {
        intrinsic << geometry.at("fx").get<double>(), 0., geometry.at("cx").get<double>(), 0.,
            geometry.at("fy").get<double>(), geometry.at("cy").get<double>(), 0., 0., 1.;
        distortion_ = read_pixel_distortion(geometry);
        if (!intrinsic.allFinite() || intrinsic(0, 0) <= 0 || intrinsic(1, 1) <= 0) {
            throw std::invalid_argument("invalid RGB intrinsics");
        }
    }
    k_ = {intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2)};
    // The existing cam_para format stores translation in millimetres.
    Eigen::Matrix3d plane;
    plane.col(0) = rotation.col(0);
    plane.col(1) = rotation.col(1);
    plane.col(2) = translation / 1000.;
    const Eigen::Matrix3d homography = intrinsic * plane;
    if (!homography.fullPivLu().isInvertible()) {
        throw std::invalid_argument("singular UCMC ground homography");
    }
    inverse_ = homography.inverse();
}

PiProjectedFrame StaticCameraProjector::project(const NativeJson& pixels,
                                                std::int64_t now_ns) const {
    if (pixels.value("schema", "") != "argus.perception.observations.v2" ||
        pixels.value("clock_domain", "") != "host_monotonic" ||
        pixels.value("coordinate_frame", "") != "camera_pixel") {
        throw std::runtime_error("camera-only requires original pixel v2/host clock");
    }
    const auto stamp = pixels.at("captured_monotonic_ns").get<std::int64_t>();
    if (stamp <= 0 || now_ns < stamp || now_ns - stamp >= 250000000 ||
        pixels.at("image_width") != width_ || pixels.at("image_height") != height_) {
        throw std::runtime_error("camera-only pixel expired or calibration size mismatch");
    }
    const auto& detections = pixels.at("detections");
    if (!detections.is_array() || detections.size() > 512) {
        throw std::runtime_error("invalid detection list");
    }
    PiProjectedFrame frame{};
    frame.sample_ns = stamp;
    frame.pose.epoch = 0;  // Internal context only: never published as an FC epoch.
    for (const auto& detection : detections) {
        std::array<double, 5> box{detection.at("x1"), detection.at("y1"), detection.at("x2"),
                                  detection.at("y2"), detection.at("confidence")};
        if (std::any_of(box.begin(), box.end(), [](double v) { return !std::isfinite(v); }) ||
            box[4] <= 0 || box[4] > 1 || box[0] < 0 || box[1] < 0 || box[2] <= box[0] ||
            box[3] <= box[1] || box[2] > width_ || box[3] > height_) {
            throw std::runtime_error("invalid original detection bounds or score");
        }
        const auto class_id = detection.at("class_id").get<std::int32_t>();
        const auto label = detection.at("label").get<std::string>();
        if (class_id < 0 || label.empty() || label.size() > 128) {
            throw std::runtime_error("invalid class/label");
        }
        const auto ideal_pixel = [&](double u, double v) {
            const auto q =
                undistort_normalized((u - k_[2]) / k_[0], (v - k_[3]) / k_[1], distortion_);
            return Eigen::Vector3d(k_[2] + k_[0] * q[0], k_[3] + k_[1] * q[1], 1.);
        };
        const double u = (box[0] + box[2]) / 2., v = box[3];
        const Eigen::Vector3d mapped = inverse_ * ideal_pixel(u, v);
        if (!mapped.allFinite() || mapped[2] <= 1e-9) {
            continue;  // Behind camera / horizon: no invented replacement point.
        }
        const double gamma = 1. / mapped[2];
        const Eigen::Matrix2d jacobian =
            gamma * inverse_.topLeftCorner<2, 2>() -
            gamma * gamma * mapped.head<2>() * inverse_.bottomLeftCorner<1, 2>();
        Eigen::Matrix2d pixel_jacobian;
        pixel_jacobian.col(0) =
            (ideal_pixel(u + .001, v) - ideal_pixel(u - .001, v)).head<2>() / .002;
        pixel_jacobian.col(1) =
            (ideal_pixel(u, v + .001) - ideal_pixel(u, v - .001)).head<2>() / .002;
        const Eigen::Matrix2d raw_jacobian = jacobian * pixel_jacobian;
        const double sx = std::clamp(.05 * (box[3] - box[1]), 2., 13.);
        const double sy = std::clamp(.05 * (box[3] - box[1]), 2., 10.);
        Eigen::Matrix2d noise = Eigen::Vector2d(sx * sx, sy * sy).asDiagonal();
        MetricBoxObservation observation;
        observation.box = box;
        observation.point = mapped.head<2>() * gamma;
        observation.covariance = raw_jacobian * noise * raw_jacobian.transpose();
        if (!observation.covariance.allFinite() || observation.covariance.determinant() <= 0) {
            continue;
        }
        frame.detections.push_back({class_id, label, observation});
        frame.detections.back().thermal = detection.value("thermal", NativeJson{});
    }
    return frame;
}
}  // namespace follow
