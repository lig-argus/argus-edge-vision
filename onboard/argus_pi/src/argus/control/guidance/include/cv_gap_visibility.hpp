#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>

#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/control/target/include/target_memory.hpp"

namespace follow {
// A finite, approximate forecast of an old observed box; not a new detection.
struct CvVisibilityChoice {
    double forward_m_s;
    double right_m_s;
    double scale;
    double score;
    bool lateral{false};
    double lateral_delta_m_s{0};
};

// Diagnostic values are populated only when explicitly requested. They are
// projections of the existing candidate evaluation, not observations.
struct CvVisibilityStepDiagnostic {
    double t_s{0}, left{0}, right{0}, top{0}, bottom{0};
    double roll_deg{0}, pitch_deg{0};
};
struct CvVisibilityCandidateDiagnostic {
    double scale{0}, forward_m_s{0}, right_m_s{0}, score{0};
    bool valid{false}, lateral{false};
    double lateral_delta_m_s{0}, worst_edge{0}, max_range_error_m{0};
    std::array<CvVisibilityStepDiagnostic, 12> steps{};
    std::size_t step_count{0};
};
struct CvVisibilityDiagnostics {
    double shape_age_s{0}, horizon_s{0};
    bool tilt_model_requested{false}, tilt_model_used{false};
    std::array<CvVisibilityCandidateDiagnostic, 5> candidates{};
    std::size_t candidate_count{0};
    int selected_index{-1};
};

inline std::optional<CvVisibilityChoice> choose_cv_visibility(
    const CameraGroundCalibration& camera, double ground_down,
    const CvVisibilityReference& reference, const SelectedTargetKey& selected, const Pose& current,
    std::array<double, 2> target_velocity, std::int64_t cv_origin_ns, Clock::time_point now,
    Clock::time_point deadline, double forward, double right, double previous_forward,
    double previous_right, std::optional<double> max_change,
    CvVisibilityDiagnostics* diagnostic = nullptr, double max_horizontal_speed = 0.,
    double desired_range_m = 0., double max_horizontal_accel = 0., double braking_accel = 0.,
    double velocity_response = 0.) {
    if (diagnostic) {
        *diagnostic = {};
    }
    const auto stamp = Clock::time_point{std::chrono::nanoseconds{reference.sample_ns}};
    if (cv_origin_ns <= 0 ||
        cv_origin_ns > std::numeric_limits<std::int64_t>::max() - 3000000000LL ||
        now < Clock::time_point{std::chrono::nanoseconds{cv_origin_ns}} ||
        deadline != Clock::time_point{std::chrono::nanoseconds{cv_origin_ns + 3000000000LL}} ||
        reference.binding.generation != selected.generation ||
        reference.binding.track_id != selected.track_id || reference.sample_ns <= 0 ||
        reference.sample_ns > cv_origin_ns || cv_origin_ns - reference.sample_ns > 3000000000LL ||
        now < stamp || now - stamp > std::chrono::seconds{6} || deadline <= now ||
        !reference.box.ground || !current.have_velocity || !current.have_attitude ||
        !current.have_position || !camera.image_width || !camera.image_height ||
        reference.image_width != camera.image_width ||
        reference.image_height != camera.image_height ||
        !std::isfinite(ground_down + forward + right + previous_forward + previous_right +
                       target_velocity[0] + target_velocity[1] + current.velocity_north_m_s +
                       current.velocity_east_m_s + current.velocity_down_m_s)) {
        return {};
    }
    if (max_change && (!std::isfinite(*max_change) || *max_change < 0)) {
        return {};
    }
    const auto& box = reference.box;
    if (!std::isfinite(box.x1 + box.x2 + box.y1 + box.y2) || box.x1 < 0 || box.y1 < 0 ||
        box.x2 > camera.image_width || box.y2 > camera.image_height || box.x2 <= box.x1 ||
        box.y2 <= box.y1) {
        return {};
    }
    const auto& op = reference.observation_pose;
    for (const double x : op) {
        if (!std::isfinite(x)) {
            return {};
        }
    }
    Pose capture;
    capture.north_m = op[0];
    capture.east_m = op[1];
    capture.down_m = op[2];
    capture.roll_deg = op[3];
    capture.pitch_deg = op[4];
    capture.yaw_deg = op[5];
    const double anchor_y = box.ground_anchor ? (*box.ground_anchor)[0] : camera.box_anchor_y;
    const double anchor_height =
        box.ground_anchor ? (*box.ground_anchor)[1] : camera.target_anchor_height_above_ground_m;
    if (!std::isfinite(anchor_y + anchor_height) || anchor_y < 0 || anchor_y > 1) {
        return {};
    }
    const std::array<double, 3> original_target{(*box.ground)[0], (*box.ground)[1],
                                                ground_down - anchor_height};
    std::optional<CalibratedGroundGeometry> geometry;
    try {
        geometry.emplace(camera);
    } catch (const std::invalid_argument&) {
        return {};
    }
    const auto measured = geometry->image_projection_with_depth(capture, original_target);
    if (!measured || (*measured)[2] <= .01) {
        return {};
    }
    const double bbox_x = (box.x1 + box.x2) / camera.image_width - 1.;
    const double bbox_anchor_y =
        2. * (box.y1 + anchor_y * (box.y2 - box.y1)) / camera.image_height - 1.;
    const double bias_x = bbox_x - (*measured)[0];
    const double bias_y = bbox_anchor_y - (*measured)[1];
    const double half_width = (box.x2 - box.x1) / camera.image_width;
    const double height = 2. * (box.y2 - box.y1) / camera.image_height;
    const double age = std::chrono::duration<double>(now - stamp).count();
    const double horizon = std::min(1.2, std::chrono::duration<double>(deadline - now).count());
    if (!(horizon > 0) || !std::isfinite(age + horizon + bias_x + bias_y)) {
        return {};
    }
    if (diagnostic) {
        diagnostic->shape_age_s = age;
        diagnostic->horizon_s = horizon;
    }
    const double yaw = current.yaw_deg * std::acos(-1.) / 180.;
    if (!std::isfinite(yaw)) {
        return {};
    }
    const auto wall = [](double x) {
        const double d = std::max(0., std::abs(x) - .75);
        return d * d;
    };
    const bool tilt_requested = v21_enabled("ARGUS_V23_CV_TILT_MODEL");
    const bool tilt_enabled = tilt_requested && std::isfinite(max_horizontal_accel) &&
                              max_horizontal_accel > 0 && std::isfinite(braking_accel) &&
                              braking_accel >= max_horizontal_accel &&
                              std::isfinite(velocity_response) && velocity_response > 0 &&
                              std::isfinite(current.roll_deg) && std::isfinite(current.pitch_deg);
    if (diagnostic) {
        diagnostic->tilt_model_requested = tilt_requested;
        diagnostic->tilt_model_used = tilt_enabled;
    }
    const bool lateral_enabled =
        v21_enabled("ARGUS_V23_CV_LATERAL") && max_change && std::isfinite(max_horizontal_speed) &&
        max_horizontal_speed > 0 && std::isfinite(desired_range_m) && desired_range_m > 0 &&
        std::hypot(previous_forward, previous_right) <= max_horizontal_speed + 1e-9;
    struct Metrics {
        bool valid{true};
        double score{0}, worst_edge{0}, max_range_error_m{0};
    };
    auto evaluate = [&](double f, double rightward, double penalty,
                        CvVisibilityCandidateDiagnostic* trace) {
        const double command_n = std::cos(yaw) * f - std::sin(yaw) * rightward;
        const double command_e = std::sin(yaw) * f + std::cos(yaw) * rightward;
        Metrics result;
        result.score = penalty;
        double predicted_roll = current.roll_deg, predicted_pitch = current.pitch_deg;
        // The existing analytic CV position/velocity and yaw remain unchanged.
        // Optional roll/pitch are a model assumption, never an FC observation.
        for (int i = 1; i <= 12; ++i) {
            const double t = horizon * i / 12.;
            const double response = (1. - std::exp(-1.8 * t)) / 1.8;
            Pose p = current;
            p.north_m += command_n * t + (current.velocity_north_m_s - command_n) * response;
            p.east_m += command_e * t + (current.velocity_east_m_s - command_e) * response;
            p.down_m += current.velocity_down_m_s * response;
            if (tilt_enabled) {
                const double dt = horizon / 12., elapsed = t - dt;
                const double decay = std::exp(-1.8 * elapsed);
                const double vn = command_n + (current.velocity_north_m_s - command_n) * decay;
                const double ve = command_e + (current.velocity_east_m_s - command_e) * decay;
                double an = velocity_response * (command_n - vn),
                       ae = velocity_response * (command_e - ve);
                const double speed = std::hypot(vn, ve),
                             target_speed = std::hypot(target_velocity[0], target_velocity[1]);
                const bool reversing =
                    speed > .1 && target_speed > .15 &&
                    vn * target_velocity[0] + ve * target_velocity[1] < -.5 * speed * target_speed;
                const double cy = std::cos(yaw), sy = std::sin(yaw);
                if (reversing) {
                    const double n = vn / speed, e = ve / speed;
                    double along = an * n + ae * e, across = -an * e + ae * n;
                    const double budget = along < 0 ? braking_accel : max_horizontal_accel;
                    const double ratio = std::hypot(along / budget, across / max_horizontal_accel);
                    if (ratio > 1.) {
                        along /= ratio;
                        across /= ratio;
                    }
                    an = along * n - across * e;
                    ae = along * e + across * n;
                } else {
                    double af = cy * an + sy * ae, ar = -sy * an + cy * ae;
                    const double vf = cy * vn + sy * ve;
                    const double budget =
                        std::abs(vf) > .1 && af * vf < 0 ? braking_accel : max_horizontal_accel;
                    const double ratio = std::hypot(af / budget, ar / max_horizontal_accel);
                    if (ratio > 1.) {
                        af /= ratio;
                        ar /= ratio;
                    }
                    an = cy * af - sy * ar;
                    ae = sy * af + cy * ar;
                }
                const double degrees = 180. / std::acos(-1.);
                const double target_pitch = -std::atan2(cy * an + sy * ae, 9.81) * degrees;
                const double target_roll = std::atan2(-sy * an + cy * ae, 9.81) * degrees;
                predicted_pitch += dt / (.18 + dt) * (target_pitch - predicted_pitch);
                predicted_roll += dt / (.18 + dt) * (target_roll - predicted_roll);
                p.pitch_deg = predicted_pitch;
                p.roll_deg = predicted_roll;
            }
            const std::array<double, 3> target{original_target[0] + target_velocity[0] * (age + t),
                                               original_target[1] + target_velocity[1] * (age + t),
                                               original_target[2]};
            const auto uv = geometry->image_projection_with_depth(p, target);
            if (!uv || (*uv)[2] <= .01) {
                result.valid = false;
                break;
            }
            const double depth_scale = std::clamp((*measured)[2] / (*uv)[2], .5, 2.);
            const double x = (*uv)[0] + bias_x, anchor = (*uv)[1] + bias_y;
            const double top = anchor - anchor_y * height * depth_scale;
            const double bottom = anchor + (1. - anchor_y) * height * depth_scale;
            const double left = x - half_width * depth_scale,
                         right_edge = x + half_width * depth_scale;
            if (trace) {
                trace->steps[trace->step_count++] = {t,      left,       right_edge, top,
                                                     bottom, p.roll_deg, p.pitch_deg};
            }
            const double center_y = .5 * (top + bottom);
            result.score += 2. * (x * x + center_y * center_y) +
                            160. * (wall(x + half_width * depth_scale) +
                                    wall(x - half_width * depth_scale) + wall(top) + wall(bottom));
            if (lateral_enabled || diagnostic) {
                result.worst_edge =
                    std::max({result.worst_edge, std::abs(left), std::abs(right_edge),
                              std::abs(top), std::abs(bottom)});
                if (std::isfinite(desired_range_m) && desired_range_m > 0) {
                    const double range = std::hypot(target[0] - p.north_m, target[1] - p.east_m);
                    result.max_range_error_m =
                        std::max(result.max_range_error_m, std::abs(range - desired_range_m));
                }
            }
        }
        result.valid = result.valid && std::isfinite(result.score) &&
                       std::isfinite(result.worst_edge) && std::isfinite(result.max_range_error_m);
        if (trace) {
            trace->score = result.score;
            trace->valid = result.valid;
            trace->worst_edge = result.worst_edge;
            trace->max_range_error_m = result.max_range_error_m;
        }
        return result;
    };
    auto bounded = [&](double f, double rightward) {
        if (max_change) {
            const double dx = f - previous_forward, dy = rightward - previous_right;
            const double change = std::hypot(dx, dy);
            if (change > *max_change && change > 0) {
                f = previous_forward + dx * (*max_change) / change;
                rightward = previous_right + dy * (*max_change) / change;
            }
        }
        return std::array<double, 2>{f, rightward};
    };
    std::optional<CvVisibilityChoice> best;
    Metrics baseline_metrics;
    for (double scale : {1., .875, .75}) {
        const auto command = bounded(forward * scale, right * scale);
        CvVisibilityCandidateDiagnostic* trace = nullptr;
        if (diagnostic) {
            trace = &diagnostic->candidates[diagnostic->candidate_count++];
            trace->scale = scale;
            trace->forward_m_s = command[0];
            trace->right_m_s = command[1];
        }
        const auto metrics =
            evaluate(command[0], command[1], .1 * (1. - scale) * (1. - scale), trace);
        if (metrics.valid && (!best || metrics.score < best->score - 1e-9)) {
            best = CvVisibilityChoice{command[0], command[1], scale, metrics.score};
            baseline_metrics = metrics;
            if (diagnostic) {
                diagnostic->selected_index = static_cast<int>(diagnostic->candidate_count) - 1;
            }
        }
    }
    // Explore only at predicted edge risk. Thresholds are simulation exploration values.
    const double original_speed = std::hypot(forward, right);
    if (lateral_enabled && best && baseline_metrics.worst_edge > .75 && original_speed > 1e-8) {
        const auto original_best = *best;
        const auto original_metrics = baseline_metrics;
        double best_lateral_edge = std::numeric_limits<double>::infinity();
        std::optional<CvVisibilityChoice> lateral_best;
        int lateral_index = -1;
        for (double delta : {-.15, .15}) {
            double f = forward - delta * right / original_speed;
            double rightward = right + delta * forward / original_speed;
            const double speed = std::hypot(f, rightward);
            if (speed > max_horizontal_speed) {
                f *= max_horizontal_speed / speed;
                rightward *= max_horizontal_speed / speed;
            }
            const auto command = bounded(f, rightward);
            CvVisibilityCandidateDiagnostic* trace = nullptr;
            if (diagnostic) {
                trace = &diagnostic->candidates[diagnostic->candidate_count++];
                trace->scale = 1.;
                trace->lateral = true;
                trace->lateral_delta_m_s = delta;
                trace->forward_m_s = command[0];
                trace->right_m_s = command[1];
            }
            const auto metrics = evaluate(command[0], command[1], .1, trace);
            const double progress =
                (command[0] * forward + command[1] * right) / (original_speed * original_speed);
            const bool within_limits =
                std::hypot(command[0], command[1]) <= max_horizontal_speed + 1e-9 &&
                std::hypot(command[0] - previous_forward, command[1] - previous_right) <=
                    *max_change + 1e-9;
            if (metrics.valid && within_limits && std::isfinite(progress) && progress >= .75 &&
                metrics.worst_edge < original_metrics.worst_edge - .002 &&
                metrics.score < original_best.score &&
                metrics.max_range_error_m <= original_metrics.max_range_error_m + .05 &&
                metrics.worst_edge < best_lateral_edge) {
                lateral_best =
                    CvVisibilityChoice{command[0], command[1], 1., metrics.score, true, delta};
                best_lateral_edge = metrics.worst_edge;
                if (diagnostic) {
                    lateral_index = static_cast<int>(diagnostic->candidate_count) - 1;
                }
            }
        }
        if (lateral_best) {
            best = lateral_best;
            if (diagnostic) {
                diagnostic->selected_index = lateral_index;
            }
        }
    }
    return best;
}
}  // namespace follow
