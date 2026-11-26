#include "argus/control/guidance/include/predictive_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/control/guidance/include/model_image_velocity_cost.hpp"
#include "argus/control/guidance/include/vertical_command_slew.hpp"
namespace follow {
namespace {
constexpr double kVerticalAcceleration = 0.6;  // m/s^2
constexpr double kVerticalBraking = 1.2;       // m/s^2
constexpr double rad = 0.017453292519943295, deg = 1 / rad, step = .1, image_margin = .75;
double square(double x) { return x * x; }
double wall(double x, double bound) { return square(std::max(0., std::abs(x) - bound)); }
}  // namespace
PredictiveFollower::PredictiveFollower(PredictiveFollowConfig c)
    : config_(c), projection_(c.camera) {
    // V3 기능은 설정 파일에서만 켠다 (실내·실외 기존 방식). 실외 PID·yaw 설정은 0/false 그대로.
    if (c.sent_command_slew && c.pampc_point_profile != 1) {
        throw std::invalid_argument("V3.2 sent command slew requires point profile 1");
    }
    point_ = pampc_point_settings(c.pampc_point_profile);
    if (!std::isfinite(c.point_range_deadband_m) || c.point_range_deadband_m < 0 ||
        (c.point_range_deadband_m > 0 &&
         (c.point_range_deadband_m < .1 || c.point_range_deadband_m > 1. || !point_.enabled()))) {
        throw std::invalid_argument("point range deadband must be 0 or 0.1..1.0 m in point mode");
    }
    if (c.point_range_deadband_m > 0) {
        point_.range_deadband = c.point_range_deadband_m;
    }
    range_near_deadband_ = point_.range_deadband;
    if (!std::isfinite(c.point_range_near_deadband_m) || c.point_range_near_deadband_m < 0 ||
        (c.point_range_near_deadband_m > 0 &&
         (c.point_range_near_deadband_m < .1 ||
          c.point_range_near_deadband_m > point_.range_deadband || !point_.enabled()))) {
        throw std::invalid_argument(
            "point near range deadband must be 0 or 0.1..point range deadband in point mode");
    }
    if (c.point_range_near_deadband_m > 0) {
        range_near_deadband_ = c.point_range_near_deadband_m;
    }
    if (!std::isfinite(c.point_bottom_margin) || c.point_bottom_margin < 0 ||
        (c.point_bottom_margin > 0 &&
         (c.point_bottom_margin < .3 || c.point_bottom_margin > .75 || !point_.enabled()))) {
        throw std::invalid_argument("point bottom margin must be 0 or 0.3..0.75 in point mode");
    }
    bottom_margin_ = c.point_bottom_margin;
    if (!std::isfinite(c.point_tangential_weight) || c.point_tangential_weight < 0 ||
        c.point_tangential_weight > 50 || (c.point_tangential_weight > 0 && !point_.enabled())) {
        throw std::invalid_argument("point tangential weight must be 0..50 in point mode");
    }
    image_velocity_weight_ = checked_image_velocity_weight(c.image_velocity_weight);
    if (point_.enabled()) {
        image_velocity_weight_ = point_.image_rate;
    }
    if (!std::isfinite(c.range_m + c.ground_down_m + c.velocity_response + c.max_speed_m_s +
                       c.max_backward_m_s + c.max_vertical_m_s + c.max_acceleration_m_s2 +
                       c.braking_acceleration_m_s2 + c.max_yaw_rate_deg_s + c.max_yaw_accel_deg_s2 +
                       c.min_height_m + c.max_height_m + c.position_gain + c.velocity_gain +
                       c.side_offset_m + c.integral_gain + c.integral_limit_m_s) ||
        c.range_m <= 0 || c.velocity_response <= 0 || c.max_speed_m_s <= 0 ||
        c.max_backward_m_s <= 0 || c.max_vertical_m_s <= 0 || c.max_acceleration_m_s2 <= 0 ||
        c.braking_acceleration_m_s2 < c.max_acceleration_m_s2 || c.max_yaw_rate_deg_s <= 0 ||
        // 2026-11-24 사용자 지시: 실내 데모2(yaw 고정)만 최소 높이 0.8 m까지 허용. 나머지는 1 m
        // 유지.
        c.max_yaw_accel_deg_s2 <= 0 || c.min_height_m < (c.fixed_yaw_follow ? .8 : 1) ||
        c.max_height_m > (c.outdoor_profile ? 4.8 : 2.0) || c.min_height_m >= c.max_height_m ||
        c.position_gain <= 0 || c.velocity_gain < 0 || c.integral_gain < 0 ||
        c.integral_limit_m_s <= 0 ||
        (c.integral_gain > 0 && c.integral_limit_m_s > c.max_speed_m_s) ||
        std::abs(c.side_offset_m) >= c.range_m) {
        throw std::invalid_argument("invalid predictive follower settings");
    }
    if (!std::isfinite(c.target_height_m) || c.target_height_m < 0 ||
        (c.target_height_m > 0 && (!c.outdoor_profile || c.target_height_m <= c.min_height_m ||
                                   c.target_height_m >= c.max_height_m)) ||
        (c.outdoor_profile && c.target_height_m == 0)) {
        throw std::invalid_argument(
            "outdoor profile requires a target strictly inside its height envelope");
    }
    const bool anchor_off =
        c.height_anchor_m == 0 && c.height_anchor_weight == 0 && c.height_anchor_deadband_m == 0;
    // 실외는 기존 target_height_m을 고정 높이로 쓰고, 실내는 height_anchor_m을 따로 준다.
    const double anchor = c.target_height_m > 0 ? c.target_height_m : c.height_anchor_m;
    if (!std::isfinite(c.height_anchor_m + c.height_anchor_weight + c.height_anchor_deadband_m +
                       c.vertical_deadband_m_s) ||
        (!anchor_off && (!point_.enabled() || (c.target_height_m > 0 && c.height_anchor_m != 0) ||
                         c.height_anchor_weight <= 0 || c.height_anchor_deadband_m < 0 ||
                         c.height_anchor_deadband_m > .3 || anchor < c.min_height_m + .1 ||
                         anchor > c.max_height_m - .1)) ||
        c.vertical_deadband_m_s < 0 || !std::isfinite(c.vertical_deadband_release_m) ||
        c.vertical_deadband_release_m < 0 || c.vertical_deadband_release_m > .5 ||
        !std::isfinite(c.vertical_release_max_m_s) ||
        (c.vertical_release_max_m_s != 0 && (c.vertical_release_max_m_s < c.vertical_deadband_m_s ||
                                             c.vertical_release_max_m_s > c.max_vertical_m_s)) ||
        (c.vertical_deadband_m_s > 0 &&
         (!point_.enabled() || c.vertical_deadband_m_s >= .5 * c.max_vertical_m_s))) {
        throw std::invalid_argument(
            "height anchor and vertical deadband need point mode and a height inside the envelope");
    }
    // 2026-11-19 PAMPC #3: rollout 기체 응답 이득. 0은 기존 값, 그 밖에는 0.2..5 (1/s)만 허용.
    // 2026-11-19 검토 수정: producer 기준 안전 상수 (기능이 켜져 있을 때만 작용한다).
    if (!std::isfinite(c.producer_ground_max_deviation_m + c.producer_ground_hold_s +
                       c.ground_datum_rate_m_s + c.ceiling_margin_m) ||
        c.producer_ground_max_deviation_m < .05 || c.producer_ground_max_deviation_m > .5 ||
        c.producer_ground_hold_s < 0 || c.producer_ground_hold_s > 10 ||
        c.ground_datum_rate_m_s < 0 || c.ground_datum_rate_m_s > 1 || c.ceiling_margin_m < 0 ||
        c.ceiling_margin_m > .3) {
        throw std::invalid_argument(
            "producer ground: deviation 0.05..0.5 m, hold 0..10 s, rate 0..1 m/s, margin 0..0.3 m");
    }
    // 고정 높이 + 무시 구간이 상한 - 여유 안에 있어야 기준 차이가 0일 때도 줄이지 않는다.
    if (c.use_producer_ground && !anchor_off &&
        anchor + c.height_anchor_deadband_m > c.max_height_m - c.ceiling_margin_m + 1e-9) {
        throw std::invalid_argument(
            "producer ground needs height anchor + deadband <= max height - ceiling margin");
    }
    const auto valid_plant = [](double gain) {
        return std::isfinite(gain) && (gain == 0 || (gain >= .2 && gain <= 5.));
    };
    if (!valid_plant(c.plant_horizontal_response) || !valid_plant(c.plant_vertical_response)) {
        throw std::invalid_argument("PAMPC plant responses must be 0 (legacy) or 0.2..5 1/s");
    }
    // 2026-11-19 데모2 yaw 고정 추종: point 모드 전용. 재정렬 각과 속도는 둘 다 0이거나 둘 다 양수.
    if (!std::isfinite(c.fixed_yaw_realign_deg + c.fixed_yaw_realign_rate_deg_s) ||
        (c.fixed_yaw_follow &&
         (!point_.enabled() || c.fixed_yaw_realign_deg < 0 || c.fixed_yaw_realign_deg > 45 ||
          c.fixed_yaw_realign_rate_deg_s < 0 ||
          c.fixed_yaw_realign_rate_deg_s > c.max_yaw_rate_deg_s ||
          (c.fixed_yaw_realign_deg > 0) != (c.fixed_yaw_realign_rate_deg_s > 0))) ||
        (!c.fixed_yaw_follow &&
         (c.fixed_yaw_realign_deg != 0 || c.fixed_yaw_realign_rate_deg_s != 0))) {
        throw std::invalid_argument(
            "fixed yaw follow needs point mode, realign 0..45 deg and rate 0..max yaw rate "
            "(both zero or both positive)");
    }
}
void PredictiveFollower::reset() {
    reset_motion();
    offset_axis_.reset();
}
void PredictiveFollower::reset_motion() {
    successful_world_command_.reset();
    pending_world_command_.reset();
    successful_command_ns_ = pending_command_ns_ = 0;
    reset_plan();
    reset_integral();
    previous_yaw_rate_ = 0;
    filtered_los_rate_.reset();  // 2026-11-16 시선각 필터도 처음부터
    fixed_yaw_realigning_ = false;
    previous_vertical_command_.reset();
    pending_vertical_command_.reset();
    successful_vertical_ns_ = pending_vertical_ns_ = 0;
    previous_box_extent_.reset();
    box_width_growth_ = 0;
    box_growth_started_ns_ = 0;
    visible_shape_.reset();
    feature_observation_.reset();
    feature_rate_ = {};
    image_velocity_diagnostics_ = {};
}
void PredictiveFollower::reset_plan() { previous_plan_.reset(); }
void PredictiveFollower::reset_integral() {
    integral_ = {};
    pending_integral_.reset();
    integral_diagnostics_ = {};
    image_velocity_diagnostics_ = {};
}
void PredictiveFollower::set_yaw_plan_options(double plan_hysteresis, bool stop_keeps_seed_yaw,
                                              bool point_yaw_follows_seed) {
    if (!std::isfinite(plan_hysteresis) || plan_hysteresis < 0 || plan_hysteresis > .5) {
        throw std::invalid_argument("plan hysteresis must be 0..0.5");
    }
    plan_hysteresis_ = plan_hysteresis;
    stop_keeps_seed_yaw_ = stop_keeps_seed_yaw;
    point_yaw_follows_seed_ = point_yaw_follows_seed;
}
void PredictiveFollower::set_approach_feedforward(double gain, double threshold_m_s) {
    if (!std::isfinite(gain + threshold_m_s) || gain < 0 || gain > 2 || threshold_m_s < 0 ||
        threshold_m_s > .5) {
        throw std::invalid_argument("approach feedforward gain 0..2, threshold 0..0.5 m/s");
    }
    approach_gain_ = gain;
    approach_threshold_ = threshold_m_s;
}
void PredictiveFollower::set_los_rate_filter(double tau_s) {
    if (!std::isfinite(tau_s) || tau_s < 0 || tau_s > 1) {
        throw std::invalid_argument("los rate filter must be 0..1 s");
    }
    los_rate_filter_s_ = tau_s;
    filtered_los_rate_.reset();
}
void PredictiveFollower::set_yaw_rate_cap(std::optional<double> cap_deg_s) {
    if (cap_deg_s && (!std::isfinite(*cap_deg_s) || *cap_deg_s < 0)) {
        throw std::invalid_argument("yaw rate cap must be finite and non-negative");
    }
    yaw_rate_cap_deg_s_ = cap_deg_s;
}
void PredictiveFollower::note_external_command(std::array<double, 2> world_ne_m_s,
                                               double vertical_m_s, double yaw_rate_deg_s) {
    if (!std::isfinite(world_ne_m_s[0] + world_ne_m_s[1] + vertical_m_s + yaw_rate_deg_s)) {
        throw std::invalid_argument("external command must be finite");
    }
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    pending_world_command_.reset();
    pending_vertical_command_.reset();
    pending_integral_.reset();
    successful_world_command_ = world_ne_m_s;
    successful_command_ns_ = now_ns;
    previous_vertical_command_ = vertical_m_s;
    successful_vertical_ns_ = now_ns;
    previous_yaw_rate_ = yaw_rate_deg_s * rad;
    previous_plan_.reset();
}
void PredictiveFollower::complete_update(bool sent) {
    if (pending_vertical_command_ && sent) {
        previous_vertical_command_ = pending_vertical_command_;
        successful_vertical_ns_ = pending_vertical_ns_;
    }
    pending_vertical_command_.reset();
    if (pending_world_command_ && sent) {
        successful_world_command_ = pending_world_command_;
        successful_command_ns_ = pending_command_ns_;
    }
    pending_world_command_.reset();
    if (!pending_integral_) {
        return;
    }
    if (sent) {
        integral_ = *pending_integral_;
    }
    integral_diagnostics_.committed = integral_;
    integral_diagnostics_.pending = false;
    pending_integral_.reset();
}
std::optional<PredictiveFollowResult> PredictiveFollower::update(
    const Pose& current, const Pose& observation, const TrackRecordV1& box,
    std::array<double, 2> velocity, double age, double nominal_down, double dt, double time_rate,
    bool recovered, std::optional<std::int64_t> feature_clock_ns,
    std::optional<std::array<double, 2>> gap_displacement,
    std::optional<ProducerGround> producer_ground) {
    pending_world_command_.reset();
    pending_vertical_command_.reset();
    // 2026-11-10 어느 검사에서 결과를 내지 못했는지 남긴다 (controller가 기록한다).
    last_reject_reason_.clear();
    last_slew_reconciled_ = false;
    const auto reject = [this](const char* why) -> std::optional<PredictiveFollowResult> {
        last_reject_reason_ = why;
        return std::nullopt;
    };
    const auto command_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
    const double command_dt =
        successful_world_command_
            ? std::clamp((command_now_ns - successful_command_ns_) / 1e9, .001, .1)
            : std::min(dt, .1);
    const std::array<double, 2> command_reference = successful_world_command_.value_or(
        std::array<double, 2>{current.velocity_north_m_s, current.velocity_east_m_s});
    pending_integral_.reset();
    integral_diagnostics_ = {};
    image_velocity_diagnostics_ = {};
    // V3 point 모드는 현재 관측 위치(POI)만 쓴다. 표적 속도 외삽은 쓰지 않는다.
    // 2026-11-10: 켜면 GAP_CV 공백에서만 CV 속도를 유지한다(크기는 최대 속도로 제한).
    if (point_.enabled()) {
        if (config_.gap_velocity_feedforward && gap_displacement &&
            std::isfinite(velocity[0] + velocity[1])) {
            const double speed = std::hypot(velocity[0], velocity[1]);
            if (speed > config_.max_speed_m_s) {
                velocity[0] *= config_.max_speed_m_s / speed;
                velocity[1] *= config_.max_speed_m_s / speed;
            }
        } else {
            velocity = {0., 0.};
        }
    }
    const auto& c = config_;
    if (!current.have_velocity || !box.ground ||
        !std::isfinite(velocity[0] + velocity[1] + age + dt + time_rate + nominal_down +
                       current.velocity_north_m_s + current.velocity_east_m_s +
                       current.velocity_down_m_s) ||
        age < 0 || age > (gap_displacement ? 3. : .25) || dt <= 0 || dt > .2 || time_rate < .1 ||
        time_rate > 2) {
        return reject(!current.have_velocity ? "current_velocity_missing"
                      : !box.ground          ? "target_ground_missing"
                      : (age < 0 || age > (gap_displacement ? 3. : .25))
                          ? "observation_age_out_of_range"
                      : (dt <= 0 || dt > .2)              ? "dt_out_of_range"
                      : (time_rate < .1 || time_rate > 2) ? "time_rate_out_of_range"
                                                          : "input_nonfinite");
    }
    // 2026-11-19 PAMPC #3: 예측 rollout 전용 기체 응답 이득. 0이면 기존 값과 같은 double을 쓴다.
    const double plant_horizontal =
        c.plant_horizontal_response > 0 ? c.plant_horizontal_response : c.velocity_response;
    const double plant_vertical = c.plant_vertical_response > 0 ? c.plant_vertical_response : 2.;
    // 2026-11-19 PAMPC #5: 관측과 같은 지면 기준(레이저 기반 평면 down)과 투영 자세를 고른다.
    // 꺼져 있거나 검사에 걸리면 기존 기준(ground_down_m, observation pose)을 그대로 쓴다.
    PampcModelDiagnostics model;
    model.plant_horizontal_response = plant_horizontal;
    model.plant_vertical_response = plant_vertical;
    double ground_datum = c.ground_down_m;
    Pose projection_observation = observation;
    if (producer_ground) {
        // 꺼져 있어도 받은 값을 기록한다 (이후 모델 검증용).
        model.producer_ground_down_m = producer_ground->ground_plane_down_m;
        model.producer_attitude_deg = producer_ground->attitude_deg;
    }
    if (c.use_producer_ground) {
        auto fallback = PampcGroundFallback::None;
        // 2026-11-19 검토 수정: 이 tick 계획기가 향할 기준(target_datum). 실제 적용 값은 아래에서
        // 변화율 제한을 거친다. 기본은 기존 기준이다.
        double target_datum = c.ground_down_m;
        // FC epoch이 바뀌면 유지 값과 적용 값을 버리고 기존 기준에서 다시 시작한다.
        if (producer_ground && datum_epoch_ && *datum_epoch_ != producer_ground->fc_epoch) {
            held_datum_.reset();
            applied_datum_.reset();
        }
        if (producer_ground) {
            datum_epoch_ = producer_ground->fc_epoch;
        }
        if (!producer_ground) {
            fallback = PampcGroundFallback::Absent;
        } else {
            const double plane = producer_ground->ground_plane_down_m;
            const auto& attitude = producer_ground->attitude_deg;
            const double yaw_difference =
                std::abs(std::remainder(attitude[2] - observation.yaw_deg, 360.));
            if (!std::isfinite(plane + attitude[0] + attitude[1] + attitude[2])) {
                fallback = PampcGroundFallback::NonFinite;
            } else if (std::abs(plane - c.ground_down_m) > c.producer_ground_max_deviation_m) {
                fallback = PampcGroundFallback::Deviation;
            } else if (std::abs(attitude[0]) > kProducerAttitudeMaxTiltDeg ||
                       std::abs(attitude[1]) > kProducerAttitudeMaxTiltDeg ||
                       std::abs(attitude[0] - observation.roll_deg) >
                           kProducerAttitudeMaxDifferenceDeg ||
                       std::abs(attitude[1] - observation.pitch_deg) >
                           kProducerAttitudeMaxDifferenceDeg ||
                       !(yaw_difference <= kProducerAttitudeMaxDifferenceDeg)) {
                fallback = PampcGroundFallback::Attitude;
            } else if (age > kProducerGroundMaxAgeS) {
                fallback = PampcGroundFallback::Stale;
            }
            if (fallback == PampcGroundFallback::None) {
                target_datum = plane;
                held_datum_ = HeldDatum{plane, command_now_ns};
                projection_observation.roll_deg = attitude[0];
                projection_observation.pitch_deg = attitude[1];
                projection_observation.yaw_deg = attitude[2];
                model.datum_source = PampcGroundSource::Producer;
                model.attitude_source = PampcGroundSource::Producer;
            }
        }
        // 오래된 관측(Stale)이나 짧은 필드 공백(Absent)은 마지막으로 받은 평면을 hold_s 동안
        // 유지한다. 투영 자세는 프레임마다 기존 관측 pose로 돌아간다. 값이 틀린 경우(NonFinite,
        // Deviation, Attitude)는 유지하지 않고 기존 기준으로 향한다.
        if ((fallback == PampcGroundFallback::Stale || fallback == PampcGroundFallback::Absent) &&
            held_datum_ && c.producer_ground_hold_s > 0 &&
            (command_now_ns - held_datum_->accepted_ns) / 1e9 <= c.producer_ground_hold_s) {
            target_datum = held_datum_->plane;
            model.datum_source = PampcGroundSource::Held;
            ++producer_ground_held_ticks_;
        }
        if (fallback == PampcGroundFallback::None) {
            ++producer_ground_ticks_;
        } else {
            ++producer_ground_fallback_ticks_;
        }
        model.fallback = fallback;
        model.target_datum_down_m = target_datum;
        // 적용 기준은 양방향 모두 변화율 제한 (epoch 첫 tick은 기존 기준에서 시작한다).
        // rate 0이면 제한 없이 바로 바꾼다.
        double applied = applied_datum_.value_or(c.ground_down_m);
        if (c.ground_datum_rate_m_s > 0) {
            const double step = c.ground_datum_rate_m_s * (dt / time_rate);
            applied = std::clamp(target_datum, applied - step, applied + step);
        } else {
            applied = target_datum;
        }
        applied_datum_ = applied;
        ground_datum = applied;
    }
    model.ground_datum_down_m = ground_datum;
    model.projection_attitude_deg = {projection_observation.roll_deg,
                                     projection_observation.pitch_deg,
                                     projection_observation.yaw_deg};
    // 높이 하한 검사는 두 기준 중 낮은 높이, 상한 검사는 높은 높이를 쓴다 (기존 EKF 기준 천장을
    // 넘지 않게 한다). 기능이 꺼져 있으면 두 값이 같아 기존과 같다.
    const double floor_datum = std::min(ground_datum, c.ground_down_m);
    const double ceiling_datum = std::max(ground_datum, c.ground_down_m);
    // A new or recovered session starts from the measured FC velocity. The controller resets
    // this history on loss, STOP, RC takeover and epoch changes. `dt` is sensor-time scaled;
    // the output rate budget must use elapsed host time instead.
    const double previous_vertical =
        std::clamp(previous_vertical_command_.value_or(current.velocity_down_m_s),
                   -c.max_vertical_m_s, c.max_vertical_m_s);
    const double vertical_change = c.max_acceleration_m_s2 * (dt / time_rate);
    // V3.3 rate limit only: same model/cost/safety, last accepted vz is the reference.
    const double vertical_dt =
        previous_vertical_command_
            ? std::clamp((command_now_ns - successful_vertical_ns_) / 1e9, .001, .1)
            : std::min(dt, .1);
    const double anchor_y = box.ground_anchor ? (*box.ground_anchor)[0] : c.camera.box_anchor_y;
    const double anchor_height =
        box.ground_anchor ? (*box.ground_anchor)[1] : c.camera.target_anchor_height_above_ground_m;
    // 2026-11-19 #5: 표적 높이와 t0 재투영은 고른 지면 기준과 투영 자세를 쓴다 (꺼짐이면 기존).
    std::array<double, 3> target{(*box.ground)[0], (*box.ground)[1], ground_datum - anchor_height};
    const auto measured = projection_.image_projection_with_depth(projection_observation, target);
    if (!measured) {
        return reject("observation_projection_failed");
    }
    const double observed_x = (box.x1 + box.x2) / c.camera.image_width - 1;
    const bool roof = anchor_y == 0;
    const double observed_y =
        2 * (box.y1 + anchor_y * (box.y2 - box.y1)) / c.camera.image_height - 1;
    // 현재 실제 접점과 중심 사전값의 차이만 보정한다. 원 시각/TTL을 만들지 않는다.
    const std::array<double, 2> bias{observed_x - (*measured)[0], observed_y - (*measured)[1]};
    // 만료 검사는 host 초 그대로, 위치 외삽만 센서 시간 초로 바꾼다.
    target[0] += gap_displacement ? (*gap_displacement)[0] : velocity[0] * age * time_rate;
    target[1] += gap_displacement ? (*gap_displacement)[1] : velocity[1] * age * time_rate;
    const double dn = target[0] - current.north_m, de = target[1] - current.east_m,
                 range = std::hypot(dn, de);
    if (range < .1 || !std::isfinite(range)) {
        return reject("target_range_too_small");
    }
    if (!offset_axis_) {
        offset_axis_ = std::array<double, 2>{dn / range, de / range};
    }
    const auto base_axis = *offset_axis_;
    const double back = std::sqrt(c.range_m * c.range_m - c.side_offset_m * c.side_offset_m);
    // 정면 이동축은 명시 시작 때 고정하고 총간격을 유지한 채 옆간격을 조립한다.
    const std::array<double, 2> axis =
        point_.enabled() ? std::array<double, 2>{dn / range, de / range}
                         : std::array<double, 2>{
                               (back * base_axis[0] + c.side_offset_m * base_axis[1]) / c.range_m,
                               (back * base_axis[1] - c.side_offset_m * base_axis[0]) / c.range_m};
    std::array<double, 2> goal{target[0] - c.range_m * axis[0], target[1] - c.range_m * axis[1]};
    if (point_.enabled()) {
        const double radial_error = range - std::clamp(range, c.range_m - range_near_deadband_,
                                                       c.range_m + point_.range_deadband);
        goal = {current.north_m + radial_error * axis[0], current.east_m + radial_error * axis[1]};
        const double center_y = (box.y1 + box.y2) / c.camera.image_height - 1;
        if (radial_error == 0 && std::abs(observed_x) <= point_.center_deadband &&
            std::abs(center_y) <= point_.center_deadband &&
            std::hypot(current.velocity_north_m_s, current.velocity_east_m_s) < .1 &&
            std::abs(current.velocity_down_m_s) < .1) {
            reset_integral();
        }
    }
    // 접근·제동 때 하단 시야를 잃기 전에 낮은 정지 높이를 확보한다. 가속 때는 시야 비용으로
    // 상승한다.
    // 실외는 명시 목표 높이(4m)를 유지한다. 실내 V3 point 모드는 현재 높이를 범위 안에서 유지한다.
    // 2026-11-09 고도 고정항을 켜면 point 모드도 설정 높이를 기준으로 둔다.
    const bool height_anchored = point_.enabled() && c.height_anchor_weight > 0;
    double height_ref =
        c.target_height_m > 0 ? c.target_height_m
        : height_anchored     ? c.height_anchor_m
        : point_.enabled()
            ? std::clamp(ground_datum - current.down_m, c.min_height_m, c.max_height_m)
            : std::clamp(std::min(ground_datum - nominal_down, c.max_height_m - .15),
                         c.min_height_m + .05, c.max_height_m - .05);
    // 2026-11-19 검토 수정: producer 기준을 쓰면 고정 높이 + 무시 구간이 EKF 기준 상한보다
    // ceiling_margin_m 이상 아래에 있도록 기준 높이를 줄인다. EKF 높이 = 기준 높이 + (ground_down_m
    // - ground_datum). 기능이 꺼져 있으면 쓰지 않는다 (기존 동작).
    if (c.use_producer_ground) {
        const double deadband = height_anchored ? c.height_anchor_deadband_m : 0.;
        const double limit = std::max(
            c.max_height_m - c.ceiling_margin_m - deadband - (c.ground_down_m - ground_datum),
            c.min_height_m + .1);
        model.height_ref_limit_m = limit;
        if (height_ref > limit) {
            height_ref = limit;
            model.height_ref_clamped = true;
            ++height_ref_clamped_ticks_;
        }
    }
    const double width = (box.x2 - box.x1) / c.camera.image_width;
    if (box.last_real_detection_monotonic_ns > 0 &&
        (!previous_box_extent_ ||
         box.last_real_detection_monotonic_ns != previous_box_extent_->sample_ns)) {
        const double extent = width * (*measured)[2];
        const double elapsed =
            previous_box_extent_
                ? (box.last_real_detection_monotonic_ns - previous_box_extent_->sample_ns) / 1e9 *
                      time_rate
                : 0;
        if (!previous_box_extent_) {
            box_growth_started_ns_ = box.last_real_detection_monotonic_ns;
        }
        const double since_reset =
            (box.last_real_detection_monotonic_ns - box_growth_started_ns_) / 1e9 * time_rate;
        // 깊이 변화를 제외한 새 실박스의 복원/회전 폭 증가만 예측 시야 비용에 반영한다.
        // 복귀 직후 관측/예측 두 250ms 구간에만 적용하고 일반 곡선의 기존 계획은 보존한다.
        // 재확인 첫 명령은 아직 폭 변화 쌍이 없다. 다음 실표본 전까지 같은 유계 여유를 먼저
        // 사용한다.
        box_width_growth_ =
            !previous_box_extent_ && recovered ? extent / .25
            : since_reset <= .5 && elapsed > 0 && elapsed <= .25
                ? std::max(0., (extent - previous_box_extent_->width_depth) / elapsed)
                : 0;
        previous_box_extent_ = BoxExtent{box.last_real_detection_monotonic_ns, extent};
    }
    const double box_height = 2 * (box.y2 - box.y1) / c.camera.image_height;
    // 급격한 실루엣 축소는 관측 불확실성이다. 최근 큰 실관측을 FOV 비용에만 보존한다.
    const auto sample = box.last_real_detection_monotonic_ns;
    const double shape_age =
        visible_shape_ ? (sample - visible_shape_->sample_ns) / 1e9 * time_rate : 0;
    if (visible_shape_ && (sample < visible_shape_->sample_ns || shape_age > 1.2 ||
                           box.track_id != visible_shape_->track_id)) {
        visible_shape_.reset();
    }
    const double extent = width * (*measured)[2], height_extent = box_height * (*measured)[2];
    const bool contracting = sample > 0 && visible_shape_ && shape_age > 0 &&
                             extent < .9 * visible_shape_->width_depth &&
                             height_extent < .95 * visible_shape_->height_depth &&
                             (1 - extent / visible_shape_->width_depth) / shape_age > .5;
    if (sample > 0 && !contracting &&
        (!visible_shape_ ||
         (sample > visible_shape_->sample_ns && extent >= .98 * visible_shape_->width_depth &&
          height_extent >= .98 * visible_shape_->height_depth))) {
        visible_shape_ = VisibleShape{sample,
                                      box.track_id,
                                      extent,
                                      height_extent,
                                      {(*box.ground)[0], (*box.ground)[1], target[2]},
                                      velocity,
                                      {(*box.ground)[2], (*box.ground)[3], (*box.ground)[4]},
                                      bias};
    }
    if (point_.enabled()) {
        if (contracting || !feature_clock_ns || *feature_clock_ns <= 0) {
            feature_observation_.reset();
            feature_rate_ = {};
        } else if (!feature_observation_ || *feature_clock_ns != feature_observation_->clock_ns) {
            const auto previous = feature_observation_;
            if (previous && previous->track_id == box.track_id) {
                const double elapsed = (*feature_clock_ns - previous->clock_ns) / 1e9;
                const auto stationary = projection_.image_projection_with_depth(
                    projection_observation, previous->point);
                if (elapsed >= .01 && elapsed <= .2 && stationary) {
                    const double actual_range =
                        std::hypot(target[0] - observation.north_m, target[1] - observation.east_m);
                    const double static_range = std::hypot(previous->point[0] - observation.north_m,
                                                           previous->point[1] - observation.east_m);
                    const std::array<double, 3> residual{
                        (observed_x - (*stationary)[0] - previous->bias[0]) / elapsed,
                        (observed_y - (*stationary)[1] - previous->bias[1]) / elapsed,
                        (actual_range - static_range) / elapsed};
                    const std::array<double, 3> limits{2., 2., 1.5};
                    if (std::isfinite(residual[0] + residual[1] + residual[2])) {
                        for (unsigned axis = 0; axis < 3; ++axis) {
                            feature_rate_[axis] +=
                                elapsed / (.15 + elapsed) *
                                (std::clamp(residual[axis], -limits[axis], limits[axis]) -
                                 feature_rate_[axis]);
                        }
                    } else {
                        feature_rate_ = {};
                    }
                } else {
                    feature_rate_ = {};
                }
            } else {
                feature_rate_ = {};
            }
            feature_observation_ =
                FeatureObservation{*feature_clock_ns, box.track_id, target, bias};
        }
    }
    const double current_speed = std::hypot(current.velocity_north_m_s, current.velocity_east_m_s);
    const double target_speed = std::hypot(velocity[0], velocity[1]);
    const bool reversal_response =
        current_speed > .1 && target_speed > .15 &&
        current.velocity_north_m_s * velocity[0] + current.velocity_east_m_s * velocity[1] <
            -.5 * current_speed * target_speed;
    // 반전 제동의 빠른 roll/yaw 때 단순 bbox 크기 모델이 놓치는 형상 변화를 여유에 포함한다.
    const double visibility_margin = image_margin - (reversal_response ? .1 : 0.);
    // 하단 주변부의 bbox 양자화/형상 근사 오차에만 최대6px(480높이)의 추가 여유를 둔다.
    const double observed_bottom = 2 * box.y2 / c.camera.image_height - 1;
    const double default_bottom_margin =
        visibility_margin - .025 * std::clamp((observed_bottom - .5) / .25, 0., 1.);
    // 2026-11-16: 설정하면 하단 비용을 더 일찍 시작한다 (기존 값보다 클 때는 기존 값 유지).
    const double bottom_visibility_margin = point_.enabled() && bottom_margin_ > 0
                                                ? std::min(default_bottom_margin, bottom_margin_)
                                                : default_bottom_margin;
    const double yaw = current.yaw_deg * rad, cy = std::cos(yaw), sy = std::sin(yaw);
    // 현재 상대속도로 자세·속도 응답 동안의 위치 오차를 예측해 관성이 남기 전에 제동한다.
    auto feedback_goal = goal;
    if (target_speed > .15 && current_speed > .1 &&
        current.velocity_north_m_s * velocity[0] + current.velocity_east_m_s * velocity[1] >
            .5 * current_speed * target_speed) {
        const double excess = ((current.velocity_north_m_s - velocity[0]) * velocity[0] +
                               (current.velocity_east_m_s - velocity[1]) * velocity[1]) /
                              target_speed;
        if (excess > 0) {
            // 같은 방향의 과속 따라잡기는 기존 가속 예산의 정지거리만큼 먼저 감속한다.
            const double stopping = excess * excess / (2 * c.max_acceleration_m_s2);
            if (!std::isfinite(stopping)) {
                return reject("catch_up_stopping_nonfinite");
            }
            feedback_goal[0] -= stopping * velocity[0] / target_speed;
            feedback_goal[1] -= stopping * velocity[1] / target_speed;
        }
    }
    // The current command uses I_k. Only a successful final send commits I_{k+1};
    // optimizer evaluations below advance private copies of I_k.
    std::array<double, 2> proposed_integral = integral_;
    if (c.integral_gain > 0) {
        integral_diagnostics_.enabled = true;
        integral_diagnostics_.committed = integral_;
        integral_diagnostics_.sample_ns = box.last_real_detection_monotonic_ns;
        integral_diagnostics_.error = {feedback_goal[0] - current.north_m,
                                       feedback_goal[1] - current.east_m};
        for (unsigned axis = 0; axis < 2; ++axis) {
            proposed_integral[axis] = std::clamp(
                integral_[axis] + c.integral_gain * integral_diagnostics_.error[axis] * dt,
                -c.integral_limit_m_s, c.integral_limit_m_s);
        }
    }
    // 2026-11-19 #3: 시드의 1/velocity_response는 PX4 가속 setpoint 지연 보상 이득(PID형 시드)이다.
    // 사용자 지시대로 rollout 이득으로 바꾸지 않는다 (기존 시드와 같은 값).
    double seed_n = velocity[0] + c.position_gain * (feedback_goal[0] - current.north_m) +
                    (c.velocity_gain + c.position_gain * (.18 + 1 / c.velocity_response)) *
                        (velocity[0] - current.velocity_north_m_s);
    double seed_e = velocity[1] + c.position_gain * (feedback_goal[1] - current.east_m) +
                    (c.velocity_gain + c.position_gain * (.18 + 1 / c.velocity_response)) *
                        (velocity[1] - current.velocity_east_m_s);
    // 2026-11-16 접근 감쇠: 표적이 다가오는 속도만큼 미리 물러난다 (point 모드, 기본 꺼짐).
    double approach_ff = 0;
    if (point_.enabled() && approach_gain_ > 0 && feature_rate_[2] < -approach_threshold_ &&
        range <= c.range_m + point_.range_deadband && range > .1) {
        approach_ff = std::min(c.max_backward_m_s, approach_gain_ * -feature_rate_[2]);
        const double kd = 1 + c.velocity_gain + c.position_gain * (.18 + 1 / c.velocity_response);
        // 표적 → 기체 방향 (dn, de는 기체 → 표적)
        seed_n -= kd * approach_ff * dn / range;
        seed_e -= kd * approach_ff * de / range;
    }
    if (c.integral_gain > 0) {
        seed_n += integral_[0];
        seed_e += integral_[1];
    }
    // 2026-11-19 데모2: 모든 계획 후보와 송신 명령의 yaw 속도 상한. 끄면 기존 max_yaw_rate 그대로.
    // yaw 오차를 구한 뒤(아래) 이번 틱 값으로 정한다. 그 전에는 bound를 부르지 않는다.
    double yaw_limit = c.max_yaw_rate_deg_s * rad;
    const auto bound = [&](Control u, double heading) {
        const double speed = std::hypot(u[0], u[1]);
        if (speed > c.max_speed_m_s) {
            u[0] *= c.max_speed_m_s / speed;
            u[1] *= c.max_speed_m_s / speed;
        }
        const double forward = std::cos(heading) * u[0] + std::sin(heading) * u[1];
        if (forward < -c.max_backward_m_s) {
            u[0] -= (forward + c.max_backward_m_s) * std::cos(heading);
            u[1] -= (forward + c.max_backward_m_s) * std::sin(heading);
        }
        u[2] = std::clamp(u[2], -c.max_vertical_m_s, c.max_vertical_m_s);
        u[3] = std::clamp(u[3], -yaw_limit, yaw_limit);
        return u;
    };
    const double raw_los_rate = (dn * (velocity[1] - current.velocity_east_m_s) -
                                 de * (velocity[0] - current.velocity_north_m_s)) /
                                (range * range);
    // 2026-11-16: 시간 기준 1차 저역 필터 (제어 주기와 무관). 0.3초 넘게 비면 새로 시작한다.
    double los_rate = raw_los_rate;
    if (los_rate_filter_s_ > 0 && std::isfinite(raw_los_rate)) {
        if (filtered_los_rate_ && std::isfinite(dt) && dt > 0 && dt <= .3) {
            const double alpha = dt / (los_rate_filter_s_ + dt);
            los_rate = *filtered_los_rate_ + alpha * (raw_los_rate - *filtered_los_rate_);
        }
        filtered_los_rate_ = los_rate;
    }
    double yaw_error = std::remainder(std::atan2(de, dn) - yaw, 2 * std::acos(-1.));
    {
        // 반전 조건이 풀려도 잔류 roll/pitch는 고정 카메라의 광축을 움직인다.
        const double roll = current.roll_deg * rad, pitch = current.pitch_deg * rad;
        const auto& mount = c.camera.camera_to_body;
        const double optical_y = std::cos(roll) * mount[5] - std::sin(roll) * mount[8];
        const double optical_z = std::sin(roll) * mount[5] + std::cos(roll) * mount[8];
        const double optical_x = std::cos(pitch) * mount[2] + std::sin(pitch) * optical_z;
        // 영상 투영과 같은 카메라 위치에서 표적 방향을 구한다. 기체 중심 목표는 유지한다.
        const auto& offset = c.camera.camera_offset_body_m;
        const double offset_y = std::cos(roll) * offset[1] - std::sin(roll) * offset[2];
        const double offset_z = std::sin(roll) * offset[1] + std::cos(roll) * offset[2];
        const double offset_x = std::cos(pitch) * offset[0] + std::sin(pitch) * offset_z;
        const double camera_n = dn - (cy * offset_x - sy * offset_y);
        const double camera_e = de - (sy * offset_x + cy * offset_y);
        if (std::hypot(camera_n, camera_e) < .1) {
            return reject("camera_target_too_close");
        }
        yaw_error =
            std::remainder(std::atan2(camera_e, camera_n) - yaw - std::atan2(optical_y, optical_x),
                           2 * std::acos(-1.));
        if (c.fixed_yaw_follow) {
            // 2026-11-19 데모2 yaw 고정: 재정렬 판단(이력 있음)과 이번 틱 yaw 상한.
            const double error_deg = std::abs(yaw_error) * deg;
            if (c.fixed_yaw_realign_deg <= 0) {
                fixed_yaw_realigning_ = false;
            } else if (error_deg > c.fixed_yaw_realign_deg) {
                fixed_yaw_realigning_ = true;
            } else if (error_deg < .5 * c.fixed_yaw_realign_deg) {
                fixed_yaw_realigning_ = false;
            }
            yaw_limit = fixed_yaw_realigning_ ? c.fixed_yaw_realign_rate_deg_s * rad : 0.;
            // 옆이동 목표: 카메라 광축의 수평 방향에 수직인 오차만큼 옆으로 옮겨 대상을 광축
            // 앞에 둔다. 양수 = 대상이 광축 오른쪽. 5 cm 안은 무시한다(영상 중앙 비용이 마무리).
            constexpr double kFixedYawLateralDeadbandM = .05;
            const double heading = yaw + std::atan2(optical_y, optical_x);
            const double lateral_raw = std::hypot(camera_n, camera_e) * std::sin(yaw_error);
            const double lateral = std::copysign(
                std::max(0., std::abs(lateral_raw) - kFixedYawLateralDeadbandM), lateral_raw);
            const double right_n = -std::sin(heading), right_e = std::cos(heading);
            goal[0] += lateral * right_n;
            goal[1] += lateral * right_e;
            seed_n += c.position_gain * lateral * right_n;
            seed_e += c.position_gain * lateral * right_e;
        }
    }
    const Control raw_seed{seed_n, seed_e, 2 * (current.down_m - (ground_datum - height_ref)) * -1,
                           los_rate + 4 * yaw_error};
    Control seed = bound(raw_seed, yaw);
    // 반전으로 관측 차량과 기체의 진행이 반대이면 실제 수평 이동 방향에 제동 예산을 배분한다.
    // 일반 동행/곡선은 기존 body 예산, 새 방향 가속과 횡변경은 기존 가속 예산을 유지한다.
    const auto limit_horizontal_change = [&](double dn, double de, double vn, double ve,
                                             double heading) {
        const double speed = std::hypot(vn, ve);
        const bool reversing = speed > .1 && target_speed > .15 &&
                               vn * velocity[0] + ve * velocity[1] < -.5 * speed * target_speed;
        if (!reversing) {
            const double cos_y = std::cos(heading), sin_y = std::sin(heading);
            double forward = cos_y * dn + sin_y * de, right = -sin_y * dn + cos_y * de;
            const double vf = cos_y * vn + sin_y * ve;
            const double budget = std::abs(vf) > .1 && forward * vf < 0
                                      ? c.braking_acceleration_m_s2
                                      : c.max_acceleration_m_s2;
            const double ratio = std::hypot(forward / budget, right / c.max_acceleration_m_s2);
            if (ratio > 1) {
                forward /= ratio;
                right /= ratio;
            }
            return std::array<double, 2>{cos_y * forward - sin_y * right,
                                         sin_y * forward + cos_y * right};
        }
        const double n = vn / speed, e = ve / speed;
        double along = dn * n + de * e, across = -dn * e + de * n;
        const double budget = along < 0 ? c.braking_acceleration_m_s2 : c.max_acceleration_m_s2;
        const double ratio = std::hypot(along / budget, across / c.max_acceleration_m_s2);
        if (ratio > 1) {
            along /= ratio;
            across /= ratio;
        }
        return std::array<double, 2>{along * n - across * e, along * e + across * n};
    };
    // 화면 가장자리·높이 범위 경계. 실외는 설정 높이 범위(4m 기준), 실내는 기존 1.05/1.95m.
    // 2026-11-24: 데모2(yaw 고정)는 설정 하한 + 0.05 m (0.8이면 0.85). 다른 실내는 1.05 그대로.
    const double edge_floor = c.outdoor_profile    ? c.min_height_m - .05
                              : c.fixed_yaw_follow ? c.min_height_m + .05
                                                   : 1.05;
    const double edge_ceiling = c.outdoor_profile ? c.max_height_m + .05 : 1.95;
    // V3 point 모드가 계획 단계에서도 쓰는 수직 보호. 최종 명령에는 아래 기존 검사를 그대로 쓴다.
    const auto guard_vertical = [&](double vz, const Pose& state, bool image_edges) {
        // 2026-11-19 #5: 화면 가장자리 판단은 고른 기준, 하한/상한은 보수적인 기준 높이를 쓴다.
        const double height = ground_datum - state.down_m;
        const double low_height = floor_datum - state.down_m;
        const double high_height = ceiling_datum - state.down_m;
        if (image_edges && box.y2 >= .85 * c.camera.image_height &&
            height - std::max(0., state.velocity_down_m_s) / 2 > edge_floor) {
            vz = std::max(0., vz);
        }
        if (image_edges && box.y1 <= .15 * c.camera.image_height &&
            height + std::max(0., -state.velocity_down_m_s) / 2 < edge_ceiling) {
            vz = std::min(0., vz);
        }
        if (low_height - std::max(0., state.velocity_down_m_s) / 2 <= c.min_height_m) {
            vz = std::min(0., vz);
        }
        if (high_height + std::max(0., -state.velocity_down_m_s) / 2 >= c.max_height_m) {
            vz = std::max(0., vz);
        }
        return vz;
    };
    // 명령 변화는 실제 FC 속도에 대한 응답 예산으로 제한한다.
    // 2026-11-19 #3: 여기 velocity_response는 PX4가 명령에서 만드는 가속 setpoint 이득
    // (MPC_XY_VEL_P_ACC 1.8, 실측 acc_sp = 1.80 x 오차)이라 rollout 이득으로 바꾸지 않는다.
    const auto first_bound = [&](Control u) {
        u = bound(u, yaw);
        const auto acceleration =
            limit_horizontal_change((u[0] - current.velocity_north_m_s) * c.velocity_response,
                                    (u[1] - current.velocity_east_m_s) * c.velocity_response,
                                    current.velocity_north_m_s, current.velocity_east_m_s, yaw);
        u[0] = current.velocity_north_m_s + acceleration[0] / c.velocity_response;
        u[1] = current.velocity_east_m_s + acceleration[1] / c.velocity_response;
        if (c.sent_command_slew) {
            u[2] = limit_vertical_command_change(u[2], previous_vertical, vertical_dt,
                                                 kVerticalAcceleration, kVerticalBraking);
        } else {
            u[2] = std::clamp(u[2], previous_vertical - vertical_change,
                              previous_vertical + vertical_change);
        }
        const double change = c.max_yaw_accel_deg_s2 * rad * dt;
        u[3] = std::clamp(u[3], previous_yaw_rate_ - change, previous_yaw_rate_ + change);
        if (point_.enabled()) {
            u[2] = guard_vertical(u[2], current, true);
        }
        if (c.sent_command_slew) {
            const auto rate =
                limit_horizontal_change((u[0] - command_reference[0]) / command_dt,
                                        (u[1] - command_reference[1]) / command_dt,
                                        command_reference[0], command_reference[1], yaw);
            u[0] = command_reference[0] + rate[0] * command_dt;
            u[1] = command_reference[1] + rate[1] * command_dt;
        }
        return bound(u, yaw);
    };
    struct CostTerms {
        double height{0};
        double visibility{0};
        double boundary{0};
        double center{0};
    };
    std::array<double, 2> initial_image_center{};
    if (image_velocity_weight_ > 0) {
        auto image_target = target;
        if (contracting && !point_.enabled()) {
            const auto& reference = *visible_shape_;
            const double elapsed = shape_age + age * time_rate;
            image_target = {reference.target[0] + reference.velocity[0] * elapsed,
                            reference.target[1] + reference.velocity[1] * elapsed,
                            reference.target[2]};
        }
        // Same current FC pose, target/shape model and constant bias as every candidate's t=0.
        const auto uv = projection_.image_projection_with_depth(current, image_target);
        if (!uv) {
            return reject("image_projection_failed");
        }
        const auto& image_bias =
            contracting && !point_.enabled() ? visible_shape_->image_bias : bias;
        const double scale = std::clamp((*measured)[2] / std::max(.3, (*uv)[2]), .5, 2.);
        const double height = contracting && !point_.enabled()
                                  ? visible_shape_->height_depth / std::max(.3, (*uv)[2])
                                  : box_height * scale;
        initial_image_center = {
            (*uv)[0] + image_bias[0],
            v21_vertical_box_center((*uv)[1] + image_bias[1], anchor_y, height)};
        if (!std::isfinite(initial_image_center[0] + initial_image_center[1])) {
            return reject("image_center_nonfinite");
        }
    }
    const auto score = [&](const Plan& plan, double* visibility = nullptr,
                           CostTerms* breakdown = nullptr,
                           PredictiveImageVelocityDiagnostics* image_diagnostics = nullptr) {
        if (visibility) {
            *visibility = 0;
        }
        if (breakdown) {
            *breakdown = {};
        }
        Pose p = current;
        Control prior_control{current.velocity_north_m_s, current.velocity_east_m_s,
                              current.velocity_down_m_s, previous_yaw_rate_};
        if (c.sent_command_slew) {
            prior_control[0] = command_reference[0];
            prior_control[1] = command_reference[1];
        }
        double cost = 0, yaw_rate = previous_yaw_rate_;
        double planned_vertical = previous_vertical;
        auto predicted_integral = integral_;
        ModelImageVelocityCost image_rate(initial_image_center, image_velocity_weight_);
        if (image_diagnostics) {
            *image_diagnostics = {};
            image_diagnostics->weight = image_velocity_weight_;
            image_diagnostics->initial_center = initial_image_center;
        }
        std::optional<Pose> paused;
        double paused_yaw_rate = 0;
        // V3 point: 0.4+0.4+1.2초 입력 / 2초 예측. 기존: 3×0.4초 / 1.2초.
        for (int k = 0; k < (point_.enabled() ? 20 : 12); ++k) {
            const double y = p.yaw_deg * rad, cos_y = std::cos(y), sin_y = std::sin(y);
            auto requested = plan[std::min(k / 4, 2)];
            if (k >= 4 && !point_.enabled()) {
                const double t = k * step;
                requested[0] = velocity[0] +
                               c.position_gain * (goal[0] + velocity[0] * t - p.north_m) +
                               c.velocity_gain * (velocity[0] - p.velocity_north_m_s) +
                               plan[k / 4][0] - seed[0];
                requested[1] = velocity[1] +
                               c.position_gain * (goal[1] + velocity[1] * t - p.east_m) +
                               c.velocity_gain * (velocity[1] - p.velocity_east_m_s) +
                               plan[k / 4][1] - seed[1];
            }
            if (c.integral_gain > 0 && k >= 4 && !point_.enabled()) {
                requested[0] += predicted_integral[0];
                requested[1] += predicted_integral[1];
            }
            auto u = k < 4 ? first_bound(requested) : bound(requested, y);
            if (k >= 4) {
                const double planned_change = c.max_acceleration_m_s2 * step;
                u[2] = std::clamp(u[2], planned_vertical - planned_change,
                                  planned_vertical + planned_change);
            }
            if (point_.enabled()) {
                u[2] = guard_vertical(u[2], p, false);
            }
            planned_vertical = u[2];
            if (c.integral_gain > 0) {
                for (unsigned axis = 0; axis < 2; ++axis) {
                    const double position = axis ? p.east_m : p.north_m;
                    const double error = goal[axis] + velocity[axis] * k * step - position;
                    const double delta = c.integral_gain * error * step;
                    const bool toward_bound =
                        delta * (requested[axis] - u[axis]) > 1e-12 ||
                        (k < 4 && delta * (raw_seed[axis] - seed[axis]) > 1e-12);
                    if (!c.integral_anti_windup || !toward_bound) {
                        predicted_integral[axis] =
                            std::clamp(predicted_integral[axis] + delta, -c.integral_limit_m_s,
                                       c.integral_limit_m_s);
                    }
                }
                if (k == 3 || k == 7 || k == 11) {
                    integral_diagnostics_.forecast[k / 4] = predicted_integral;
                }
            }
            // 2026-11-19 #3: 예측 기체 응답(위치/자세 전파)은 실측 적합 이득을 쓴다 (0이면
            // 기존 1.8). 가속 한도는 PX4 가속 한도 모델이라 그대로 적용한다.
            double an = plant_horizontal * (u[0] - p.velocity_north_m_s);
            double ae = plant_horizontal * (u[1] - p.velocity_east_m_s);
            const auto acceleration =
                limit_horizontal_change(an, ae, p.velocity_north_m_s, p.velocity_east_m_s, y);
            an = acceleration[0];
            ae = acceleration[1];
            const double pitch = -std::atan2(cos_y * an + sin_y * ae, 9.81) * deg;
            const double roll = std::atan2(-sin_y * an + cos_y * ae, 9.81) * deg;
            p.pitch_deg += step / (.18 + step) * (pitch - p.pitch_deg);
            p.roll_deg += step / (.18 + step) * (roll - p.roll_deg);
            // 위치와 영상에 같은 예측 자세를 쓴다. 자세가 아직 바뀌기 전에 즉시 제동한 것으로
            // 계산하지 않는다.
            const double actual_af = -9.81 * std::tan(p.pitch_deg * rad),
                         actual_ar = 9.81 * std::tan(p.roll_deg * rad);
            an = cos_y * actual_af - sin_y * actual_ar;
            ae = sin_y * actual_af + cos_y * actual_ar;
            // 2026-11-19 #3: 수직 예측 응답 이득 (0이면 기존 2.0).
            const double az = plant_vertical * (u[2] - p.velocity_down_m_s);
            p.north_m += p.velocity_north_m_s * step + .5 * an * step * step;
            p.east_m += p.velocity_east_m_s * step + .5 * ae * step * step;
            p.down_m += p.velocity_down_m_s * step + .5 * az * step * step;
            p.velocity_north_m_s += an * step;
            p.velocity_east_m_s += ae * step;
            p.velocity_down_m_s += az * step;
            yaw_rate += step / (.12 + step) * (u[3] - yaw_rate);
            p.yaw_deg += yaw_rate * step * deg;
            const double t = (k + 1) * step;
            if (contracting) {
                if (k == 0) {
                    paused = p;
                    paused_yaw_rate = yaw_rate;
                }
                if (paused) {
                    // 미래 관측이 끊기면 원 Guard의 중립 명령으로 제동하는 경우도 미리 평가한다.
                    auto& stopped = *paused;
                    if (k > 0) {
                        const double angle = stopped.yaw_deg * rad, cn = std::cos(angle),
                                     sn = std::sin(angle);
                        // 2026-11-19 #3: 제동 계획도 예측 기체 응답이라 rollout 이득을 쓴다.
                        const auto brake = limit_horizontal_change(
                            -plant_horizontal * stopped.velocity_north_m_s,
                            -plant_horizontal * stopped.velocity_east_m_s,
                            stopped.velocity_north_m_s, stopped.velocity_east_m_s, angle);
                        const double pitch = -std::atan2(cn * brake[0] + sn * brake[1], 9.81) * deg;
                        const double roll = std::atan2(-sn * brake[0] + cn * brake[1], 9.81) * deg;
                        stopped.pitch_deg += step / (.18 + step) * (pitch - stopped.pitch_deg);
                        stopped.roll_deg += step / (.18 + step) * (roll - stopped.roll_deg);
                        const double forward = -9.81 * std::tan(stopped.pitch_deg * rad),
                                     right = 9.81 * std::tan(stopped.roll_deg * rad);
                        const double an = cn * forward - sn * right, ae = sn * forward + cn * right;
                        stopped.north_m +=
                            stopped.velocity_north_m_s * step + .5 * an * step * step;
                        stopped.east_m += stopped.velocity_east_m_s * step + .5 * ae * step * step;
                        stopped.velocity_north_m_s += an * step;
                        stopped.velocity_east_m_s += ae * step;
                        // 수직 0 명령의 1차 응답 (이득 plant_vertical, 기존 2.0).
                        const double decay = std::exp(-plant_vertical * step);
                        stopped.down_m += stopped.velocity_down_m_s * (1 - decay) / plant_vertical;
                        stopped.velocity_down_m_s *= decay;
                        const double yaw_decay = std::exp(-step / .12);
                        stopped.yaw_deg += paused_yaw_rate * (1 - yaw_decay) * .12 * deg;
                        paused_yaw_rate *= yaw_decay;
                    }
                    const auto& reference = *visible_shape_;
                    const double elapsed = shape_age + age * time_rate + t;
                    // Point mode keeps historical size/covariance, never a historical world POI.
                    const std::array<double, 3> anticipated =
                        point_.enabled()
                            ? target
                            : std::array<double, 3>{
                                  reference.target[0] + reference.velocity[0] * elapsed,
                                  reference.target[1] + reference.velocity[1] * elapsed,
                                  reference.target[2]};
                    const auto view = projection_.image_projection_with_depth(stopped, anticipated);
                    if (!view) {
                        return std::numeric_limits<double>::infinity();
                    }
                    auto north = anticipated, east = anticipated;
                    north[0] += .01;
                    east[1] += .01;
                    const auto view_n = projection_.image_projection_with_depth(stopped, north);
                    const auto view_e = projection_.image_projection_with_depth(stopped, east);
                    if (!view_n || !view_e) {
                        return std::numeric_limits<double>::infinity();
                    }
                    const double jn = ((*view_n)[0] - (*view)[0]) / .01,
                                 je = ((*view_e)[0] - (*view)[0]) / .01;
                    const auto& covariance = reference.ground_covariance;
                    const double variance = jn * jn * covariance[0] + 2 * jn * je * covariance[1] +
                                            je * je * covariance[2];
                    if (!std::isfinite(variance)) {
                        return std::numeric_limits<double>::infinity();
                    }
                    // 실제 지면 관측의 공분산을 영상에 투영한다. 2σ 여유는 확률 보장이 아닌 선형
                    // 근사다.
                    const double radius = reference.width_depth / std::max(.3, (*view)[2]) +
                                          2 * std::sqrt(std::max(0., variance));
                    // V3 point 모드는 과거 형상 크기만 쓰고 현재 관측 bias를 쓴다.
                    const auto& current_bias = point_.enabled() ? bias : reference.image_bias;
                    const double risk =
                        std::max(0., std::abs((*view)[0] + current_bias[0]) + radius - 1.);
                    if (visibility) {
                        *visibility = std::max(*visibility, risk);
                    }
                    cost += 800 * square(risk);
                    if (breakdown) {
                        breakdown->visibility += 800 * square(risk);
                    }
                }
            }
            const std::array<double, 3> future{target[0] + velocity[0] * t,
                                               target[1] + velocity[1] * t, target[2]};
            auto image_target = future;
            if (contracting && !point_.enabled()) {
                // 현재 조각의 중심을 미래의 전체 차 중심으로 쓰지 않는다. 실기억은 예측 비용에만
                // 사용한다.
                const auto& reference = *visible_shape_;
                const double elapsed = shape_age + age * time_rate + t;
                image_target = {reference.target[0] + reference.velocity[0] * elapsed,
                                reference.target[1] + reference.velocity[1] * elapsed,
                                reference.target[2]};
            }
            const auto uv = projection_.image_projection_with_depth(p, image_target);
            if (!uv) {
                return std::numeric_limits<double>::infinity();
            }
            const auto& image_bias =
                contracting && !point_.enabled() ? visible_shape_->image_bias : bias;
            const double feature_horizon = point_.enabled() ? t : 0.;
            const double x = (*uv)[0] + image_bias[0] + feature_rate_[0] * feature_horizon,
                         anchor_y = (*uv)[1] + image_bias[1] + feature_rate_[1] * feature_horizon;
            // 원 bbox 크기는 수평거리 대신 원관측/예측의 실제 광학 깊이 비율로 축척한다.
            const double scale = std::clamp((*measured)[2] / std::max(.3, (*uv)[2]), .5, 2.);
            // 성장 외삽은 원관측 유효 구간과 현재 관측 폭 한 배 이내로 제한한다.
            const double future_width =
                contracting
                    ? visible_shape_->width_depth / std::max(.3, (*uv)[2])
                    : (width + std::clamp(box_width_growth_ * std::min(t, .25) / (*measured)[2], 0.,
                                          width)) *
                          scale;
            const double future_height = contracting
                                             ? visible_shape_->height_depth / std::max(.3, (*uv)[2])
                                             : box_height * scale;
            const double center_height = point_.enabled() ? box_height * scale : future_height;
            const double top = roof ? anchor_y : anchor_y - future_height;
            const double bottom = roof ? anchor_y + future_height : anchor_y;
            if (image_velocity_weight_ > 0) {
                const double anchor_fraction =
                    box.ground_anchor ? (*box.ground_anchor)[0] : c.camera.box_anchor_y;
                const std::array<double, 2> centre{
                    x, v21_vertical_box_center(anchor_y, anchor_fraction, center_height)};
                const auto image_cost = image_rate.add(centre, step);
                if (!image_cost) {
                    return std::numeric_limits<double>::infinity();
                }
                cost += *image_cost;
                if (image_diagnostics) {
                    image_diagnostics->cost += *image_cost;
                    ++image_diagnostics->steps;
                    image_diagnostics->forecast[k] = {centre[0], centre[1], image_rate.rate()[0],
                                                      image_rate.rate()[1]};
                }
            }
            const double static_range = std::hypot(future[0] - p.north_m, future[1] - p.east_m);
            const double future_range =
                point_.enabled() ? std::max(.1, static_range + feature_rate_[2] * t) : static_range;
            // 하강으로 영상 비용만 낮추기 전에 접근 중 남은 간격을 확보한다.
            const double minimum_range =
                point_.enabled() ? c.range_m - range_near_deadband_ : c.range_m;
            const double approach_margin = std::max(0., minimum_range - future_range) / c.range_m;
            if (visibility) {
                *visibility = std::max(
                    *visibility,
                    std::max({approach_margin, std::abs(x) + future_width - visibility_margin,
                              -top - visibility_margin, bottom - bottom_visibility_margin}));
            }
            // 2026-11-19 #5: 높이 비용, 고정 높이, 높이 벽은 고른 지면 기준 높이를 쓴다.
            const double h = ground_datum - p.down_m;
            if (point_.enabled()) {
                const double fraction =
                    box.ground_anchor ? (*box.ground_anchor)[0] : c.camera.box_anchor_y;
                const double center_y = v21_vertical_box_center(anchor_y, fraction, center_height);
                cost += point_.center * (wall(x, point_.center_deadband) +
                                         1.5 * wall(center_y, point_.center_deadband));
                // 2026-11-16: 가까운 쪽/먼 쪽 무시 구간을 따로 본다 (같으면 기존 wall과 같은 값).
                const double range_offset = future_range - c.range_m;
                cost +=
                    point_.range * (range_offset < 0 ? wall(range_offset, range_near_deadband_)
                                                     : wall(range_offset, point_.range_deadband));
                cost += height_anchored ? c.height_anchor_weight *
                                              wall(h - height_ref, c.height_anchor_deadband_m)
                                        : point_.height * square(h - height_ref);
                cost += point_.attitude * (square(p.roll_deg * rad) + square(p.pitch_deg * rad));
                cost += point_.self_velocity *
                        (square(p.velocity_north_m_s) + square(p.velocity_east_m_s));
                cost += point_.acceleration * (square(an) + square(ae));
                if (c.point_tangential_weight > 0) {
                    // 2026-11-10 대상 둘레 방향 상대속도. 거리 방향 접근·후퇴와 yaw는 막지 않는다.
                    const double ln = future[0] - p.north_m, le = future[1] - p.east_m;
                    const double lr = std::max(.1, std::hypot(ln, le));
                    const double tangential = ((p.velocity_north_m_s - velocity[0]) * -le +
                                               (p.velocity_east_m_s - velocity[1]) * ln) /
                                              lr;
                    cost += c.point_tangential_weight * square(tangential);
                }
                for (unsigned d = 0; d < 4; ++d) {
                    cost += point_.control * square(u[d]) +
                            point_.slew * square(u[d] - prior_control[d]);
                }
                prior_control = u;
            } else {
                cost += 4 * (square(future[0] - c.range_m * axis[0] - p.north_m) +
                             square(future[1] - c.range_m * axis[1] - p.east_m));
                cost += .8 * (square(p.velocity_north_m_s - velocity[0]) +
                              square(p.velocity_east_m_s - velocity[1]));
                cost += 2 * square(h - height_ref) + 2 * square(x) +
                        .06 * (square(an) + square(ae)) + .08 * square(u[3]);
            }
            // 실제 bbox 좌우·상하 여유. 순간 잡음으로 infeasible을 만들지 않는다.
            double center_cost = 0;
            if (!point_.enabled() && v21_enabled("ARGUS_V21_VERTICAL_CENTER")) {
                const double anchor_fraction =
                    box.ground_anchor ? (*box.ground_anchor)[0] : c.camera.box_anchor_y;
                const double center_y =
                    v21_vertical_box_center(anchor_y, anchor_fraction, future_height);
                center_cost = .5 * square(center_y);
                cost += center_cost;
            }
            cost += 160 * (wall(x + future_width, visibility_margin) +
                           wall(x - future_width, visibility_margin) +
                           wall(top, visibility_margin) + wall(bottom, bottom_visibility_margin));
            cost += 800 * square(approach_margin);
            // 2026-11-19 검토 수정: 높이 벽은 하한을 낮은 기준, 상한을 높은 기준(EKF)으로 본다.
            const double h_low = floor_datum - p.down_m, h_high = ceiling_datum - p.down_m;
            cost += 800 * (square(std::max(0., c.min_height_m - h_low)) +
                           square(std::max(0., h_high - c.max_height_m)));
            if (breakdown) {
                breakdown->center += center_cost;
                breakdown->height +=
                    height_anchored
                        ? c.height_anchor_weight * wall(h - height_ref, c.height_anchor_deadband_m)
                        : 2 * square(h - height_ref);
                breakdown->visibility +=
                    160 * (wall(x + future_width, visibility_margin) +
                           wall(x - future_width, visibility_margin) +
                           wall(top, visibility_margin) + wall(bottom, bottom_visibility_margin));
                breakdown->boundary += 800 * square(approach_margin) +
                                       800 * (square(std::max(0., c.min_height_m - h_low)) +
                                              square(std::max(0., h_high - c.max_height_m)));
            }
        }
        return cost;
    };
    Plan best{seed, seed, seed};
    // 미래에도 현재 yaw P 출력을 계속 적용하면 과회전을 예측해 현재 회전을 미룬다.
    for (std::size_t block = 1; block < 3; ++block) {
        best[block][3] = los_rate + (seed[3] - los_rate) * std::exp(-4 * .4 * block);
    }
    double best_score = score(best);
    yaw_trace_ = {};
    yaw_trace_.los_rate = los_rate;
    yaw_trace_.p_term = 4 * yaw_error;
    yaw_trace_.seed = seed[3];
    yaw_trace_.seed_score = best_score;
    yaw_trace_.previous = previous_yaw_rate_;
    yaw_trace_.range_rate = feature_rate_[2];
    yaw_trace_.approach_ff = approach_ff;
    if (point_.enabled()) {
        // V3 point 모드는 정지 계획도 같은 비용으로 비교한다.
        Plan stopped{};
        if (stop_keeps_seed_yaw_) {
            for (std::size_t b = 0; b < 3; ++b) {
                stopped[b][3] = best[b][3];  // 2026-11-16 ②: 이동만 멈추고 요는 시드 일정 유지
            }
        }
        const double value = score(stopped);
        if (value < best_score) {
            best = stopped;
            best_score = value;
            yaw_trace_.source = 1;
        }
    }
    std::optional<Plan> warm_plan;  // 2026-11-16 ①
    double warm_score = std::numeric_limits<double>::infinity();
    if (previous_plan_) {
        auto warm = *previous_plan_;
        // receding horizon의 실제 경과만큼 warm start를 이동한다.
        const double fraction = std::clamp(dt / .4, 0., 1.);
        for (std::size_t b = 0; b < 2; ++b) {
            for (std::size_t d = 0; d < 4; ++d) {
                warm[b][d] = (1 - fraction) * (*previous_plan_)[b][d] +
                             fraction * (*previous_plan_)[b + 1][d];
            }
        }
        for (std::size_t b = 0; b < 3 && !point_.enabled(); ++b) {
            warm[b][0] = seed[0];
            warm[b][1] = seed[1];
            warm[b][3] = best[b][3];
        }
        if (point_.enabled() && point_yaw_follows_seed_) {
            for (std::size_t b = 0; b < 3; ++b) {
                warm[b][3] = los_rate + (seed[3] - los_rate) * std::exp(-4 * .4 * b);  // ④
            }
        }
        const double value = score(warm);
        warm_plan = warm;
        warm_score = value;
        if (value < best_score) {
            best = warm;
            best_score = value;
            yaw_trace_.source = 2;
        }
    }
    yaw_trace_.after_init = best[0][3];
    // 수평은 현재 위치/속도 피드백을 지킨다. 시야 비용으로 제동이나 따라잡기를 미루지 않는다.
    // 수직 27개 coarse seed를 비교해 빠른 하강·상승 조합을 놓치지 않는다.
    std::array<double, 3> coarse_scores{std::numeric_limits<double>::infinity(),
                                        std::numeric_limits<double>::infinity(),
                                        std::numeric_limits<double>::infinity()};
    for (double vd0 : {-c.max_vertical_m_s, 0., c.max_vertical_m_s}) {
        for (double vd1 : {-c.max_vertical_m_s, 0., c.max_vertical_m_s}) {
            for (double vd2 : {-c.max_vertical_m_s, 0., c.max_vertical_m_s}) {
                auto trial = Plan{seed, seed, seed};
                for (std::size_t b = 0; b < 3; ++b) {
                    trial[b][3] = los_rate + (seed[3] - los_rate) * std::exp(-4 * .4 * b);
                }
                trial[0][2] = vd0;
                trial[1][2] = vd1;
                trial[2][2] = vd2;
                for (auto& u : trial) {
                    u = bound(u, yaw);
                }
                const double value = score(trial);
                const std::size_t seed_index = vd0 < 0 ? 0 : (vd0 > 0 ? 2 : 1);
                coarse_scores[seed_index] = std::min(coarse_scores[seed_index], value);
                if (value < best_score) {
                    best = trial;
                    best_score = value;
                }
            }
        }
    }
    yaw_trace_.after_coarse = best[0][3];
    // 고정 횟수의 coordinate search. 전역 최적성이나 NMPC solver 성공 보증은 아니다.
    // 수평 제동은 실제 FC 속도에 대한 예산 안에서 유지하고 수직 계획을 세밀하게 조정한다.
    // yaw는 LOS 피드백을 유지하며 시야 비용으로 옆으로 돌리지 않는다.
    for (double resolution : {.2, .1, .05}) {
        for (std::size_t block = 0; block < 3; ++block) {
            for (std::size_t dimension = point_.enabled() ? 0 : 2;
                 dimension < (point_.enabled() && !point_yaw_follows_seed_ ? 4 : 3); ++dimension) {
                const double delta = dimension == 3 ? resolution * 1.5 : resolution;
                for (double direction : {-1., 1.}) {
                    auto trial = best;
                    trial[block][dimension] += direction * delta;
                    trial[block] = bound(trial[block], yaw);
                    const double value = score(trial);
                    if (value < best_score) {
                        best = trial;
                        best_score = value;
                    }
                }
            }
        }
    }
    yaw_trace_.after_search = best[0][3];
    // nominal 측면 위치가 시야를 위협할 때만 작은 수평 이탈을 허용한다.
    // 같은 자세 응답/가속 경계로 평가하며 원 관측이나 목표축은 바꾸지 않는다.
    double visibility = 0;
    score(best, &visibility);
    if (visibility > .02) {
        const Plan nominal = best;
        for (double radial : {-.6, -.3, 0., .3, .6}) {
            for (double tangent : {-.3, 0., .3}) {
                auto trial = nominal;
                for (std::size_t b = 0; b < 3; ++b) {
                    const double fade = std::exp(-.4 * b);
                    trial[b][0] += fade * (-radial * dn - tangent * de) / range;
                    trial[b][1] += fade * (-radial * de + tangent * dn) / range;
                    trial[b] = bound(trial[b], yaw);
                }
                double risk = 0;
                const double value = score(trial, &risk);
                if (std::isfinite(value) &&
                    ((point_.enabled() && value < best_score) ||
                     (!point_.enabled() &&
                      (risk < visibility - 1e-4 ||
                       (std::abs(risk - visibility) <= 1e-4 && value < best_score))))) {
                    best = trial;
                    best_score = value;
                    visibility = risk;
                }
            }
        }
    }
    yaw_trace_.after_visibility = best[0][3];
    // V3 point 모드: 수평·수직 명령의 조합을 같은 운동/시야/거리/높이 비용으로 비교한다.
    if (point_.enabled()) {
        const Plan nominal = best;
        for (int start_block = 0; start_block < 3; ++start_block) {
            for (double radial : {-.6, 0., .6}) {
                for (double vertical : {-c.max_vertical_m_s, 0., c.max_vertical_m_s}) {
                    auto trial = nominal;
                    for (std::size_t b = 0; b < 3; ++b) {
                        const double fade = std::exp(-.4 * b);
                        if (static_cast<int>(b) >= start_block) {
                            trial[b][0] += fade * (-radial * dn) / range;
                            trial[b][1] += fade * (-radial * de) / range;
                        }
                        trial[b][2] += fade * vertical;
                        trial[b] = bound(trial[b], yaw);
                    }
                    double risk = 0;
                    const double value = score(trial, &risk);
                    if (std::isfinite(value) && value < best_score) {
                        best = trial;
                        best_score = value;
                        visibility = risk;
                    }
                }
            }
        }
    }
    yaw_trace_.after_point = best[0][3];
    // 2026-11-19 데모2: yaw 상한이 0이면 아래 yaw 후보는 모두 같은 계획이라 건너뛴다.
    if (contracting && visibility > .02 && yaw_limit > 0) {
        // 현재 fresh 실관측에서만 미리 회전한다. GAP 중 예측 표적 명령이나 TTL 연장이 아니다.
        const Plan nominal = best;
        const auto& reference = *visible_shape_;
        const double elapsed = shape_age + age * time_rate;
        const auto& los_target = point_.enabled() ? target : reference.target;
        const auto los_velocity =
            point_.enabled() ? std::array<double, 2>{0, 0} : reference.velocity;
        const double rn = los_target[0] + los_velocity[0] * elapsed - current.north_m;
        const double re = los_target[1] + los_velocity[1] * elapsed - current.east_m;
        // 2026-11-16: point 모드는 표적 속도 0이라 이 값이 위 los_rate와 같은 식(기체 속도
        // 기반)이다. 저역 필터를 켰으면 같은 필터 값을 쓴다. 꺼져 있으면 기존 계산 그대로.
        const double image_los_rate = point_.enabled() && los_rate_filter_s_ > 0
                                          ? los_rate
                                          : (rn * (los_velocity[1] - current.velocity_east_m_s) -
                                             re * (los_velocity[0] - current.velocity_north_m_s)) /
                                                std::max(.01, rn * rn + re * re);
        for (double yaw_candidate :
             {-c.max_yaw_rate_deg_s, -45., -30., -15., 0., 15., 30., 45., c.max_yaw_rate_deg_s}) {
            auto trial = nominal;
            for (std::size_t b = 0; b < 3; ++b) {
                // 시야를 확보한 뒤에는 기존 yaw P와 같은 감쇠로 영상 LOS 속도에 수렴한다.
                trial[b][3] =
                    image_los_rate + (yaw_candidate * rad - image_los_rate) * std::exp(-4 * .4 * b);
                trial[b] = bound(trial[b], yaw);
            }
            double risk = 0;
            const double value = score(trial, &risk);
            if (std::isfinite(value) &&
                ((point_.enabled() && value < best_score) ||
                 (!point_.enabled() &&
                  (risk < visibility - 1e-4 ||
                   (std::abs(risk - visibility) <= 1e-4 && value < best_score))))) {
                best = trial;
                best_score = value;
                visibility = risk;
            }
        }
    }
    // 시야 위험 후보(visibility > .02)를 고른 tick은 바꾸지 않는다.
    if (plan_hysteresis_ > 0 && warm_plan && std::isfinite(warm_score) && visibility <= .02 &&
        best_score > (1 - plan_hysteresis_) * warm_score) {
        best = *warm_plan;  // 2026-11-16 ①: 개선이 여유보다 작으면 직전 계획 유지
        best_score = warm_score;
    }
    yaw_trace_.after_contract = best[0][3];
    yaw_trace_.visibility = visibility;
    yaw_trace_.best_score = best_score;
    // 2026-11-09 작은 수직 속도는 0으로 보낸다. first_bound의 가속 제한으로 0까지 줄어든다.
    bool vertical_deadbanded = false;
    // 2026-11-16 높이 기준에서 많이 벗어났으면 기준 쪽 작은 수직 속도는 보낸다 (down 양수 = 하강).
    bool vertical_release = false;
    bool indoor_release = false;
    if (c.vertical_deadband_release_m > 0) {
        const double height_now = ground_datum - current.down_m;
        const double band = c.height_anchor_deadband_m + c.vertical_deadband_release_m;
        const bool indoor_anchor = height_anchored && c.target_height_m <= 0;
        indoor_release = indoor_anchor && ((height_now - height_ref > band && best[0][2] > 0) ||
                                           (height_now - height_ref < -band && best[0][2] < 0));
        // 상한 이상 하강 허용은 높은 쪽 기준 높이로 본다 (#5, 꺼짐이면 기존과 같다).
        vertical_release =
            indoor_release || (ceiling_datum - current.down_m >= c.max_height_m && best[0][2] > 0);
    }
    if (indoor_release && c.vertical_deadband_m_s > 0 && c.vertical_release_max_m_s > 0) {
        best[0][2] =
            std::clamp(best[0][2], -c.vertical_release_max_m_s, c.vertical_release_max_m_s);
    }
    if (point_.enabled() && c.vertical_deadband_m_s > 0 && !vertical_release &&
        std::abs(best[0][2]) < c.vertical_deadband_m_s) {
        vertical_deadbanded = best[0][2] != 0;
        best[0][2] = 0;
    }
    if (point_.enabled()) {
        best[0] = first_bound(best[0]);
    }
    if (!std::isfinite(best_score)) {
        return reject("plan_score_nonfinite");
    }
    CostTerms cost_terms;
    // 선택된 계획을 다시 평가한다 (판단 기록과 V3 영상 속도 진단이 마지막 후보를 설명하지 않게).
    const double diagnostic_score =
        score(best, nullptr, &cost_terms,
              image_velocity_weight_ > 0 ? &image_velocity_diagnostics_ : nullptr);
    if (image_velocity_weight_ > 0 && !std::isfinite(diagnostic_score)) {
        return reject("diagnostic_score_nonfinite");
    }
    // Preserve the already-computed coarse-grid best for each initial vertical seed. These
    // scores precede fine search and any optional horizontal/yaw visibility refinements.
    const auto candidate_score = [&](std::size_t index) -> std::optional<double> {
        const double value = coarse_scores[index];
        return std::isfinite(value) ? std::optional<double>{value} : std::nullopt;
    };
    const auto score_up = candidate_score(0);
    const auto score_zero = candidate_score(1);
    const auto score_down = candidate_score(2);
    previous_plan_ = best;
    auto command = first_bound(best[0]);
    if (c.sent_command_slew) {
        // 2026-11-19 #3: 송신 slew 검사와 조정은 PX4 가속 setpoint 모델이라
        // velocity_response(1.8)를 그대로 쓴다 (rollout 이득 아님).
        const auto physical =
            limit_horizontal_change((command[0] - current.velocity_north_m_s) * c.velocity_response,
                                    (command[1] - current.velocity_east_m_s) * c.velocity_response,
                                    current.velocity_north_m_s, current.velocity_east_m_s, yaw);
        const auto rate = limit_horizontal_change((command[0] - command_reference[0]) / command_dt,
                                                  (command[1] - command_reference[1]) / command_dt,
                                                  command_reference[0], command_reference[1], yaw);
        const bool physical_ok =
            std::abs(current.velocity_north_m_s + physical[0] / c.velocity_response - command[0]) <=
                1e-7 &&
            std::abs(current.velocity_east_m_s + physical[1] / c.velocity_response - command[1]) <=
                1e-7;
        const bool rate_ok =
            std::abs(command_reference[0] + rate[0] * command_dt - command[0]) <= 1e-7 &&
            std::abs(command_reference[1] + rate[1] * command_dt - command[1]) <= 1e-7;
        if (!physical_ok || !rate_ok) {
            if (!c.sent_command_slew_reconcile) {
                return reject(!physical_ok ? "sent_command_slew_physical_conflict"
                                           : "sent_command_slew_rate_conflict");
            }
            // 2026-11-10: 실제 FC 속도 기준 범위에서 직전 송신값에 가장 가까운 점을 향해,
            // 직전 송신값으로부터 변화율 한도만큼만 옮긴다. 명령이 튀지 않고 실제 속도 쪽으로
            // 수렴한다.
            const auto toward = limit_horizontal_change(
                (command_reference[0] - current.velocity_north_m_s) * c.velocity_response,
                (command_reference[1] - current.velocity_east_m_s) * c.velocity_response,
                current.velocity_north_m_s, current.velocity_east_m_s, yaw);
            const std::array<double, 2> nearest{
                current.velocity_north_m_s + toward[0] / c.velocity_response,
                current.velocity_east_m_s + toward[1] / c.velocity_response};
            const auto step =
                limit_horizontal_change((nearest[0] - command_reference[0]) / command_dt,
                                        (nearest[1] - command_reference[1]) / command_dt,
                                        command_reference[0], command_reference[1], yaw);
            Control reconciled = command;
            reconciled[0] = command_reference[0] + step[0] * command_dt;
            reconciled[1] = command_reference[1] + step[1] * command_dt;
            reconciled = bound(reconciled, yaw);
            if (!std::isfinite(reconciled[0] + reconciled[1])) {
                return reject("sent_command_slew_reconcile_nonfinite");
            }
            command[0] = reconciled[0];
            command[1] = reconciled[1];
            last_slew_reconciled_ = true;
        }
        pending_world_command_ = std::array<double, 2>{command[0], command[1]};
        pending_command_ns_ = command_now_ns;
    }

    const double limited_vertical = command[2];
    if (c.integral_gain > 0) {
        // I_{k+1}는 준비만 한다. 실제 송신이 성공했을 때만 complete_update(true)로 확정한다.
        integral_diagnostics_.seed_requested = {raw_seed[0], raw_seed[1]};
        integral_diagnostics_.seed_limited = {seed[0], seed[1]};
        integral_diagnostics_.first_requested = {best[0][0], best[0][1]};
        integral_diagnostics_.first_limited = {command[0], command[1]};
        for (unsigned axis = 0; axis < 2; ++axis) {
            const double residual = best[0][axis] - command[axis];
            const double increment = proposed_integral[axis] - integral_[axis];
            const double seed_residual = raw_seed[axis] - seed[axis];
            if (std::abs(residual) > 1e-12 || std::abs(seed_residual) > 1e-12) {
                integral_diagnostics_.saturated = true;
            }
            if (c.integral_anti_windup &&
                (increment * residual > 1e-12 || increment * seed_residual > 1e-12)) {
                proposed_integral[axis] = integral_[axis];
            }
        }
        integral_diagnostics_.proposed = proposed_integral;
        integral_diagnostics_.pending = true;
        pending_integral_ = proposed_integral;
    }
    if (yaw_rate_cap_deg_s_) {
        // 2026-11-10 GAP_CV 등에서 controller가 준 상한. 내부 yaw 기준도 같은 값으로 남긴다.
        command[3] =
            std::clamp(command[3], -*yaw_rate_cap_deg_s_ * rad, *yaw_rate_cap_deg_s_ * rad);
    }
    previous_yaw_rate_ = command[3];
    yaw_trace_.final = command[3];
    // 짧은 재획득에서 속도 적합이 대기 중이어도 실제 화면 가장자리 쪽으로 수직 이동하지 않는다.
    // 2026-11-19 #5: 가장자리 판단은 고른 기준, 높이 하한/상한은 보수적인 기준 높이를 쓴다.
    const double height = ground_datum - current.down_m;
    const double low_height = floor_datum - current.down_m;
    const double high_height = ceiling_datum - current.down_m;
    unsigned safety_flags = 0;
    if (box.y2 >= .85 * c.camera.image_height &&
        height - std::max(0., current.velocity_down_m_s) / 2 > edge_floor) {
        safety_flags |= 1;
        command[2] = std::max(0., command[2]);
    }
    if (box.y1 <= .15 * c.camera.image_height &&
        height + std::max(0., -current.velocity_down_m_s) / 2 < edge_ceiling) {
        safety_flags |= 2;
        command[2] = std::min(0., command[2]);
    }
    // soft 높이 비용보다 현재 잔류속도의 정지 높이를 우선한다. 실제 검증 범위 안의 여유를 둔다.
    if (low_height - std::max(0., current.velocity_down_m_s) / 2 <= c.min_height_m) {
        safety_flags |= 4;
        command[2] = std::min(0., command[2]);
    }
    if (high_height + std::max(0., -current.velocity_down_m_s) / 2 >= c.max_height_m) {
        safety_flags |= 8;
        command[2] = std::max(0., command[2]);
    }
    if (vertical_deadbanded) {
        safety_flags |= 16;  // 판단 기록용: 이번 계획의 작은 수직 속도를 무시했다
    }
    if (vertical_release && c.vertical_deadband_m_s > 0 &&
        std::abs(best[0][2]) < c.vertical_deadband_m_s) {
        safety_flags |= 32;  // 2026-11-16 판단 기록용: 높이 이탈로 작은 수직 속도를 그대로 보냈다
    }
    if (c.sent_command_slew) {
        pending_vertical_command_ = command[2];
        pending_vertical_ns_ = command_now_ns;
    } else {
        previous_vertical_command_ = command[2];
    }
    // 2026-11-19 판단 기록용 (동작에 쓰지 않는다): 누적 횟수, 영상 예측 1/3/5/10/20 단계, 계획
    // 블록.
    model.producer_ticks = producer_ground_ticks_;
    model.held_ticks = producer_ground_held_ticks_;
    model.height_ref_clamped_ticks = height_ref_clamped_ticks_;
    model.fallback_ticks = producer_ground_fallback_ticks_;
    if (image_velocity_weight_ > 0 && image_velocity_diagnostics_.steps >= 20) {
        constexpr std::array<std::size_t, 5> kForecastSteps{0, 2, 4, 9, 19};
        for (std::size_t i = 0; i < kForecastSteps.size(); ++i) {
            const auto& forecast = image_velocity_diagnostics_.forecast[kForecastSteps[i]];
            model.image_forecast[i] = {forecast[0], forecast[1]};
        }
        model.image_forecast_valid = true;
    }
    for (std::size_t b = 0; b < 3; ++b) {
        model.plan_blocks[b] = {best[b][0], best[b][1], best[b][2], best[b][3] * deg};
    }
    return PredictiveFollowResult{
        {cy * command[0] + sy * command[1], -sy * command[0] + cy * command[1], command[2],
         command[3] * deg},
        goal,
        ground_datum - height_ref,
        {best[0][2], limited_vertical, command[2], previous_vertical,
         c.sent_command_slew && (limited_vertical - previous_vertical) * previous_vertical < 0
             ? kVerticalBraking
             : (c.sent_command_slew ? kVerticalAcceleration : c.max_acceleration_m_s2),
         height_ref, best_score, cost_terms.height, cost_terms.visibility, cost_terms.boundary,
         diagnostic_score - cost_terms.height - cost_terms.visibility - cost_terms.boundary -
             cost_terms.center,
         score_up, score_zero, score_down, safety_flags, cost_terms.center, model}};
}
}  // namespace follow
