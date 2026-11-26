#include "argus/mavlink_gateway/follow_command_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "argus/mavlink_gateway/follow_activity.hpp"

namespace argus::mavlink_gateway {
namespace {

constexpr double kPi = 3.14159265358979323846;

// value 가 min_value~kMaxCommandInteger 의 정수이면 그 값
std::optional<std::uint32_t> command_integer(float value, std::uint32_t min_value) {
    if (!std::isfinite(value) || std::trunc(value) != value) {
        return std::nullopt;
    }
    if (value < static_cast<float>(min_value) || value > static_cast<float>(kMaxCommandInteger)) {
        return std::nullopt;
    }
    return static_cast<std::uint32_t>(value);
}

CommandRejection denied(std::string reason) {
    return CommandRejection{MAV_RESULT_DENIED, std::move(reason)};
}

double value_or_default(float value, double default_value) {
    return std::isnan(value) ? default_value : static_cast<double>(value);
}

// 상한 limit 을 float 로 반올림한 값. 통제기가 float 로 보낸 상한값(2π 등)이 double 의 상한보다
// 조금 커져 범위 밖이 되지 않게 한다
double float_limit(double limit) { return static_cast<double>(static_cast<float>(limit)); }

}  // namespace

std::variant<TrackStartParams, CommandRejection> parse_track_start(
    const mavlink_command_long_t& command) {
    const std::optional<std::uint32_t> track_id = command_integer(command.param1, 1);
    if (!track_id.has_value()) {
        return denied("track_id 범위 밖");
    }
    const std::optional<std::uint32_t> class_id = command_integer(command.param3, 0);
    if (!class_id.has_value()) {
        return denied("class_id 범위 밖");
    }
    const std::optional<std::uint32_t> generation = command_integer(command.param4, 1);
    if (!generation.has_value()) {
        return denied("generation 범위 밖");
    }
    const std::optional<std::uint32_t> sensor = command_integer(command.param2, 0);
    if (!sensor.has_value() ||
        (sensor.value() != ARGUS_SENSOR_IR && sensor.value() != ARGUS_SENSOR_EO)) {
        return denied("센서 범위 밖");
    }
    TrackStartParams params;
    params.sensor = static_cast<ARGUS_SENSOR>(sensor.value());
    params.track_id = track_id.value();
    params.class_id = static_cast<std::int32_t>(class_id.value());
    params.generation = generation.value();
    return params;
}

std::optional<SensorSource> find_sensor(const SensorSourceMap& sensors, ARGUS_SENSOR sensor) {
    const auto& entries = sensors.entries();
    const auto found =
        std::find_if(entries.begin(), entries.end(),
                     [sensor](const SensorSource& entry) { return entry.sensor == sensor; });
    if (found == entries.end()) {
        return std::nullopt;
    }
    return *found;
}

std::variant<vehicle_api::TrackTarget, CommandRejection> select_track_target(
    const TrackSelection& selection) {
    if (!selection.followable) {
        return CommandRejection{MAV_RESULT_TEMPORARILY_REJECTED, "표적 지정 불가 센서 상태"};
    }
    if (!selection.tracks.has_value() ||
        selection.received_ns - selection.tracks->sample_monotonic_ns > kLatestTrackMaxAgeNs) {
        return denied("최신 추적 결과 없음");
    }
    const perception_api::Tracks& tracks = selection.tracks.value();
    const TrackStartParams& params = selection.params;
    if (tracks.generation != params.generation) {
        return denied("추적기 재시작 (generation 불일치)");
    }
    const auto found = std::find_if(tracks.tracks.begin(), tracks.tracks.end(),
                                    [&params](const perception_api::Track& track) {
                                        return track.track_id == params.track_id &&
                                               track.classification.class_id == params.class_id &&
                                               track.state == perception_api::TrackState::Confirmed;
                                    });
    if (found == tracks.tracks.end()) {
        return denied("표적 없음");
    }
    vehicle_api::TrackTarget target;
    target.sensor = static_cast<vehicle_api::Sensor>(params.sensor);
    target.generation = params.generation;
    target.class_id = params.class_id;
    target.track_id = params.track_id;
    target.label = found->classification.label;
    return target;
}

std::variant<vehicle_api::ScanRequest, CommandRejection> parse_scan_start(
    const mavlink_command_long_t& command) {
    const double yaw_range_rad = value_or_default(command.param1, kDefaultScanYawRangeRad);
    const double yaw_rate_rad_s = value_or_default(command.param2, kDefaultScanYawRateRadS);
    const double timeout_s = value_or_default(command.param3, kDefaultScanTimeoutS);
    if (!(yaw_range_rad > 0.0 && yaw_range_rad <= float_limit(kDefaultScanYawRangeRad))) {
        return denied("yaw 범위 밖");
    }
    if (!(yaw_rate_rad_s != 0.0 && std::fabs(yaw_rate_rad_s) <= float_limit(kMaxScanYawRateRadS))) {
        return denied("yaw 속도 범위 밖");
    }
    if (!(std::isfinite(timeout_s) && timeout_s > yaw_range_rad / std::fabs(yaw_rate_rad_s))) {
        return denied("제한 시간 범위 밖");
    }
    vehicle_api::ScanRequest scan;
    scan.yaw_range_rad = yaw_range_rad;
    scan.yaw_rate_rad_s = yaw_rate_rad_s;
    scan.timeout_s = timeout_s;
    return scan;
}

bool follow_stop_needed(const std::optional<vehicle_api::ReceivedFollowStatus>& latest,
                        std::int64_t now_ns) {
    if (!latest.has_value() || now_ns - latest->arrived_monotonic_ns > kFollowStatusTimeoutNs) {
        return false;
    }
    const vehicle_api::MissionState state = latest->status.mission_state;
    return state != vehicle_api::MissionState::Idle && state != vehicle_api::MissionState::HoldWait;
}

std::uint8_t scan_progress_percent(double progress_deg, double yaw_range_rad) {
    constexpr double kDegToRad = kPi / 180.0;
    constexpr double kPercent = 100.0;
    if (!(yaw_range_rad > 0.0) || !std::isfinite(progress_deg)) {
        return 0;
    }
    const double percent = progress_deg * kDegToRad / yaw_range_rad * kPercent;
    return static_cast<std::uint8_t>(std::clamp(std::round(percent), 0.0, kPercent));
}

}  // namespace argus::mavlink_gateway
