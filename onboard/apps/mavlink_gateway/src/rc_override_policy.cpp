#include "argus/mavlink_gateway/rc_override_policy.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>

namespace argus::mavlink_gateway {
namespace {

using vehicle_api::FlightMode;

constexpr std::uint16_t kUnusedChannel = std::numeric_limits<std::uint16_t>::max();

bool fresh(std::int64_t now_ns, std::int64_t received_ns, std::int64_t max_age_ns) {
    return now_ns - received_ns <= max_age_ns;
}

// RC 수신기와 추정 상태. 충족하면 None
ReturnBlock sensor_block(const ReturnObservation& observation, const RcOverrideConfig& config) {
    const bool health_fresh =
        observation.sensor_health.has_value() &&
        fresh(observation.now_ns, observation.sensor_health->received_monotonic_ns,
              config.sensor_max_age_ns);
    if (!health_fresh || !observation.sensor_health->rc_receiver_ok) {
        return ReturnBlock::RcReceiver;
    }
    if (!observation.sensor_health->local_position_ok || !observation.sensor_health->attitude_ok) {
        return ReturnBlock::Estimate;
    }
    return ReturnBlock::None;
}

}  // namespace

bool is_pilot_mode(FlightMode mode) {
    return mode == FlightMode::Position || mode == FlightMode::Altitude ||
           mode == FlightMode::Stabilized || mode == FlightMode::Manual;
}

std::optional<double> normalized_stick(std::uint16_t raw_us, const StickCalibration& calibration) {
    if (raw_us == 0 || raw_us == kUnusedChannel || calibration.min_us >= calibration.max_us) {
        return std::nullopt;
    }
    const double center = calibration.center_is_midpoint
                              ? (static_cast<double>(calibration.min_us) + calibration.max_us) / 2.0
                              : static_cast<double>(calibration.trim_us);
    const double min = calibration.min_us;
    const double max = calibration.max_us;
    if (center <= min || center >= max) {
        return std::nullopt;
    }
    const double raw = raw_us;
    const double value =
        raw >= center ? (raw - center) / (max - center) : (raw - center) / (center - min);
    return calibration.reversed ? -value : value;
}

bool sticks_neutral(const RcChannels& channels, const RcOverrideConfig& config) {
    if (channels.channel_count == 0) {
        return false;
    }
    for (const StickCalibration& stick : config.sticks) {
        if (stick.channel == 0 || stick.channel > channels.channel_count ||
            stick.channel > kRcChannelCount) {
            return false;
        }
        const std::optional<double> value =
            normalized_stick(channels.raw_us[static_cast<std::size_t>(stick.channel - 1)], stick);
        if (!value.has_value() || std::fabs(*value) > config.neutral_band) {
            return false;
        }
    }
    return true;
}

bool is_pilot_takeover(const ModeChange& change, const std::optional<GcsModeCommand>& gcs_command,
                       const RcOverrideConfig& config) {
    if (!change.armed || !is_pilot_mode(change.to)) {
        return false;
    }
    const bool by_gcs =
        gcs_command.has_value() && gcs_command->mode == change.to &&
        gcs_command->forwarded_ns <= change.received_ns &&
        change.received_ns - gcs_command->forwarded_ns <= config.gcs_mode_command_window_ns;
    return !by_gcs;
}

ReturnBlock check_return_conditions(const ReturnObservation& observation,
                                    const RcOverrideConfig& config) {
    if (!observation.pilot_in_control) {
        return ReturnBlock::NotPilot;
    }
    const std::int64_t now_ns = observation.now_ns;
    if (!observation.status.has_value() ||
        !fresh(now_ns, observation.status->received_monotonic_ns, config.status_max_age_ns) ||
        (observation.status->mode != FlightMode::Hold &&
         observation.status->mode != FlightMode::Position)) {
        return ReturnBlock::FcMode;
    }
    if (!observation.neutral_since_ns.has_value() ||
        now_ns - *observation.neutral_since_ns < config.neutral_hold_ns) {
        return ReturnBlock::Sticks;
    }
    if (const ReturnBlock block = sensor_block(observation, config); block != ReturnBlock::None) {
        return block;
    }
    if (observation.command_in_progress || observation.follow_in_progress ||
        observation.fc_entering) {
        return ReturnBlock::Busy;
    }
    return ReturnBlock::None;
}

bool return_procedure_allowed(const ReturnObservation& observation,
                              const RcOverrideConfig& config) {
    return observation.pilot_in_control && observation.neutral_since_ns.has_value() &&
           sensor_block(observation, config) == ReturnBlock::None;
}

std::string_view to_text(ReturnBlock block) {
    switch (block) {
        case ReturnBlock::None:
            return "";
        case ReturnBlock::NotPilot:
            return "조종사 개입 중 아님";
        case ReturnBlock::FcMode:
            return "FC 모드가 Hold 또는 Position 아님";
        case ReturnBlock::Sticks:
            return "스틱 중립 유지 시간 부족";
        case ReturnBlock::RcReceiver:
            return "RC 수신기 상태 이상";
        case ReturnBlock::Estimate:
            return "위치 또는 자세 추정 무효";
        case ReturnBlock::Busy:
            return "진행 중인 명령 또는 동작 있음";
    }
    return "";
}

}  // namespace argus::mavlink_gateway
