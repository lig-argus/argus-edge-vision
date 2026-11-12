#include "argus/onboard/fc_telemetry_receiver.hpp"

#include <cstdint>
#include <utility>

#include "argus/mavlink.h"
#include "argus/onboard/mavlink_ids.hpp"
#include "argus/onboard/px4_flight_mode.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::onboard {
namespace {

constexpr double kDegPerE7 = 1e-7;  // GLOBAL_POSITION_INT 의 lat, lon 단위(degE7)
constexpr double kMPerMm = 1e-3;    // GLOBAL_POSITION_INT 의 alt 단위(mm)
constexpr double kMPerCm = 1e-2;    // DISTANCE_SENSOR 의 current_distance 단위(cm)

}  // namespace

FcTelemetryReceiver::FcTelemetryReceiver(IMavlinkConnection& connection, FcLinkTracker& link,
                                         const IFcTimeConverter& time_converter)
    : link_(link), time_converter_(time_converter) {
    subscribe_fc_message(connection, MAVLINK_MSG_ID_ATTITUDE, &FcTelemetryReceiver::on_attitude);
    subscribe_fc_message(connection, MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                         &FcTelemetryReceiver::on_local_position);
    subscribe_fc_message(connection, MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                         &FcTelemetryReceiver::on_global_position);
    subscribe_fc_message(connection, MAVLINK_MSG_ID_DISTANCE_SENSOR,
                         &FcTelemetryReceiver::on_distance_sensor);
    subscribe_fc_message(connection, MAVLINK_MSG_ID_HEARTBEAT, &FcTelemetryReceiver::on_heartbeat);
    // 이전 fc_epoch 에서 받은 값을 새 fc_epoch 의 값과 함께 전달하지 않도록 삭제한다
    epoch_subscription_ = link_.subscribe_epoch_changes([this](std::uint64_t) { state_ = {}; });
}

void FcTelemetryReceiver::subscribe_fc_message(IMavlinkConnection& connection, std::uint32_t msgid,
                                               MessageHandler handler) {
    message_subscriptions_.push_back(
        connection.subscribe(msgid, [this, handler](const mavlink_message_t& message) {
            if (is_from_fc(message)) {
                (this->*handler)(message);
            }
        }));
}

core::Subscription FcTelemetryReceiver::subscribe_attitude(AttitudeHandler handler) {
    return attitude_handlers_.add(std::move(handler));
}

core::Subscription FcTelemetryReceiver::subscribe_state(StateHandler handler) {
    return state_handlers_.add(std::move(handler));
}

void FcTelemetryReceiver::on_attitude(const mavlink_message_t& message) {
    mavlink_attitude_t fc{};
    mavlink_msg_attitude_decode(&message, &fc);

    vehicle_api::Attitude attitude;
    attitude.received_monotonic_ns = link_.on_fc_message(FcTimedMessage::Attitude, fc.time_boot_ms);
    attitude.fc_epoch = link_.fc_epoch();
    attitude.roll_rad = static_cast<double>(fc.roll);
    attitude.pitch_rad = static_cast<double>(fc.pitch);
    attitude.yaw_rad = static_cast<double>(fc.yaw);
    attitude.roll_rate_rad_s = static_cast<double>(fc.rollspeed);
    attitude.pitch_rate_rad_s = static_cast<double>(fc.pitchspeed);
    attitude.yaw_rate_rad_s = static_cast<double>(fc.yawspeed);
    attitude.fc_time_boot_ms = fc.time_boot_ms;
    attitude.fc_time_monotonic_ns = time_converter_.to_monotonic_ns(fc.time_boot_ms);
    attitude.timesync_valid = attitude.fc_time_monotonic_ns.has_value();
    state_.attitude = attitude;
    attitude_handlers_.call(attitude);
}

void FcTelemetryReceiver::on_local_position(const mavlink_message_t& message) {
    mavlink_local_position_ned_t fc{};
    mavlink_msg_local_position_ned_decode(&message, &fc);

    vehicle_api::LocalPosition local;
    local.received_monotonic_ns =
        link_.on_fc_message(FcTimedMessage::LocalPosition, fc.time_boot_ms);
    local.fc_epoch = link_.fc_epoch();
    local.north_m = static_cast<double>(fc.x);
    local.east_m = static_cast<double>(fc.y);
    local.down_m = static_cast<double>(fc.z);
    local.vel_north_m_s = static_cast<double>(fc.vx);
    local.vel_east_m_s = static_cast<double>(fc.vy);
    local.vel_down_m_s = static_cast<double>(fc.vz);
    state_.local = local;
    state_handlers_.call(state_);
}

void FcTelemetryReceiver::on_global_position(const mavlink_message_t& message) {
    mavlink_global_position_int_t fc{};
    mavlink_msg_global_position_int_decode(&message, &fc);

    vehicle_api::GlobalPosition global;
    global.received_monotonic_ns =
        link_.on_fc_message(FcTimedMessage::GlobalPosition, fc.time_boot_ms);
    global.fc_epoch = link_.fc_epoch();
    global.lat_deg = static_cast<double>(fc.lat) * kDegPerE7;
    global.lon_deg = static_cast<double>(fc.lon) * kDegPerE7;
    global.alt_m = static_cast<double>(fc.alt) * kMPerMm;
    state_.global = global;
    state_handlers_.call(state_);
}

void FcTelemetryReceiver::on_distance_sensor(const mavlink_message_t& message) {
    mavlink_distance_sensor_t fc{};
    mavlink_msg_distance_sensor_decode(&message, &fc);
    // 아래 방향 센서의 값만 하향 거리로 사용한다. 다른 방향 센서의 메시지는 FC 연결 판단에만
    // 포함하며, 센서마다 time_boot_ms 가 다르므로 재부팅 감지에는 사용하지 않는다
    if (fc.orientation != MAV_SENSOR_ROTATION_PITCH_270) {
        link_.on_fc_message();
        return;
    }

    vehicle_api::DownwardDistance distance;
    distance.received_monotonic_ns =
        link_.on_fc_message(FcTimedMessage::DownwardDistance, fc.time_boot_ms);
    distance.fc_epoch = link_.fc_epoch();
    distance.distance_m = static_cast<double>(fc.current_distance) * kMPerCm;
    state_.downward_distance = distance;
    state_handlers_.call(state_);
}

void FcTelemetryReceiver::on_heartbeat(const mavlink_message_t& message) {
    mavlink_heartbeat_t fc{};
    mavlink_msg_heartbeat_decode(&message, &fc);

    vehicle_api::FlightStatus status;
    status.received_monotonic_ns = link_.on_fc_message();
    status.fc_epoch = link_.fc_epoch();
    status.mode = flight_mode_from_custom_mode(fc.custom_mode);
    status.armed = (fc.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    // in_air 는 HEARTBEAT 로 알 수 없다. 규약(icd.md 2장)에 비행 여부를 알리는 메시지가 없으므로
    // false 로 둔다
    status.in_air = false;
    state_.status = status;
    state_handlers_.call(state_);
}

}  // namespace argus::onboard
