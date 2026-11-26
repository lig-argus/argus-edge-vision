#include "argus/gcs/link/telemetry_decoder.hpp"

#include <algorithm>
#include <cmath>

#include "argus/gcs/link/angles.hpp"
#include "argus/gcs/link/px4_mode.hpp"

namespace argus::gcs::link {

namespace {

constexpr float kRadToDeg = 57.29578F;
constexpr double kDegE7 = 1e7;           // GLOBAL_POSITION_INT.lat, lon 의 단위 (degE7)
constexpr float kMmPerM = 1000.0F;       // 고도 (mm)
constexpr float kMvPerV = 1000.0F;       // 배터리 전압 (mV)
constexpr float kCmPerM = 100.0F;        // 속도 (cm/s), 거리 (cm)
constexpr float kCdegPerDeg = 100.0F;    // 방향 (cdeg)
constexpr std::int32_t kPwmFloor = 900;  // PWM 출력은 이보다 작아지지 않는다 (DISARM 1000)

void decodeHeartbeat(FcState& fc, const mavlink_message_t& message) {
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&message, &heartbeat);
    fc.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0U;
    fc.mode = px4FlightMode(heartbeat.custom_mode);
}

void decodeEstimator(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_estimator_status_t estimator{};
    mavlink_msg_estimator_status_decode(&message, &estimator);
    fc.estimatorSeen = true;
    fc.estimatorFlags = estimator.flags;
    fc.estimatorNs = nowNs;
}

void decodeGlobalPosition(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_global_position_int_t position{};
    mavlink_msg_global_position_int_decode(&message, &position);
    fc.latDeg = position.lat / kDegE7;
    fc.lonDeg = position.lon / kDegE7;
    fc.relAltM = static_cast<float>(position.relative_alt) / kMmPerM;
    fc.altAmslM = static_cast<float>(position.alt) / kMmPerM;
    fc.globalPositionNs = nowNs;
    fc.groundSpeedMS = static_cast<float>(std::hypot(position.vx, position.vy)) / kCmPerM;
    fc.climbMS = -static_cast<float>(position.vz) / kCmPerM;
    fc.headingDeg =
        position.hdg == UINT16_MAX ? kUnknown : static_cast<float>(position.hdg) / kCdegPerDeg;
}

void decodeLocalPosition(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_local_position_ned_t local{};
    mavlink_msg_local_position_ned_decode(&message, &local);
    fc.localPositionNs = nowNs;
    fc.localZM = local.z;
    fc.localNorthM = local.x;
    fc.localEastM = local.y;
    // GPS 가 없는 실내에서는 GLOBAL_POSITION_INT 가 오지 않는다. 그때는 local 위치로 고도, 상승률,
    // 속도, 방향을 채운다. 고도는 위치 추정 원점(전원을 켠 자리) 기준이다
    if (nowNs - fc.globalPositionNs > kFcPositionStaleNs || !std::isfinite(fc.altAmslM)) {
        fc.relAltM = -local.z;
        fc.climbMS = -local.vz;
        fc.groundSpeedMS = std::hypot(local.vx, local.vy);
        if (std::isfinite(fc.yawDeg)) {
            fc.headingDeg = static_cast<float>(wrap360(fc.yawDeg));
        }
    }
}

void decodeAttitude(FcState& fc, const mavlink_message_t& message) {
    mavlink_attitude_t attitude{};
    mavlink_msg_attitude_decode(&message, &attitude);
    fc.rollDeg = attitude.roll * kRadToDeg;
    fc.pitchDeg = attitude.pitch * kRadToDeg;
    fc.yawDeg = attitude.yaw * kRadToDeg;
    fc.yawRateDegS = attitude.yawspeed * kRadToDeg;
}

void decodeDistance(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_distance_sensor_t distance{};
    mavlink_msg_distance_sensor_decode(&message, &distance);
    if (distance.orientation != MAV_SENSOR_ROTATION_PITCH_270) {
        return;  // 아래를 보는 센서만 지면까지 거리다
    }
    const bool inRange = distance.current_distance >= distance.min_distance &&
                         distance.current_distance <= distance.max_distance;
    fc.groundDistM = inRange ? static_cast<float>(distance.current_distance) / kCmPerM : kUnknown;
    fc.groundDistNs = nowNs;
}

void decodeSysStatus(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_sys_status_t status{};
    mavlink_msg_sys_status_decode(&message, &status);
    fc.batteryV = status.voltage_battery == UINT16_MAX
                      ? kUnknown
                      : static_cast<float>(status.voltage_battery) / kMvPerV;
    // battery_remaining 의 int8_t 는 문자가 아니라 숫자다 (-1 은 모름)
    // NOLINTNEXTLINE(bugprone-signed-char-misuse)
    fc.batteryPct = status.battery_remaining;
    fc.rcPresent =
        (status.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) != 0U;
    fc.rcHealthy =
        (status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) != 0U;
    fc.rcStatusNs = nowNs;
    fc.sensorsPresent = status.onboard_control_sensors_present;
    fc.sensorsEnabled = status.onboard_control_sensors_enabled;
    fc.sensorsHealth = status.onboard_control_sensors_health;
}

void decodeServoOutput(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_servo_output_raw_t servo{};
    mavlink_msg_servo_output_raw_decode(&message, &servo);
    if (servo.port != 0U) {
        return;  // 출력 9~16. 쿼드콥터 모터는 1~4 이다
    }
    fc.motorOutput = {servo.servo1_raw, servo.servo2_raw, servo.servo3_raw, servo.servo4_raw};
    // PWM 은 DISARM 중에도 1000 근처를 낸다. 그보다 훨씬 작은 값이 오면 0 부터 세는 방식이다
    fc.motorFromZero =
        fc.motorFromZero || std::any_of(fc.motorOutput.begin(), fc.motorOutput.end(),
                                        [](std::int32_t value) { return value < kPwmFloor; });
    fc.motorNs = nowNs;
}

void decodeRcChannels(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_rc_channels_t rc{};
    mavlink_msg_rc_channels_decode(&message, &rc);
    fc.rcChannels = rc.chancount;
    fc.rcRssi = rc.rssi == UINT8_MAX ? -1 : rc.rssi;
    fc.rcChannelsNs = nowNs;
}

void decodeGpsRaw(FcState& fc, const mavlink_message_t& message) {
    mavlink_gps_raw_int_t gps{};
    mavlink_msg_gps_raw_int_decode(&message, &gps);
    fc.gpsFix = gps.fix_type;
    fc.satellites = gps.satellites_visible == UINT8_MAX ? -1 : gps.satellites_visible;
}

void decodeExtendedState(FcState& fc, const mavlink_message_t& message) {
    mavlink_extended_sys_state_t state{};
    mavlink_msg_extended_sys_state_decode(&message, &state);
    fc.landedState = state.landed_state;
}

void decodeArgusStatus(GatewayStatus& gateway, const mavlink_message_t& message) {
    mavlink_argus_status_t status{};
    mavlink_msg_argus_status_decode(&message, &status);
    gateway.received = true;
    gateway.missionState = toMissionState(status.mission_state);
    gateway.targetState = toTargetState(status.target_state);
    gateway.activeSensor = status.active_sensor;
    gateway.targetId = status.target_id;
    gateway.targetGeneration = status.target_generation;
    gateway.health = status.health;
    gateway.controlSysid = status.control_sysid;
    gateway.cpuTempC = status.cpu_temp;
    // 스캔 값은 확장 필드라 보내지 않는 송신자의 값이 0 으로 복원된다 (icd 3.1). 0 을 "남은 시간
    // 0 초" 로 읽지 않도록 SCAN_ACTIVE 가 아니면 모름으로 둔다
    const bool scanning = gateway.missionState == MissionState::ScanActive;
    gateway.scanYawDoneRad = scanning ? status.scan_yaw_done_rad : kUnknown;
    gateway.scanRemainingS = scanning ? status.scan_remaining_s : kUnknown;
}

void decodeSensorStatus(VehicleState& state, const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_argus_sensor_status_t status{};
    mavlink_msg_argus_sensor_status_decode(&message, &status);
    SensorStatus& sensor = state.sensors[status.sensor_id];
    sensor.flags = status.flags;
    sensor.inferFps = status.infer_fps;
    sensor.modelVersion = status.model_ver;
    sensor.imageWidth = status.img_w;
    sensor.imageHeight = status.img_h;
    sensor.receivedNs = nowNs;
}

}  // namespace

bool decodeFcState(FcState& fc, const mavlink_message_t& message, std::int64_t nowNs) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            decodeHeartbeat(fc, message);
            return true;
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
            decodeEstimator(fc, message, nowNs);
            return true;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            decodeGlobalPosition(fc, message, nowNs);
            return true;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            decodeLocalPosition(fc, message, nowNs);
            return true;
        case MAVLINK_MSG_ID_ATTITUDE:
            decodeAttitude(fc, message);
            return true;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            decodeDistance(fc, message, nowNs);
            return true;
        case MAVLINK_MSG_ID_SYS_STATUS:
            decodeSysStatus(fc, message, nowNs);
            return true;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            decodeServoOutput(fc, message, nowNs);
            return true;
        case MAVLINK_MSG_ID_RC_CHANNELS:
            decodeRcChannels(fc, message, nowNs);
            return true;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            decodeGpsRaw(fc, message);
            return true;
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
            decodeExtendedState(fc, message);
            return true;
        default:
            return false;
    }
}

bool decodeGatewayState(VehicleState& state, const mavlink_message_t& message, std::int64_t nowNs) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_ARGUS_STATUS:
            decodeArgusStatus(state.gateway, message);
            return true;
        case MAVLINK_MSG_ID_ARGUS_SENSOR_STATUS:
            decodeSensorStatus(state, message, nowNs);
            return true;
        default:
            return false;
    }
}

MissionState toMissionState(std::uint8_t value) {
    if (value >= ARGUS_MISSION_STATE_ENUM_END) {
        return MissionState::Fault;
    }
    return static_cast<MissionState>(value);
}

TargetState toTargetState(std::uint8_t value) {
    if (value >= ARGUS_TARGET_STATE_ENUM_END) {
        return TargetState::None;
    }
    return static_cast<TargetState>(value);
}

}  // namespace argus::gcs::link
