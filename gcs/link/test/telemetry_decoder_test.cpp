// FC 상태 메시지(icd 2장)와 mavlink_gateway 상태 메시지(icd 3장)의 VehicleState 반영 테스트
#include "argus/gcs/link/telemetry_decoder.hpp"

#include <QtTest>
#include <cmath>
#include <cstdint>

namespace argus::gcs::link {
namespace {

constexpr std::int64_t kNow = 5 * kNsPerSecond;
constexpr float kTolerance = 1e-3F;
constexpr float kHalfPi = 1.5707963F;

bool near(float actual, float expected) { return std::fabs(actual - expected) < kTolerance; }

mavlink_message_t heartbeat(std::uint8_t baseMode, std::uint32_t customMode) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(kVehicleSysid, kFcCompid, &message, MAV_TYPE_QUADROTOR,
                               MAV_AUTOPILOT_PX4, baseMode, customMode, MAV_STATE_ACTIVE);
    return message;
}

mavlink_message_t globalPosition(std::int32_t altMm, std::int32_t relAltMm, std::uint16_t hdg) {
    mavlink_message_t message{};
    mavlink_msg_global_position_int_pack(kVehicleSysid, kFcCompid, &message, 0, 375'000'000,
                                         1'270'000'000, altMm, relAltMm, 300, 400, -50, hdg);
    return message;
}

mavlink_message_t localPosition(float z, float vz) {
    mavlink_message_t message{};
    mavlink_msg_local_position_ned_pack(kVehicleSysid, kFcCompid, &message, 0, 1.0F, 2.0F, z, 3.0F,
                                        4.0F, vz);
    return message;
}

mavlink_message_t attitude(float yawRad) {
    mavlink_message_t message{};
    mavlink_msg_attitude_pack(kVehicleSysid, kFcCompid, &message, 0, 0.1F, -0.2F, yawRad, 0.0F,
                              0.0F, 0.5F);
    return message;
}

mavlink_message_t distance(std::uint16_t currentCm, std::uint8_t orientation) {
    mavlink_message_t message{};
    mavlink_msg_distance_sensor_pack(kVehicleSysid, kFcCompid, &message, 0, 10, 1000, currentCm,
                                     MAV_DISTANCE_SENSOR_LASER, 0, orientation, 0, 0.0F, 0.0F,
                                     nullptr, 0);
    return message;
}

mavlink_message_t servoOutput(std::uint8_t port, std::uint16_t value) {
    mavlink_message_t message{};
    mavlink_msg_servo_output_raw_pack(kVehicleSysid, kFcCompid, &message, 0, port, value, value,
                                      value, value, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    return message;
}

mavlink_message_t argusStatus(std::uint8_t missionState, std::uint8_t targetState) {
    mavlink_message_t message{};
    mavlink_msg_argus_status_pack(kVehicleSysid, kGatewayCompid, &message, 1000, 55.5F, 12, 3,
                                  ARGUS_HEALTH_FC_CONNECTED, missionState, targetState,
                                  ARGUS_SENSOR_IR, kGcsSysidMin, 1.5F, 7.0F);
    return message;
}

}  // namespace

class TelemetryDecoderTest : public QObject {
    Q_OBJECT

private slots:
    void heartbeatSetsArmedAndMode() {
        FcState fc;
        const std::uint32_t offboard = 6U << 16U;
        QVERIFY(decodeFcState(fc, heartbeat(MAV_MODE_FLAG_SAFETY_ARMED, offboard), kNow));
        QVERIFY(fc.armed);
        QCOMPARE(fc.mode, FlightMode::Offboard);

        QVERIFY(decodeFcState(fc, heartbeat(0, 3U << 16U), kNow));
        QVERIFY(!fc.armed);
        QCOMPARE(fc.mode, FlightMode::Position);
    }

    void estimatorStatusSetsFlags() {
        FcState fc;
        mavlink_message_t message{};
        mavlink_msg_estimator_status_pack(kVehicleSysid, kFcCompid, &message, 0,
                                          ESTIMATOR_POS_HORIZ_ABS, 0, 0, 0, 0, 0, 0, 0, 0);
        QVERIFY(decodeFcState(fc, message, kNow));
        QVERIFY(fc.estimatorSeen);
        QCOMPARE(fc.estimatorFlags, std::uint32_t{ESTIMATOR_POS_HORIZ_ABS});
        QCOMPARE(fc.estimatorNs, kNow);
    }

    void globalPositionConvertsUnits() {
        FcState fc;
        QVERIFY(decodeFcState(fc, globalPosition(120'000, 15'000, 9000), kNow));
        QCOMPARE(fc.latDeg, 37.5);
        QCOMPARE(fc.lonDeg, 127.0);
        QVERIFY(near(fc.altAmslM, 120.0F));
        QVERIFY(near(fc.relAltM, 15.0F));
        QVERIFY(near(fc.groundSpeedMS, 5.0F));  // hypot(3, 4) m/s
        QVERIFY(near(fc.climbMS, 0.5F));        // vz 는 아래가 +
        QVERIFY(near(fc.headingDeg, 90.0F));
        QCOMPARE(fc.globalPositionNs, kNow);

        QVERIFY(decodeFcState(fc, globalPosition(120'000, 15'000, UINT16_MAX), kNow));
        QVERIFY(std::isnan(fc.headingDeg));
    }

    // 전역 위치가 없으면(실내) local 위치로 고도, 상승률, 속도, 방향을 채운다
    void localPositionFillsAltitudeWithoutGlobal() {
        FcState fc;
        QVERIFY(decodeFcState(fc, attitude(-kHalfPi), kNow));
        QVERIFY(decodeFcState(fc, localPosition(-2.0F, -0.3F), kNow));
        QCOMPARE(fc.localPositionNs, kNow);
        QVERIFY(near(fc.localZM, -2.0F));
        QVERIFY(near(fc.localNorthM, 1.0F));
        QVERIFY(near(fc.localEastM, 2.0F));
        QVERIFY(near(fc.relAltM, 2.0F));
        QVERIFY(near(fc.climbMS, 0.3F));
        QVERIFY(near(fc.groundSpeedMS, 5.0F));
        QVERIFY(near(fc.headingDeg, 270.0F));  // yaw -90° 를 0~360 으로
    }

    // 최근 전역 위치가 있으면 고도는 전역 위치의 값을 유지한다
    void localPositionKeepsFreshGlobalAltitude() {
        FcState fc;
        QVERIFY(decodeFcState(fc, globalPosition(120'000, 15'000, 9000), kNow));
        QVERIFY(decodeFcState(fc, localPosition(-2.0F, 0.0F), kNow + kFcPositionStaleNs));
        QVERIFY(near(fc.relAltM, 15.0F));
        QVERIFY(decodeFcState(fc, localPosition(-2.0F, 0.0F), kNow + kFcPositionStaleNs + 1));
        QVERIFY(near(fc.relAltM, 2.0F));
    }

    void attitudeConvertsToDegrees() {
        FcState fc;
        QVERIFY(decodeFcState(fc, attitude(1.0F), kNow));
        QVERIFY(near(fc.rollDeg, 5.7296F));
        QVERIFY(near(fc.pitchDeg, -11.4592F));
        QVERIFY(near(fc.yawDeg, 57.2958F));
        QVERIFY(near(fc.yawRateDegS, 28.6479F));
    }

    void distanceSensorUsesDownwardOnly() {
        FcState fc;
        QVERIFY(decodeFcState(fc, distance(250, MAV_SENSOR_ROTATION_NONE), kNow));
        QVERIFY(std::isnan(fc.groundDistM));
        QCOMPARE(fc.groundDistNs, std::int64_t{0});

        QVERIFY(decodeFcState(fc, distance(250, MAV_SENSOR_ROTATION_PITCH_270), kNow));
        QVERIFY(near(fc.groundDistM, 2.5F));
        QCOMPARE(fc.groundDistNs, kNow);

        // 측정 범위(10~1000 cm) 밖이면 모름
        QVERIFY(decodeFcState(fc, distance(5, MAV_SENSOR_ROTATION_PITCH_270), kNow));
        QVERIFY(std::isnan(fc.groundDistM));
    }

    void sysStatusSetsBatteryAndSensors() {
        FcState fc;
        const std::uint32_t present = MAV_SYS_STATUS_SENSOR_RC_RECEIVER | MAV_SYS_STATUS_SENSOR_GPS;
        const std::uint32_t health = MAV_SYS_STATUS_SENSOR_GPS;
        mavlink_message_t message{};
        mavlink_msg_sys_status_pack(kVehicleSysid, kFcCompid, &message, present, present, health, 0,
                                    15'800, 0, 76, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        QVERIFY(decodeFcState(fc, message, kNow));
        QVERIFY(near(fc.batteryV, 15.8F));
        QCOMPARE(fc.batteryPct, 76);
        QVERIFY(fc.rcPresent);
        QVERIFY(!fc.rcHealthy);
        QCOMPARE(fc.rcStatusNs, kNow);
        QCOMPARE(fc.sensorsPresent, present);
        QCOMPARE(fc.sensorsEnabled, present);
        QCOMPARE(fc.sensorsHealth, health);

        mavlink_msg_sys_status_pack(kVehicleSysid, kFcCompid, &message, 0, 0, 0, 0, UINT16_MAX, 0,
                                    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        QVERIFY(decodeFcState(fc, message, kNow));
        QVERIFY(std::isnan(fc.batteryV));
        QCOMPARE(fc.batteryPct, -1);
    }

    void servoOutputSetsMotorsFromPortZero() {
        FcState fc;
        QVERIFY(decodeFcState(fc, servoOutput(1, 1500), kNow));
        QCOMPARE(fc.motorOutput.at(0), 0);  // port 1 은 출력 9~16

        QVERIFY(decodeFcState(fc, servoOutput(0, 1000), kNow));
        QCOMPARE(fc.motorOutput.at(3), 1000);
        QVERIFY(!fc.motorFromZero);
        QCOMPARE(fc.motorNs, kNow);

        // 900 미만을 한 번이라도 받으면 0 부터 세는 방식으로 본다
        QVERIFY(decodeFcState(fc, servoOutput(0, 0), kNow));
        QVERIFY(decodeFcState(fc, servoOutput(0, 1200), kNow));
        QVERIFY(fc.motorFromZero);
    }

    void rcChannelsSetsCountAndRssi() {
        FcState fc;
        mavlink_message_t message{};
        mavlink_msg_rc_channels_pack(kVehicleSysid, kFcCompid, &message, 0, 8, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 200);
        QVERIFY(decodeFcState(fc, message, kNow));
        QCOMPARE(fc.rcChannels, 8);
        QCOMPARE(fc.rcRssi, 200);
        QCOMPARE(fc.rcChannelsNs, kNow);

        mavlink_msg_rc_channels_pack(kVehicleSysid, kFcCompid, &message, 0, 8, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, UINT8_MAX);
        QVERIFY(decodeFcState(fc, message, kNow));
        QCOMPARE(fc.rcRssi, -1);
    }

    void gpsRawSetsFixAndSatellites() {
        FcState fc;
        mavlink_message_t message{};
        mavlink_msg_gps_raw_int_pack(kVehicleSysid, kFcCompid, &message, 0, GPS_FIX_TYPE_3D_FIX, 0,
                                     0, 0, 0, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0);
        QVERIFY(decodeFcState(fc, message, kNow));
        QCOMPARE(fc.gpsFix, std::int32_t{GPS_FIX_TYPE_3D_FIX});
        QCOMPARE(fc.satellites, 11);

        mavlink_msg_gps_raw_int_pack(kVehicleSysid, kFcCompid, &message, 0, GPS_FIX_TYPE_NO_FIX, 0,
                                     0, 0, 0, 0, 0, 0, UINT8_MAX, 0, 0, 0, 0, 0, 0);
        QVERIFY(decodeFcState(fc, message, kNow));
        QCOMPARE(fc.satellites, -1);
    }

    void extendedStateSetsLandedState() {
        FcState fc;
        mavlink_message_t message{};
        mavlink_msg_extended_sys_state_pack(kVehicleSysid, kFcCompid, &message,
                                            MAV_VTOL_STATE_UNDEFINED, MAV_LANDED_STATE_IN_AIR);
        QVERIFY(decodeFcState(fc, message, kNow));
        QCOMPARE(fc.landedState, std::uint8_t{MAV_LANDED_STATE_IN_AIR});
    }

    void nonStateMessageIsNotDecoded() {
        FcState fc;
        mavlink_message_t message{};
        mavlink_msg_statustext_pack(kVehicleSysid, kFcCompid, &message, MAV_SEVERITY_INFO, "hello",
                                    0, 0);
        QVERIFY(!decodeFcState(fc, message, kNow));
        VehicleState state;
        QVERIFY(!decodeGatewayState(state, message, kNow));
        QVERIFY(!decodeGatewayState(state, heartbeat(0, 0), kNow));
        QVERIFY(!state.gateway.received);
    }

    void argusStatusSetsGatewayStatus() {
        VehicleState state;
        QVERIFY(decodeGatewayState(
            state, argusStatus(ARGUS_MISSION_STATE_FOLLOW_ACTIVE, ARGUS_TARGET_STATE_PREDICTED),
            kNow));
        const GatewayStatus& gateway = state.gateway;
        QVERIFY(gateway.received);
        QCOMPARE(gateway.missionState, MissionState::FollowActive);
        QCOMPARE(gateway.targetState, TargetState::Predicted);
        QCOMPARE(gateway.activeSensor, std::uint8_t{ARGUS_SENSOR_IR});
        QCOMPARE(gateway.targetId, TrackId{12});
        QCOMPARE(gateway.targetGeneration, std::uint32_t{3});
        QCOMPARE(gateway.health, std::uint8_t{ARGUS_HEALTH_FC_CONNECTED});
        QCOMPARE(gateway.controlSysid, kGcsSysidMin);
        QVERIFY(near(gateway.cpuTempC, 55.5F));
        // SCAN_ACTIVE 가 아니면 스캔 값은 모름
        QVERIFY(std::isnan(gateway.scanYawDoneRad));
        QVERIFY(std::isnan(gateway.scanRemainingS));
    }

    void argusStatusScanValuesOnlyWhenScanning() {
        VehicleState state;
        QVERIFY(decodeGatewayState(
            state, argusStatus(ARGUS_MISSION_STATE_SCAN_ACTIVE, ARGUS_TARGET_STATE_NONE), kNow));
        QCOMPARE(state.gateway.missionState, MissionState::ScanActive);
        QVERIFY(near(state.gateway.scanYawDoneRad, 1.5F));
        QVERIFY(near(state.gateway.scanRemainingS, 7.0F));
    }

    // 범위 밖 값: 임무 상태는 Fault, 표적 상태는 None
    void argusStatusOutOfRangeEnumsAreSafe() {
        VehicleState state;
        QVERIFY(decodeGatewayState(state, argusStatus(ARGUS_MISSION_STATE_ENUM_END, 200), kNow));
        QCOMPARE(state.gateway.missionState, MissionState::Fault);
        QCOMPARE(state.gateway.targetState, TargetState::None);
        QCOMPARE(toMissionState(ARGUS_MISSION_STATE_IDLE), MissionState::Idle);
        QCOMPARE(toMissionState(ARGUS_MISSION_STATE_FAULT), MissionState::Fault);
        QCOMPARE(toTargetState(ARGUS_TARGET_STATE_LOST), TargetState::Lost);
        QCOMPARE(toTargetState(ARGUS_TARGET_STATE_ENUM_END), TargetState::None);
    }

    void sensorStatusIsKeptPerSensor() {
        VehicleState state;
        mavlink_message_t message{};
        mavlink_msg_argus_sensor_status_pack(kVehicleSysid, kGatewayCompid, &message,
                                             ARGUS_SENSOR_IR, ARGUS_SENSOR_FLAGS_FOLLOWABLE, 14.5F,
                                             3, 640, 512);
        QVERIFY(decodeGatewayState(state, message, kNow));
        mavlink_msg_argus_sensor_status_pack(kVehicleSysid, kGatewayCompid, &message,
                                             ARGUS_SENSOR_EO, 0, kUnknown, 0, 0, 0);
        QVERIFY(decodeGatewayState(state, message, kNow + 1));

        QCOMPARE(state.sensors.size(), std::size_t{2});
        const SensorStatus& ir = state.sensors.at(ARGUS_SENSOR_IR);
        QCOMPARE(ir.flags, std::uint8_t{ARGUS_SENSOR_FLAGS_FOLLOWABLE});
        QVERIFY(near(ir.inferFps, 14.5F));
        QCOMPARE(ir.modelVersion, std::uint16_t{3});
        QCOMPARE(ir.imageWidth, std::uint16_t{640});
        QCOMPARE(ir.imageHeight, std::uint16_t{512});
        QCOMPARE(ir.receivedNs, kNow);
        const SensorStatus& eo = state.sensors.at(ARGUS_SENSOR_EO);
        QVERIFY(std::isnan(eo.inferFps));
        QCOMPARE(eo.receivedNs, kNow + 1);
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::TelemetryDecoderTest)
#include "telemetry_decoder_test.moc"
