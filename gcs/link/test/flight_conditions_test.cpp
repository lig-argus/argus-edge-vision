// 비행 조건 (flight_conditions) 테스트: 링크, 제어권, 임무 상태, 정지 상태, 위치 추정, 지면 거리,
// 조종 입력, 센서
#include "argus/gcs/link/flight_conditions.hpp"

#include <QtTest>

#include "argus/mavlink.h"

namespace argus::gcs::link {
namespace {

constexpr std::int64_t kNow = 10 * kNsPerSecond;
constexpr std::int64_t kRecent = kNow - 100 * kNsPerMs;

void setEstimator(VehicleState& state, std::uint32_t flags) {
    state.fc.estimatorSeen = true;
    state.fc.estimatorFlags = flags;
    state.fc.estimatorNs = kRecent;
}

void setMission(VehicleState& state, MissionState mission) {
    state.gateway.received = true;
    state.gateway.missionState = mission;
}

}  // namespace

class FlightConditionsTest : public QObject {
    Q_OBJECT

private slots:
    void links() {
        VehicleState state;
        QVERIFY(!FlightConditions(state, kNow).fcLinkUp());
        state.fcLink.state = LinkState::Delayed;
        state.gatewayLink.state = LinkState::Lost;
        QVERIFY(FlightConditions(state, kNow).fcLinkUp());
        QVERIFY(!FlightConditions(state, kNow).gatewayLinkUp());
    }

    // control_sysid 를 이 통제기의 sysid 와 비교한다. ARGUS_STATUS 를 받기 전에는 모른다
    void control() {
        VehicleState state;
        state.gcsSysid = 251;
        state.gateway.controlSysid = 251;
        QVERIFY(!FlightConditions(state, kNow).hasControl());
        state.gateway.received = true;
        QVERIFY(FlightConditions(state, kNow).hasControl());
        state.gateway.controlSysid = 252;
        QVERIFY(FlightConditions(state, kNow).controlHeldByOther());
        state.gateway.controlSysid = 0;
        QVERIFY(!FlightConditions(state, kNow).controlHeldByOther());
        QVERIFY(!FlightConditions(state, kNow).hasControl());
    }

    void mission() {
        VehicleState state;
        QVERIFY(!FlightConditions(state, kNow).missionReady());
        setMission(state, MissionState::HoldWait);
        QVERIFY(FlightConditions(state, kNow).missionReady());
        QVERIFY(!FlightConditions(state, kNow).missionStoppable());
        setMission(state, MissionState::ScanActive);
        QVERIFY(FlightConditions(state, kNow).missionActive());
        QVERIFY(FlightConditions(state, kNow).gatewaySendingSetpoints());
        setMission(state, MissionState::Fault);
        QVERIFY(!FlightConditions(state, kNow).missionActive());
        QVERIFY(FlightConditions(state, kNow).missionStoppable());
        setMission(state, MissionState::Manual);
        QVERIFY(FlightConditions(state, kNow).pilotOverride());
        setMission(state, MissionState::Idle);
        state.fc.mode = FlightMode::Offboard;
        QVERIFY(FlightConditions(state, kNow).gatewaySendingSetpoints());
    }

    // 정지 상태: Hold 모드, 또는 OFFBOARD 이면서 HOLD_WAIT (icd 4.2)
    void stationary() {
        VehicleState state;
        state.fc.mode = FlightMode::Position;
        QVERIFY(!FlightConditions(state, kNow).stationary());
        state.fc.mode = FlightMode::Hold;
        QVERIFY(FlightConditions(state, kNow).stationary());
        state.fc.mode = FlightMode::Offboard;
        setMission(state, MissionState::FollowActive);
        QVERIFY(!FlightConditions(state, kNow).stationary());
        setMission(state, MissionState::HoldWait);
        QVERIFY(FlightConditions(state, kNow).stationary());
    }

    void groundAndAir() {
        VehicleState state;
        QVERIFY(FlightConditions(state, kNow).onGroundDisarmed());
        state.fc.landedState = MAV_LANDED_STATE_LANDING;
        QVERIFY(!FlightConditions(state, kNow).onGroundDisarmed());
        state.fc.landedState = MAV_LANDED_STATE_IN_AIR;
        QVERIFY(FlightConditions(state, kNow).inAir());
        state.fc.landedState = MAV_LANDED_STATE_ON_GROUND;
        state.fc.armed = true;
        QVERIFY(!FlightConditions(state, kNow).onGroundDisarmed());
    }

    // 수평 위치: EKF 가 위치원(GPS, optical flow)을 쓰며 추정할 때만. EKF 상태가 오래되면 전역
    // 위치로
    void horizontalPosition() {
        VehicleState state;
        QVERIFY(!FlightConditions(state, kNow).horizontalPositionKnown());
        setEstimator(state, ESTIMATOR_POS_HORIZ_REL);
        QVERIFY(!FlightConditions(state, kNow).horizontalPositionKnown());
        setEstimator(state, ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_PRED_POS_HORIZ_REL);
        QVERIFY(FlightConditions(state, kNow).horizontalPositionKnown());
        setEstimator(state, ESTIMATOR_POS_HORIZ_ABS);
        QVERIFY(FlightConditions(state, kNow).horizontalPositionKnown());
        state.fc.estimatorNs = kNow - FlightConditions::kPositionStaleNs - 1;
        state.fc.altAmslM = 100.0F;
        state.fc.globalPositionNs = kRecent;
        QVERIFY(FlightConditions(state, kNow).horizontalPositionKnown());
        state.fc.globalPositionNs = kNow - FlightConditions::kPositionStaleNs - 1;
        QVERIFY(!FlightConditions(state, kNow).positionKnown());
    }

    void groundDistance() {
        VehicleState state;
        QVERIFY(!FlightConditions(state, kNow).groundDistance());
        state.fc.groundDistM = 1.5F;
        state.fc.groundDistNs = kRecent;
        QCOMPARE(FlightConditions(state, kNow).groundDistance().value_or(0.0F), 1.5F);
        state.fc.groundDistNs = kNow - FlightConditions::kGroundDistStaleNs - 1;
        QVERIFY(!FlightConditions(state, kNow).groundDistance());
    }

    void manualInput() {
        VehicleState state;
        state.fc.mode = FlightMode::Altitude;
        QVERIFY(FlightConditions(state, kNow).manualMode());
        QVERIFY(!FlightConditions(state, kNow).manualInputReady());
        state.fc.rcPresent = true;
        state.fc.rcHealthy = true;
        state.fc.rcStatusNs = kRecent;
        QVERIFY(FlightConditions(state, kNow).manualInputReady());
        state.fc.rcStatusNs = kNow - FlightConditions::kRcStaleNs - 1;
        QVERIFY(!FlightConditions(state, kNow).manualInputReady());
        QVERIFY(!FlightConditions(state, kNow).rcReceiverActive());
        state.fc.rcChannels = 18;
        state.fc.rcChannelsNs = kRecent;
        QVERIFY(FlightConditions(state, kNow).rcReceiverActive());
        state.fc.mode = FlightMode::Hold;
        QVERIFY(!FlightConditions(state, kNow).manualMode());
    }

    void sensorFollowable() {
        VehicleState state;
        QVERIFY(!FlightConditions(state, kNow).sensorFollowable(ARGUS_SENSOR_IR));
        SensorStatus& ir = state.sensors[ARGUS_SENSOR_IR];
        ir.flags = ARGUS_SENSOR_FLAGS_CAMERA_OK;
        ir.receivedNs = kRecent;
        QVERIFY(!FlightConditions(state, kNow).sensorFollowable(ARGUS_SENSOR_IR));
        ir.flags |= ARGUS_SENSOR_FLAGS_FOLLOWABLE;
        QVERIFY(FlightConditions(state, kNow).sensorFollowable(ARGUS_SENSOR_IR));
        ir.receivedNs = kNow - FlightConditions::kSensorStaleNs - 1;
        QVERIFY(!FlightConditions(state, kNow).sensorFollowable(ARGUS_SENSOR_IR));
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::FlightConditionsTest)
#include "flight_conditions_test.moc"
