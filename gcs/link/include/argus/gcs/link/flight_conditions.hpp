// 기체 상태로 판단하는 비행 조건: 위치 추정, 조종 입력, mavlink_gateway 의 임무 상태, 제어권, 센서.
// 명령 거부(CommandPolicy), 명령 만들기, 화면 안내가 같은 판단을 쓴다. 한 시각의 상태를 보고
// 판단하는 값 객체라 판단할 때마다 만든다
#ifndef ARGUS_GCS_LINK_FLIGHT_CONDITIONS_HPP_
#define ARGUS_GCS_LINK_FLIGHT_CONDITIONS_HPP_

#include <QString>
#include <cstdint>
#include <optional>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/gcs/link/vehicle_state.hpp"

namespace argus::gcs::link {

class FlightConditions {
public:
    // GLOBAL_POSITION_INT, LOCAL_POSITION_NED, ESTIMATOR_STATUS 가 이보다 오래되면 위치를 모른다
    static constexpr std::int64_t kPositionStaleNs = 2 * kNsPerSecond;
    static constexpr std::int64_t kGroundDistStaleNs = kNsPerSecond;  // 거리 센서 값

    // SYS_STATUS, RC_CHANNELS 가 이보다 오래되면 조종 입력 상태를 모른다 (수동 모드 차단)
    static constexpr std::int64_t kRcStaleNs = 3 * kNsPerSecond;

    // ARGUS_SENSOR_STATUS 가 이보다 오래되면 그 센서로 추종할 수 없다고 본다 (icd 6장의 두절 기준)
    static constexpr std::int64_t kSensorStaleNs = kLinkLostNs;

    FlightConditions(const VehicleState& state, std::int64_t nowNs);

    // 통신

    // 구성 요소와의 링크가 살아 있음 (Ok 또는 Delayed, icd 6장)
    bool fcLinkUp() const;
    bool gatewayLinkUp() const;

    // 제어권 (icd 5.1)

    // ARGUS_STATUS.control_sysid 가 이 통제기의 sysid
    bool hasControl() const;
    // ARGUS_STATUS 를 받았고 제어권을 다른 통제기가 가짐
    bool controlHeldByOther() const;

    // mavlink_gateway 의 임무 상태 (icd 4.2, 부록 B)

    // 명령 진행, 추종, 스캔 중 (PREPARING, FOLLOW_ACTIVE, HOLD_TRANSITION, SCAN_ACTIVE)
    bool missionActive() const;
    // 추종이나 스캔을 새로 시작할 수 있는 상태 (IDLE, HOLD_WAIT)
    bool missionReady() const;
    // TRACK_STOP 으로 끝낼 것이 있음. MANUAL, FAULT 의 복구도 TRACK_STOP 으로 한다 (icd 6장)
    bool missionStoppable() const;

    // mavlink_gateway 가 FC 로 setpoint 를 보내는 중. 임무 중이거나 FC 가 OFFBOARD 이면
    // mavlink_gateway 가 setpoint 를 계속 보낸다 (icd 4.2 정지 상태)
    bool gatewaySendingSetpoints() const;

    // 조종사(RC) 개입 중. ARGUS_STATUS 에는 개입 표시가 없으므로 mavlink_gateway 가 OFFBOARD 이탈을
    // 개입으로 판정한 결과인 MANUAL 로 본다 (icd 5.4, 부록 B)
    bool pilotOverride() const;

    // 정지 상태: FC 모드가 Hold 이거나, OFFBOARD 이면서 HOLD_WAIT (icd 4.2 정지 상태)
    bool stationary() const;

    // FC 상태

    bool inAir() const;  // EXTENDED_SYS_STATE.landed_state 가 IN_AIR
    // 지상이고 시동이 꺼져 있음 (icd 5.2 의 PARAM_SET, DO_SET_HOME, 재부팅 조건)
    bool onGroundDisarmed() const;
    // 최근 GLOBAL_POSITION_INT 있음. TAKEOFF, RTL 에 필요하다 (icd 5.2)
    bool positionKnown() const;
    // EKF 가 실제 위치원(GPS, optical flow)으로 수평 위치를 추정 중. Position 모드와 그 ARM 에 필요
    bool horizontalPositionKnown() const;
    // 지면까지 거리 (m). 거리 센서 값이 없거나 범위 밖이거나 오래되면 없음
    std::optional<float> groundDistance() const;
    // FC 가 조종 입력이 필요한 모드 (MANUAL, ALTCTL, POSCTL, ACRO, STABILIZED)
    bool manualMode() const;
    bool modeIs(FlightMode mode) const { return state_.fc.mode == mode; }

    // FC 가 조종 입력(RC 또는 가상 조종) 정상을 최근에 보고함
    bool manualInputReady() const;
    // RC 수신기가 최근 RC_CHANNELS 를 보냄. 송신기를 꺼도 수신기는 고정값을 계속 보내므로
    // 이것만으로 조종사가 조종 중인지는 알 수 없다
    bool rcReceiverActive() const;

    // 센서가 FOLLOWABLE 을 최근에 보고함 (icd 4.2 TRACK_START ③)
    bool sensorFollowable(std::uint8_t sensor) const;

private:
    const VehicleState& state_;
    std::int64_t nowNs_;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_FLIGHT_CONDITIONS_HPP_
