// 화면에 넘기는 기체 상태 값 타입. MAVLink 구조체를 쓰지 않는다 (화면 코드가 MAVLink 를 모르게
// 한다). 시각은 모두 IClock 의 monotonic ns 이다. 값을 모르면 float 은 NaN, 정수는 -1 이다.
#ifndef ARGUS_GCS_LINK_VEHICLE_STATE_HPP_
#define ARGUS_GCS_LINK_VEHICLE_STATE_HPP_

#include <QString>
#include <array>
#include <cstdint>
#include <limits>
#include <map>

#include "argus/gcs/link/link_protocol.hpp"

namespace argus::gcs::link {

inline constexpr float kUnknown = std::numeric_limits<float>::quiet_NaN();

// 추적 번호. 추적기가 1~kMaxTrackId 로 붙이고 0 은 표적 없음이다.
// 상한 2^24 - 1 은 명령 매개변수(float32)로 정확히 보낼 수 있는 가장 큰 정수다 (icd 4.2)
using TrackId = std::uint32_t;
inline constexpr TrackId kMaxTrackId = 16'777'215;

// 구성 요소(FC, mavlink_gateway) 하나와의 링크 상태 (icd 6장)
enum class LinkState : std::uint8_t {
    Never,  // 받은 적 없음
    Ok,
    Delayed,  // 기대 주기의 두 배 동안 메시지 없음
    Lost,     // 3 s 동안 메시지 없음
};

struct ComponentLink {
    LinkState state{LinkState::Never};
    std::int32_t qualityPct{-1};  // 최근 10 s 의 HEARTBEAT 수신률 (%)
};

// PX4 비행 모드 (icd 부록 A). HEARTBEAT.custom_mode 의 main mode 와 AUTO 의 sub mode 를 하나로 둔다
enum class FlightMode : std::uint8_t {
    Unknown,  // 받은 적 없거나 부록 A 에 없는 값
    Manual,
    Altitude,
    Position,
    Acro,
    Offboard,
    Stabilized,
    Ready,  // AUTO.READY
    Takeoff,
    Hold,  // AUTO.LOITER
    Mission,
    Return,  // AUTO.RTL
    Land,
    FollowTarget,
    PrecisionLand,
};

// FC 상태 (icd 2장)
struct FcState {
    bool armed{false};
    FlightMode mode{FlightMode::Unknown};
    double latDeg{std::numeric_limits<double>::quiet_NaN()};
    double lonDeg{std::numeric_limits<double>::quiet_NaN()};
    float relAltM{kUnknown};
    float altAmslM{kUnknown};          // GLOBAL_POSITION_INT.alt (해발 고도). TAKEOFF p7 계산
    std::int64_t globalPositionNs{0};  // 마지막 GLOBAL_POSITION_INT 수신 시각

    // 마지막 LOCAL_POSITION_NED 수신 시각. 전역 위치가 없으면(실내) 고도와 속도를 local 위치로
    // 채운다
    std::int64_t localPositionNs{0};

    float localZM{kUnknown};  // LOCAL_POSITION_NED.z (아래가 +)

    // LOCAL_POSITION_NED.x, y: 위치 추정 원점(전원을 켠 자리) 기준 북, 동 (m). 지도 위치
    float localNorthM{kUnknown};
    float localEastM{kUnknown};

    // ESTIMATOR_STATUS.flags (ESTIMATOR_STATUS_FLAGS). Position 모드 가능 여부 판단
    bool estimatorSeen{false};
    std::uint32_t estimatorFlags{0};
    std::int64_t estimatorNs{0};

    float groundSpeedMS{kUnknown};
    float climbMS{kUnknown};
    float headingDeg{kUnknown};
    float rollDeg{kUnknown};
    float pitchDeg{kUnknown};
    float yawDeg{kUnknown};
    float yawRateDegS{kUnknown};  // ATTITUDE.yawspeed. + 는 위에서 볼 때 시계 방향

    // 아래를 보는 거리 센서로 잰 지면까지 거리 (DISTANCE_SENSOR, orientation PITCH_270).
    // 측정 범위 밖이면 모름. 실내(GPS 없음)에서는 고도 대신 이 값으로 높이를 본다
    float groundDistM{kUnknown};
    std::int64_t groundDistNs{0};

    float batteryV{kUnknown};
    std::int32_t batteryPct{-1};
    std::int32_t gpsFix{-1};  // GPS_FIX_TYPE
    std::int32_t satellites{-1};
    std::uint8_t landedState{MAV_LANDED_STATE_UNDEFINED};

    // 조종 입력 (SYS_STATUS 의 MAV_SYS_STATUS_SENSOR_RC_RECEIVER). PX4 는 RC 와 가상 조종
    // (MANUAL_CONTROL) 중 하나라도 있으면 켠다. 수동 모드에 필요하다
    bool rcPresent{false};
    bool rcHealthy{false};
    std::int64_t rcStatusNs{0};

    // SYS_STATUS 센서 비트 (MAV_SYS_STATUS_SENSOR). 기체 그림의 부위별 상태
    std::uint32_t sensorsPresent{0};
    std::uint32_t sensorsEnabled{0};
    std::uint32_t sensorsHealth{0};

    // 모터 출력 1~4 (SERVO_OUTPUT_RAW port 0). 받은 적 없으면 0. 단위는 출력 방식에 따른다:
    // PWM 은 us (1000~2000, DISARM 1000), DShot 은 0~1999, PX4 SITL 은 0~1000
    std::array<std::int32_t, 4> motorOutput{};
    bool motorFromZero{false};  // 900 미만을 한 번이라도 받음: 0 부터 세는 방식 (DShot, SITL)
    std::int64_t motorNs{0};

    // RC 수신기 (RC_CHANNELS). PX4 는 RC 수신기 입력으로만 보낸다 (가상 조종은 보내지 않음)
    std::int32_t rcChannels{-1};
    std::int32_t rcRssi{-1};  // 0~254. 255(모름)는 -1
    std::int64_t rcChannelsNs{0};
};

// 임무 상태 (icd 부록 B, ARGUS_MISSION_STATE)
enum class MissionState : std::uint8_t {
    Idle = ARGUS_MISSION_STATE_IDLE,
    Preparing = ARGUS_MISSION_STATE_PREPARING,
    FollowActive = ARGUS_MISSION_STATE_FOLLOW_ACTIVE,
    HoldTransition = ARGUS_MISSION_STATE_HOLD_TRANSITION,
    HoldWait = ARGUS_MISSION_STATE_HOLD_WAIT,
    ScanActive = ARGUS_MISSION_STATE_SCAN_ACTIVE,
    Manual = ARGUS_MISSION_STATE_MANUAL,
    Fault = ARGUS_MISSION_STATE_FAULT,
};

// 표적 상태 (icd 부록 B, ARGUS_TARGET_STATE)
enum class TargetState : std::uint8_t {
    None = ARGUS_TARGET_STATE_NONE,
    Confirmed = ARGUS_TARGET_STATE_CONFIRMED,
    Predicted = ARGUS_TARGET_STATE_PREDICTED,
    Lost = ARGUS_TARGET_STATE_LOST,
};

// mavlink_gateway 상태 (ARGUS_STATUS, icd 3장)
struct GatewayStatus {
    bool received{false};
    MissionState missionState{MissionState::Idle};
    TargetState targetState{TargetState::None};
    std::uint8_t activeSensor{ARGUS_SENSOR_UNKNOWN};
    TrackId targetId{0};
    std::uint32_t targetGeneration{0};  // 선택 표적의 generation. 없으면 0
    std::uint8_t health{0};             // ARGUS_HEALTH_FLAGS
    std::uint8_t controlSysid{0};       // 제어권을 가진 통제기. 없으면 0
    float cpuTempC{kUnknown};
    float scanYawDoneRad{kUnknown};  // SCAN_ACTIVE 일 때 회전한 yaw
    float scanRemainingS{kUnknown};  // SCAN_ACTIVE 일 때 남은 시간
};

// 센서 상태 (ARGUS_SENSOR_STATUS, icd 3장)
struct SensorStatus {
    std::uint8_t flags{0};  // ARGUS_SENSOR_FLAGS
    float inferFps{kUnknown};
    std::uint16_t modelVersion{0};
    std::uint16_t imageWidth{0};
    std::uint16_t imageHeight{0};
    std::int64_t receivedNs{0};
};

struct VehicleState {
    std::uint8_t gcsSysid{kGcsSysidDefault};  // 이 통제기의 sysid. 제어권 보유 판단
    FcState fc;
    ComponentLink fcLink;
    ComponentLink gatewayLink;
    bool routerSuspect{
        false};  // FC 와 mavlink_gateway 가 모두 두절: RPi 또는 라우터 이상 (icd 6장)
    GatewayStatus gateway;
    std::map<std::uint8_t, SensorStatus> sensors;  // 키: ARGUS_SENSOR
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_VEHICLE_STATE_HPP_
