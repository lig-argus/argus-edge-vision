// 통제기 명령의 종류와 인수 (icd 4.2~4.4, 5.2). 종류마다 고정된 속성은 command_traits 가 맡는다
#ifndef ARGUS_GCS_LINK_COMMAND_HPP_
#define ARGUS_GCS_LINK_COMMAND_HPP_

#include <cstdint>
#include <limits>

#include "argus/gcs/link/vehicle_state.hpp"

namespace argus::gcs::link {

// 순서를 바꾸면 command_traits.cpp 의 표도 같이 바꾼다 (static_assert 로 확인)
enum class Command : std::uint8_t {
    // 비행 명령. FC 로 보내고 mavlink_gateway 가 검사한 뒤 전달한다 (icd 4.3, 5.2)
    Arm,
    Disarm,
    Takeoff,
    Land,
    EmergencyLand,  // 상단 막대의 비상 착륙. Land 와 같은 NAV_LAND 이고 ARM 상태에서만 보낸다
    Rtl,
    Hold,  // mavlink_gateway 가 제자리 유지로 처리한다 (icd 5.2)
    Stabilized,
    Altitude,
    Position,
    SetHome,   // 현재 위치를 Home 으로. 지상이고 시동이 꺼져 있을 때만 (icd 5.2)
    RebootFc,  // 지상이고 시동이 꺼져 있을 때만 (icd 5.2)

    // 추종, 스캔. mavlink_gateway 가 받는다 (icd 4.2)
    TrackStart,
    TrackStop,
    ScanStart,

    // 제어권. mavlink_gateway 가 받는다 (icd 4.4)
    RequestControl,
    ReleaseControl,
    EndRcOverride,  // 조종사(RC)에게서 통제기로 복귀 (icd 5.4)
};

inline constexpr std::size_t kCommandCount = static_cast<std::size_t>(Command::EndRcOverride) + 1;

// 가상 조종 입력 (MANUAL_CONTROL, icd 5.2). x 피치(+앞), y 롤(+오른쪽), r 요(+시계 방향)는
// -1000~1000, z 스로틀은 0~1000 (500 은 가운데: Altitude 모드에서 고도 유지)
struct ManualAxes {
    std::int16_t x{0};
    std::int16_t y{0};
    std::int16_t z{500};  // 가운데에서 시작 (켤 때 스틱을 움직인 것으로 보이지 않게)
    std::int16_t r{0};
};

struct CommandArgs {
    // 이륙 고도 범위와 간격 (입력 칸과 거부 판단이 같은 값을 쓴다)
    static constexpr float kTakeoffAltMinM = 0.1F;
    static constexpr float kTakeoffAltMaxM = 30.0F;
    static constexpr float kTakeoffAltStepM = 0.1F;

    float takeoffAltM{3.0F};

    // TRACK_START 의 표적 (icd 4.2). 운용자가 영상에서 고른 박스의 값
    TrackId trackId{0};
    std::uint32_t generation{0};
    std::uint32_t classId{0};
    std::uint8_t sensor{ARGUS_SENSOR_IR};
    bool targetSelected{false};  // 운용자가 표적을 골랐고 최근 탐지에 그 표적이 있다

    // SCAN_START 의 범위, 회전 속도, 제한 시간 (icd 4.2). NaN 이면 mavlink_gateway 의 기본값
    float scanRangeRad{std::numeric_limits<float>::quiet_NaN()};
    float scanRateRadS{std::numeric_limits<float>::quiet_NaN()};
    float scanTimeoutS{std::numeric_limits<float>::quiet_NaN()};
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_COMMAND_HPP_
