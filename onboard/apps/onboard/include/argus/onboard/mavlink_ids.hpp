// onboard 가 사용하는 MAVLink 식별자 (docs/spec/icd.md 1.1)
#ifndef ARGUS_ONBOARD_MAVLINK_IDS_HPP_
#define ARGUS_ONBOARD_MAVLINK_IDS_HPP_

#include <cstdint>

#include "argus/mavlink.h"

namespace argus::onboard {

inline constexpr std::uint8_t kVehicleSysid = 1;  // FC 와 onboard 가 함께 사용
inline constexpr std::uint8_t kFcCompid = MAV_COMP_ID_AUTOPILOT1;
// telemetry 수신과 TIMESYNC 요청에 사용하는 onboard 의 compid. 명령과 setpoint 는 이 compid 로
// 송신하지 않는다
inline constexpr std::uint8_t kFcObserverCompid = MAV_COMP_ID_ONBOARD_COMPUTER;

// message 가 FC 가 송신한 메시지인지 확인한다. onboard 는 통제기의 메시지도 같은 UDP 포트로 받는다
inline bool is_from_fc(const mavlink_message_t& message) {
    return message.sysid == kVehicleSysid && message.compid == kFcCompid;
}

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_MAVLINK_IDS_HPP_
