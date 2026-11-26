// mavlink_gateway 가 사용하는 MAVLink 식별자와 통제기 연결 포트 (docs/spec/icd.md 1.1, 1.2)
#ifndef ARGUS_ONBOARD_MAVLINK_IDS_HPP_
#define ARGUS_ONBOARD_MAVLINK_IDS_HPP_

#include <cstdint>

#include "argus/mavlink.h"

namespace argus::mavlink_gateway {

inline constexpr std::uint8_t kVehicleSysid = 1;  // FC 와 mavlink_gateway 가 함께 사용
inline constexpr std::uint8_t kFcCompid = MAV_COMP_ID_AUTOPILOT1;
// telemetry 수신과 TIMESYNC 요청에 사용하는 mavlink_gateway 의 compid. 명령과 setpoint 는 이 compid
// 로 송신하지 않는다
inline constexpr std::uint8_t kFcObserverCompid = MAV_COMP_ID_ONBOARD_COMPUTER;
// 통제기 명령의 처리와 FC 로의 명령, setpoint 송신에 사용하는 mavlink_gateway 의 compid
inline constexpr std::uint8_t kOnboardCommandCompid = MAV_COMP_ID_PATHPLANNER;

// 통제기의 compid 와 sysid 범위. 이 값이 아닌 송신자는 통제기로 인정하지 않는다 (icd 5.1)
inline constexpr std::uint8_t kGcsCompid = MAV_COMP_ID_MISSIONPLANNER;
inline constexpr std::uint8_t kGcsSysidMin = 250;
inline constexpr std::uint8_t kGcsSysidMax = 253;

// 통제기와 MAVLink 를 주고받는 UDP 포트. mavlink_gateway 가 bind 한다 (icd 1.2)
inline constexpr std::uint16_t kGcsUdpPort = 14550;

inline bool is_gcs_sysid(std::uint8_t sysid) {
    return sysid >= kGcsSysidMin && sysid <= kGcsSysidMax;
}

// MAVLink 메시지를 만드는 함수(mavlink_msg_*_pack_chan)에 넘기는 채널. MAVLink 는 패킷 번호(seq)를
// 채널마다 따로 증가시키므로, compid 마다 채널을 나누어 수신자가 compid 별 seq 로 손실을 계산할 수
// 있게 한다 (docs/spec/icd.md 5장)
inline constexpr std::uint8_t kCommandChannel = MAVLINK_COMM_0;     // compid 195
inline constexpr std::uint8_t kFcObserverChannel = MAVLINK_COMM_1;  // compid 191

// message 가 FC 가 송신한 메시지인지 확인한다. mavlink-router 가 FC 가 아닌 노드의 메시지도 이
// 연결로 보낼 수 있으므로 sysid, compid 로 확인한다
inline bool is_from_fc(const mavlink_message_t& message) {
    return message.sysid == kVehicleSysid && message.compid == kFcCompid;
}

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_MAVLINK_IDS_HPP_
