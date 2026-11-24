// FC 로 송신하는 Offboard 진입 명령(DO_SET_MODE)의 구성과 그 응답의 해석
// 규약: docs/spec/icd.md 4.2, 부록 A
// mavlink_gateway 의 구현 파일들이 함께 사용하며 모듈 밖에서는 include 하지 않는다
#ifndef ARGUS_ONBOARD_SRC_MODE_COMMAND_ENCODING_HPP_
#define ARGUS_ONBOARD_SRC_MODE_COMMAND_ENCODING_HPP_

#include <optional>

#include "argus/mavlink.h"

namespace argus::mavlink_gateway::detail {

// PX4 를 Offboard 로 전환하는 COMMAND_LONG. 만들 수 없는 경우 nullopt
std::optional<mavlink_message_t> encode_offboard_command();

// COMMAND_ACK 가 mavlink_gateway 가 송신한 DO_SET_MODE 의 최종 응답인 경우 수락 여부.
// 다른 명령의 응답, 다른 컴포넌트가 송신한 DO_SET_MODE 의 응답, 진행 중(IN_PROGRESS)은 nullopt.
// target 이 0 인 응답은 대상을 표시하지 않은 것이므로 사용한다
std::optional<bool> decode_mode_command_ack(const mavlink_message_t& message);

}  // namespace argus::mavlink_gateway::detail

#endif  // ARGUS_ONBOARD_SRC_MODE_COMMAND_ENCODING_HPP_
