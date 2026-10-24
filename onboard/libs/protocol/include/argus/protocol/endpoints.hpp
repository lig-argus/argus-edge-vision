// RPi 안 ZeroMQ 엔드포인트의 기본값. 실제 값은 각 실행 파일이 설정 파일에서 읽고, 없을 때만 이 값을
// 쓴다. 규약: docs/spec/process_interfaces.md 5.1, 5.2 엔드포인트 (모두 tcp://127.0.0.1) 토픽 연결
// 방식(직접 연결과 버스 프록시)의 통일은 같은 문서 7장 11 에서 정한다.
#ifndef ARGUS_PROTOCOL_ENDPOINTS_HPP_
#define ARGUS_PROTOCOL_ENDPOINTS_HPP_

#include <string_view>

namespace argus::protocol {

// PUB: fc/attitude, fc/state (onboard 가 bind)
inline constexpr std::string_view kDefaultFcStateEndpoint = "tcp://127.0.0.1:6370";
// REP: control/mode_request (onboard 가 bind)
inline constexpr std::string_view kDefaultControlModeRequestEndpoint = "tcp://127.0.0.1:6371";
// PUB: control/setpoint, follow/status (추종 제어가 bind)
inline constexpr std::string_view kDefaultControlStateEndpoint = "tcp://127.0.0.1:6372";
// REP: follow/command (추종 제어가 bind)
inline constexpr std::string_view kDefaultFollowCommandEndpoint = "tcp://127.0.0.1:6373";
// PUB: tracking/tracks, tracking/status (추적기가 bind)
inline constexpr std::string_view kDefaultTrackingEndpoint = "tcp://127.0.0.1:6357";
// 버스 프록시: 탐지 토픽. 발행자는 XSUB 5555 에, 구독자는 XPUB 5556 에 연결한다
inline constexpr std::string_view kDefaultVisionBusPublishEndpoint = "tcp://127.0.0.1:5555";
inline constexpr std::string_view kDefaultVisionBusSubscribeEndpoint = "tcp://127.0.0.1:5556";
// fc/mode_event 의 발행 엔드포인트는 규약에서 아직 정하지 않았다 (process_interfaces.md 7장 1)

}  // namespace argus::protocol

#endif  // ARGUS_PROTOCOL_ENDPOINTS_HPP_
