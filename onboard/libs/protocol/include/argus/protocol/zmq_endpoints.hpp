// RPi 안 ZeroMQ 엔드포인트의 기본값. ZmqBus(#59)의 주소표를 만들 때 사용하며 bus 인터페이스에는
// 나오지 않는다 (주소는 구현별 설정. bus/bus.hpp). 실제 값은 각 실행 파일이 설정 파일에서 읽고,
// 없을 때만 이 값을 사용한다. 모든 토픽은 발행자가 bind 하고 구독자가 connect 한다 (직접 연결,
// 프록시 없음). 규약: docs/spec/process_interfaces.md 5.1, 5.2 엔드포인트 (모두 tcp://127.0.0.1)
#ifndef ARGUS_PROTOCOL_ZMQ_ENDPOINTS_HPP_
#define ARGUS_PROTOCOL_ZMQ_ENDPOINTS_HPP_

#include <string_view>

namespace argus::protocol {

// PUB: fc/attitude, fc/state, fc/mode_event (onboard 가 bind)
inline constexpr std::string_view kDefaultFcStateEndpoint = "tcp://127.0.0.1:6370";
// REP: control/mode_request (onboard 가 bind)
inline constexpr std::string_view kDefaultControlModeRequestEndpoint = "tcp://127.0.0.1:6371";
// PUB: control/setpoint, follow/status (추종 제어가 bind)
inline constexpr std::string_view kDefaultControlStateEndpoint = "tcp://127.0.0.1:6372";
// REP: follow/command (추종 제어가 bind)
inline constexpr std::string_view kDefaultFollowCommandEndpoint = "tcp://127.0.0.1:6373";
// PUB: vision/frame/gray8, vision/overlay/jpeg, vision/npu_input/jpeg, perception/observations
// (탐지의 C++ 어댑터가 bind)
inline constexpr std::string_view kDefaultVisionEndpoint = "tcp://127.0.0.1:6374";
// PUB: tracking/tracks, tracking/status (추적기가 bind)
inline constexpr std::string_view kDefaultTrackingEndpoint = "tcp://127.0.0.1:6375";

}  // namespace argus::protocol

#endif  // ARGUS_PROTOCOL_ZMQ_ENDPOINTS_HPP_
