// vehicle_api_bus 의 메시지(argus/vehicle_api_bus/message_types.hpp)를 ZmqBus 로 주고받기 위한
// 변환 표. 값 타입을 메시지 정의(argus_msg/*.proto)의 protobuf 본문으로 바꾼다.
// 규약: docs/spec/process_interfaces.md 3장, 5장
// 사용: ZeroMQ 로 통신하는 실행 파일의 main() 이 ZmqBus 의 생성자에 넘긴다. 다른 모듈의 표와
// 합칠 때는 std::map 의 merge() 를 쓴다. CMake 타깃은 argus::vehicle_api_bus_zmq
//
// 변환 규칙
// - 토픽 메시지: 발행 번호(sequence)를 본문의 seq 필드에 싣는다. 바이너리는 없다.
// - 본문을 읽을 수 없거나 바이너리가 있는 메시지는 decode 가 실패한다 (토픽 메시지, 요청, 응답
//   모두). 실패한 메시지를 어떻게 다루는지는 ZmqBus 의 규칙을 따른다 (argus/bus_zmq/zmq_bus.hpp).
// - 요청은 header 또는 action 이 없는 경우, follow/command 의 stop 은 reason 이 정해진 값이 아닌
//   경우도 decode 가 실패한다.
// - 응답은 id 가 없는 경우 decode 가 실패한다. follow/command 의 응답은 outcome 이 정해진 값이 아닌
//   경우도 실패한다.
#ifndef ARGUS_VEHICLE_API_BUS_ZMQ_CODECS_HPP_
#define ARGUS_VEHICLE_API_BUS_ZMQ_CODECS_HPP_

#include "argus/bus_zmq/zmq_codec.hpp"

namespace argus::vehicle_api_bus {

bus_zmq::ZmqCodecTable zmq_codecs();

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_ZMQ_CODECS_HPP_
