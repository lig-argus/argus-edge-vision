// perception_api_bus 의 메시지(argus/perception_api_bus/message_types.hpp)를 ZmqBus 로 주고받기
// 위한 변환 표. 값 타입을 메시지 정의(argus_msg/*.proto)의 protobuf 본문으로 바꾼다.
// 규약: docs/spec/process_interfaces.md 3장, 5.1, 5.1.3, 5.1.5
// 사용: ZeroMQ 로 통신하는 실행 파일의 main() 이 ZmqBus 의 생성자에 넘긴다. 다른 모듈의 표와
// 합칠 때는 std::map 의 merge() 를 쓴다. CMake 타깃은 argus::perception_api_bus_zmq
//
// 변환 규칙
// - vision/frame/gray8: 헤더는 본문(VisionFrameGray8), 픽셀은 바이너리 프레임이다 (multipart
//   [이름][헤더][픽셀]). 본문의 seq 는 frame_id 와 같은 값이다 (vision_frame_gray8.proto). 그래서
//   encode 는 발행 번호 대신 frame_id 를 싣고, decode 는 seq 를 그대로 돌려준다. 픽셀 바이트 수가
//   stride × height 와 다르거나 stride 가 width 보다 작으면 decode 가 실패한다.
// - perception/observations, tracking/tracks, tracking/status: 발행 번호를 본문의 seq 에 싣는다.
// 바이너리가 있으면
//   decode 가 실패한다. 값 타입에 없는 필드(추론 시간, 발행 시각 등)는 읽지 않는다.
// - session_id 가 비어 있으면 값 타입에서는 값이 없는 것(nullopt)으로 바꾼다.
// - tracking/tracks 의 트랙 상태가 CONFIRMED, PREDICTED 가 아니면 decode 가 실패한다.
// - 본문을 읽을 수 없는 메시지는 decode 가 실패한다. 실패한 메시지를 어떻게 다루는지는 ZmqBus 의
//   규칙을 따른다 (argus/bus_zmq/zmq_bus.hpp).
#ifndef ARGUS_PERCEPTION_API_BUS_ZMQ_CODECS_HPP_
#define ARGUS_PERCEPTION_API_BUS_ZMQ_CODECS_HPP_

#include "argus/bus_zmq/zmq_codec.hpp"

namespace argus::perception_api_bus {

bus_zmq::ZmqCodecTable zmq_codecs();

}  // namespace argus::perception_api_bus

#endif  // ARGUS_PERCEPTION_API_BUS_ZMQ_CODECS_HPP_
