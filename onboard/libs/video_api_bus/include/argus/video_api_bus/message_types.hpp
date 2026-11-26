// video_api_bus 가 bus 로 주고받는 메시지의 타입 (bus::MessageType).
// 메시지 하나에 C++ 값 타입 하나가 대응한다. 이름은 메시지 정의(argus_msg/*.proto)의 메시지 이름과
// 같다. 통신 구현은 값 타입으로 자기 변환 방법을 찾는다 (ZmqBus 는 zmq_codecs(), DdsBus 는
// dds_types() 의 표)
// 규약: docs/spec/process_interfaces.md 5장
#ifndef ARGUS_VIDEO_API_BUS_MESSAGE_TYPES_HPP_
#define ARGUS_VIDEO_API_BUS_MESSAGE_TYPES_HPP_

#include <string_view>

#include "argus/bus/message_type.hpp"
#include "argus/video_api/types.hpp"

namespace argus::video_api_bus {

// 이름은 "..."sv 로 쓴다. const char* 로 std::string_view 를 만드는 생성자는 noexcept 가 아니어서
// namespace 범위 상수의 초기화에 쓰면 정적 분석이 경고한다
using std::string_view_literals::operator""sv;

// video/stream_info
inline const bus::MessageType kVideoStreamInfoType =
    bus::message_type<video_api::VideoStreamInfo>("argus.msg.VideoStreamInfo"sv);

}  // namespace argus::video_api_bus

#endif  // ARGUS_VIDEO_API_BUS_MESSAGE_TYPES_HPP_
