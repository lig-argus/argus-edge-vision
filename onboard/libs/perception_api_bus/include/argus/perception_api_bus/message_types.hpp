// perception_api_bus 가 bus 로 받는 메시지의 타입 (bus::MessageType)
// 규약: docs/spec/process_interfaces.md 5.1
//
// 메시지 하나에 perception_api 의 값 타입 하나가 대응한다. 이름은 메시지 정의(argus_msg/*.proto)의
// 메시지 이름과 같다. 통신 구현은 값 타입으로 자기 변환 방법을 찾는다 (ZmqBus 는 zmq_codecs() 의
// 표)
#ifndef ARGUS_PERCEPTION_API_BUS_MESSAGE_TYPES_HPP_
#define ARGUS_PERCEPTION_API_BUS_MESSAGE_TYPES_HPP_

#include <string_view>

#include "argus/bus/message_type.hpp"
#include "argus/perception_api/types.hpp"

namespace argus::perception_api_bus {

// 이름은 "..."sv 로 쓴다. const char* 로 std::string_view 를 만드는 생성자는 noexcept 가 아니어서
// namespace 범위 상수의 초기화에 쓰면 정적 분석이 경고한다
using std::string_view_literals::operator""sv;

// vision/frame/gray8
inline const bus::MessageType kVisionFrameGray8Type =
    bus::message_type<perception_api::Frame>("argus.msg.VisionFrameGray8"sv);
// perception/observations
inline const bus::MessageType kPerceptionObservationsType =
    bus::message_type<perception_api::Observations>("argus.msg.PerceptionObservations"sv);
// tracking/tracks
inline const bus::MessageType kTrackingTracksType =
    bus::message_type<perception_api::Tracks>("argus.msg.TrackingTracks"sv);
// vision/status
inline const bus::MessageType kVisionStatusType =
    bus::message_type<perception_api::VisionStatus>("argus.msg.VisionStatus"sv);
// tracking/status
inline const bus::MessageType kTrackingStatusType =
    bus::message_type<perception_api::TrackingStatus>("argus.msg.TrackingStatus"sv);

}  // namespace argus::perception_api_bus

#endif  // ARGUS_PERCEPTION_API_BUS_MESSAGE_TYPES_HPP_
