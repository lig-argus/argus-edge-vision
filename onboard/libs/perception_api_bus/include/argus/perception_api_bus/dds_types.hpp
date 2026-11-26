// perception_api_bus 의 메시지(argus/perception_api_bus/message_types.hpp)를 DdsBus 로 주고받기
// 위한 변환 표. 값 타입을 IDL 타입(onboard/libs/protocol/idl/argus_msg/*.idl)의 표본으로 바꾼다.
// 규약: docs/spec/process_interfaces.md 3장, 5.1, 5.1.3, 5.1.5, 5.1.7
// 사용: Fast DDS 로 통신하는 실행 파일의 main() 이 DdsBus::create() 에 넘긴다. 다른 모듈의 표와
// 합칠 때는 std::map 의 merge() 를 사용한다. CMake 타깃은 argus::perception_api_bus_dds
//
// 변환 규칙
// - 발행 번호(sequence)는 표본에 싣지 않는다. DDS 가 표본마다 번호를 부여한다.
// - vision/frame/gray8: 픽셀은 표본의 pixels 멤버다. 픽셀 바이트 수가 stride × height 와 다르거나
//   stride 가 width 보다 작으면 변환이 실패한다.
// - 값 타입에 없는 필드(전처리 이름, 추론 시간, 발행 시각 등)는 보낼 때 기본값으로 두고 받을 때
//   읽지 않는다.
// - session_id 가 비어 있으면 값 타입에서는 값이 없는 것(nullopt)으로 바꾼다.
// - perception/observations 의 박스는 정수 픽셀로 반올림해 보낸다 (detection.idl).
// - tracking/tracks 의 트랙 상태가 CONFIRMED, PREDICTED 가 아니면 변환이 실패한다.
// - 변환에 실패한 표본을 어떻게 다루는지는 DdsBus 의 규칙을 따른다 (argus/bus_dds/dds_bus.hpp).
#ifndef ARGUS_PERCEPTION_API_BUS_DDS_TYPES_HPP_
#define ARGUS_PERCEPTION_API_BUS_DDS_TYPES_HPP_

#include "argus/bus_dds/dds_topic_qos.hpp"
#include "argus/bus_dds/dds_type_table.hpp"

namespace argus::perception_api_bus {

bus_dds::DdsTypeTable dds_types();

// 토픽별 발행자 QoS (process_interfaces.md 3장). 주기가 고정된 토픽에 Deadline(주기의 3배)
// - tracking/status, vision/status: 1 Hz → 3 s
// - tracking/tracks (탐지 결과 프레임마다), perception/observations, vision/frame/gray8 (최대 30
// FPS)
//   는 주기가 고정되지 않아 없음
bus_dds::DdsTopicQosTable dds_topic_qos();

}  // namespace argus::perception_api_bus

#endif  // ARGUS_PERCEPTION_API_BUS_DDS_TYPES_HPP_
