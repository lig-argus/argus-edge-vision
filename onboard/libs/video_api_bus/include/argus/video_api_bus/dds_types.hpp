// video_api_bus 의 메시지(argus/video_api_bus/message_types.hpp)를 DdsBus 로 주고받기 위한 변환 표.
// 값 타입을 IDL 타입(onboard/libs/protocol/idl/argus_msg/video_stream_info.idl)의 표본으로 바꾼다.
// 규약: docs/spec/process_interfaces.md 3장, 5.1.6
// 사용: Fast DDS 로 통신하는 실행 파일의 main() 이 DdsBus::create() 에 넘긴다. 다른 모듈의 표와
// 합칠 때는 std::map 의 merge() 를 사용한다. CMake 타깃은 argus::video_api_bus_dds
//
// 변환 규칙
// - 발행 번호(sequence)는 표본에 싣지 않는다. DDS 가 표본마다 번호를 부여한다.
// - enum 의 UNSPECIFIED 와 이 코드에 정의되지 않은 값은 Unknown 으로 바꾼다.
// - 변환은 실패하지 않는다.
#ifndef ARGUS_VIDEO_API_BUS_DDS_TYPES_HPP_
#define ARGUS_VIDEO_API_BUS_DDS_TYPES_HPP_

#include "argus/bus_dds/dds_type_table.hpp"

namespace argus::video_api_bus {

bus_dds::DdsTypeTable dds_types();

}  // namespace argus::video_api_bus

#endif  // ARGUS_VIDEO_API_BUS_DDS_TYPES_HPP_
