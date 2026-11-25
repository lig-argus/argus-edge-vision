// vehicle_api_bus 의 메시지(argus/vehicle_api_bus/message_types.hpp)를 DdsBus 로 주고받기 위한
// 변환 표. 값 타입을 IDL 타입(onboard/libs/protocol/idl/argus_msg/*.idl)의 표본으로 바꾼다.
// 규약: docs/spec/process_interfaces.md 3장, 5장
// 사용: Fast DDS 로 통신하는 실행 파일의 main() 이 DdsBus::create() 에 넘긴다. 다른 모듈의 표와
// 합칠 때는 std::map 의 merge() 를 사용한다. CMake 타깃은 argus::vehicle_api_bus_dds
//
// 변환 규칙
// - 발행 번호(sequence)는 표본에 싣지 않는다. DDS 가 표본마다 번호를 부여한다.
// - 요청은 action 이 정해진 값이 아닌 경우(control/mode_request 의 UNSPECIFIED, follow/command 의
//   정의되지 않은 판별 값), follow/command 의 stop 은 reason 이 정해진 값이 아닌 경우 변환이
//   실패한다.
// - 응답은 control/mode_request 의 id 가 없는 경우, follow/command 의 outcome 이 정해진 값이 아닌
//   경우 변환이 실패한다.
// - 변환에 실패한 표본을 어떻게 다루는지는 DdsBus 의 규칙을 따른다 (argus/bus_dds/dds_bus.hpp).
#ifndef ARGUS_VEHICLE_API_BUS_DDS_TYPES_HPP_
#define ARGUS_VEHICLE_API_BUS_DDS_TYPES_HPP_

#include "argus/bus_dds/dds_type_table.hpp"

namespace argus::vehicle_api_bus {

bus_dds::DdsTypeTable dds_types();

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_DDS_TYPES_HPP_
