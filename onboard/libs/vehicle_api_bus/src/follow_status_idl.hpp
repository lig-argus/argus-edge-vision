// follow/status IDL 타입과 vehicle_api 값 타입 사이의 변환.
// 생성된 IDL 타입(argus_msg/follow_status.idl 의 argus::idl::FollowStatus)을 사용하는 코드는 이
// 파일의 함수뿐이다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_STATUS_IDL_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_STATUS_IDL_HPP_

#include "argus/vehicle_api/types.hpp"
#include "argus_msg/follow_status.hpp"

namespace argus::vehicle_api_bus {

void to_idl_follow_status(const vehicle_api::FollowStatus& status, idl::FollowStatus& sample);
// 항상 true. enum 의 UNSPECIFIED 와 이 코드에 정의되지 않은 값은 Unknown 으로 바꾼다. outcome 이
// 정해진 값이 아닌 명령 결과는 어느 결과인지 판단할 수 없으므로 command_results 에 넣지 않는다
bool from_idl_follow_status(const idl::FollowStatus& sample, vehicle_api::FollowStatus& status);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_FOLLOW_STATUS_IDL_HPP_
