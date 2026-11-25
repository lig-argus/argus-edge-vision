// 여러 IDL 타입의 변환 함수가 함께 사용하는 것: 공통 타입(argus_msg/common.idl)의 변환.
// 라이브러리 밖에서는 include 하지 않는다
#ifndef ARGUS_VEHICLE_API_BUS_SRC_IDL_COMMON_HPP_
#define ARGUS_VEHICLE_API_BUS_SRC_IDL_COMMON_HPP_

#include "argus/vehicle_api/types.hpp"
#include "argus_msg/common.hpp"

namespace argus::vehicle_api_bus {

void to_idl(const vehicle_api::RequestId& id, idl::RequestId& sample);
vehicle_api::RequestId from_idl(const idl::RequestId& sample);

void to_idl(const vehicle_api::RequestHeader& header, idl::RequestHeader& sample);
vehicle_api::RequestHeader from_idl(const idl::RequestHeader& sample);

idl::FlightMode to_idl(vehicle_api::FlightMode mode);
// 이 코드에 정의되지 않은 값(표본을 만든 프로그램이 더 새 버전인 경우)은 Unknown 으로 바꾼다
vehicle_api::FlightMode from_idl(idl::FlightMode mode);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_SRC_IDL_COMMON_HPP_
