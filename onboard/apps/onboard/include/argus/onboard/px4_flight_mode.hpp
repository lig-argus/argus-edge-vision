// PX4 의 HEARTBEAT.custom_mode 해석 (docs/spec/icd.md 부록 A)
// 사용: FC 의 HEARTBEAT 를 받는 클래스
#ifndef ARGUS_ONBOARD_PX4_FLIGHT_MODE_HPP_
#define ARGUS_ONBOARD_PX4_FLIGHT_MODE_HPP_

#include <cstdint>

#include "argus/vehicle_api/types.hpp"

namespace argus::onboard {

// custom_mode 를 비행 모드로 변환한다. 부록 A 의 표에 없는 값은 Unknown
vehicle_api::FlightMode flight_mode_from_custom_mode(std::uint32_t custom_mode);

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_PX4_FLIGHT_MODE_HPP_
