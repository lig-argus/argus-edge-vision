// PX4 의 HEARTBEAT.custom_mode 해석 (docs/spec/icd.md 부록 A)
// 사용: FC 의 HEARTBEAT 를 받는 클래스
#ifndef ARGUS_ONBOARD_PX4_FLIGHT_MODE_HPP_
#define ARGUS_ONBOARD_PX4_FLIGHT_MODE_HPP_

#include <cstdint>
#include <optional>

#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// custom_mode 를 비행 모드로 변환한다. 부록 A 의 표에 없는 값은 Unknown
vehicle_api::FlightMode flight_mode_from_custom_mode(std::uint32_t custom_mode);

// MAV_CMD_DO_SET_MODE 의 param2(main_mode)와 param3(sub_mode)
struct Px4ModeParams {
    float main_mode = 0.0F;
    float sub_mode = 0.0F;
};

// mode 로 전환하는 MAV_CMD_DO_SET_MODE 의 매개변수. 부록 A 의 표에 없는 mode(Unknown)는 nullopt
std::optional<Px4ModeParams> px4_mode_params(vehicle_api::FlightMode mode);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_PX4_FLIGHT_MODE_HPP_
