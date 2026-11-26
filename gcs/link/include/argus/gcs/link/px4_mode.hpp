// PX4 HEARTBEAT.custom_mode 해석과 화면 이름 (icd 부록 A). 비트 16~23 이 main mode, 비트 24~31 이
// sub mode (main mode 가 AUTO 일 때)다
#ifndef ARGUS_GCS_LINK_PX4_MODE_HPP_
#define ARGUS_GCS_LINK_PX4_MODE_HPP_

#include <QString>
#include <cstdint>

#include "argus/gcs/link/vehicle_state.hpp"

namespace argus::gcs::link {

// custom_mode → 비행 모드. 부록 A 에 없는 값은 Unknown
FlightMode px4FlightMode(std::uint32_t customMode);

// 통제기 화면 이름 (부록 A: Manual, Stabilized, Altitude, Position, Offboard, Hold, Return, Land,
// Takeoff, Mission 등)
QString flightModeName(FlightMode mode);

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_PX4_MODE_HPP_
