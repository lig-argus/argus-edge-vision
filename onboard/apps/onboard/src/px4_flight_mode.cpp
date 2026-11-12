#include "argus/onboard/px4_flight_mode.hpp"

#include <cstdint>

#include "argus/vehicle_api/types.hpp"

namespace argus::onboard {
namespace {

// PX4 custom_mode 의 주 모드와 부 모드 (icd.md 부록 A)
constexpr std::uint32_t kMainManual = 1;
constexpr std::uint32_t kMainAltitude = 2;
constexpr std::uint32_t kMainPosition = 3;
constexpr std::uint32_t kMainAuto = 4;
constexpr std::uint32_t kMainOffboard = 6;
constexpr std::uint32_t kMainStabilized = 7;
constexpr std::uint32_t kSubTakeoff = 2;
constexpr std::uint32_t kSubHold = 3;
constexpr std::uint32_t kSubMission = 4;
constexpr std::uint32_t kSubReturn = 5;
constexpr std::uint32_t kSubLand = 6;

vehicle_api::FlightMode auto_mode_from_sub_mode(std::uint32_t sub_mode) {
    switch (sub_mode) {
        case kSubTakeoff:
            return vehicle_api::FlightMode::Takeoff;
        case kSubHold:
            return vehicle_api::FlightMode::Hold;
        case kSubMission:
            return vehicle_api::FlightMode::Mission;
        case kSubReturn:
            return vehicle_api::FlightMode::Return;
        case kSubLand:
            return vehicle_api::FlightMode::Land;
        default:
            return vehicle_api::FlightMode::Unknown;
    }
}

}  // namespace

vehicle_api::FlightMode flight_mode_from_custom_mode(std::uint32_t custom_mode) {
    const std::uint32_t main_mode = (custom_mode >> 16U) & 0xFFU;
    const std::uint32_t sub_mode = (custom_mode >> 24U) & 0xFFU;
    switch (main_mode) {
        case kMainManual:
            return vehicle_api::FlightMode::Manual;
        case kMainAltitude:
            return vehicle_api::FlightMode::Altitude;
        case kMainPosition:
            return vehicle_api::FlightMode::Position;
        case kMainAuto:
            return auto_mode_from_sub_mode(sub_mode);
        case kMainOffboard:
            return vehicle_api::FlightMode::Offboard;
        case kMainStabilized:
            return vehicle_api::FlightMode::Stabilized;
        default:
            return vehicle_api::FlightMode::Unknown;
    }
}

}  // namespace argus::onboard
