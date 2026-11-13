#include "argus/onboard/px4_flight_mode.hpp"

#include <gtest/gtest.h>

#include <cstdint>

#include "argus/vehicle_api/types.hpp"

namespace argus::onboard {
namespace {

using vehicle_api::FlightMode;

constexpr std::uint32_t main_mode(std::uint32_t main) { return main << 16U; }
constexpr std::uint32_t auto_mode(std::uint32_t sub) { return (4U << 16U) | (sub << 24U); }

TEST(Px4FlightModeTest, ConvertsMainModes) {
    EXPECT_EQ(flight_mode_from_custom_mode(main_mode(1)), FlightMode::Manual);
    EXPECT_EQ(flight_mode_from_custom_mode(main_mode(2)), FlightMode::Altitude);
    EXPECT_EQ(flight_mode_from_custom_mode(main_mode(3)), FlightMode::Position);
    EXPECT_EQ(flight_mode_from_custom_mode(main_mode(6)), FlightMode::Offboard);
    EXPECT_EQ(flight_mode_from_custom_mode(main_mode(7)), FlightMode::Stabilized);
}

TEST(Px4FlightModeTest, ConvertsAutoSubModes) {
    EXPECT_EQ(flight_mode_from_custom_mode(auto_mode(2)), FlightMode::Takeoff);
    EXPECT_EQ(flight_mode_from_custom_mode(auto_mode(3)), FlightMode::Hold);
    EXPECT_EQ(flight_mode_from_custom_mode(auto_mode(4)), FlightMode::Mission);
    EXPECT_EQ(flight_mode_from_custom_mode(auto_mode(5)), FlightMode::Return);
    EXPECT_EQ(flight_mode_from_custom_mode(auto_mode(6)), FlightMode::Land);
}

TEST(Px4FlightModeTest, ValuesOutsideTableAreUnknown) {
    EXPECT_EQ(flight_mode_from_custom_mode(0), FlightMode::Unknown);
    EXPECT_EQ(flight_mode_from_custom_mode(main_mode(5)), FlightMode::Unknown);  // Acro
    EXPECT_EQ(flight_mode_from_custom_mode(auto_mode(1)), FlightMode::Unknown);  // Ready
    EXPECT_EQ(flight_mode_from_custom_mode(auto_mode(8)), FlightMode::Unknown);  // Follow Target
}

TEST(Px4FlightModeTest, IgnoresSubModeOutsideAuto) {
    EXPECT_EQ(flight_mode_from_custom_mode(main_mode(6) | (3U << 24U)), FlightMode::Offboard);
}

}  // namespace
}  // namespace argus::onboard
