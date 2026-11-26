// 최종 송신 직전 만료·FC mode·명시 선택 epoch를 검사한다.
#include "argus/control/safety/include/flight_command_guard.hpp"

#include <cassert>
#include <chrono>
#include <vector>

using namespace follow;
using namespace std::chrono_literals;

namespace {
class FakeFlightState final : public IVehicleFlightStatusSource {
public:
    VehicleFlightStatus value;
    VehicleFlightStatus read() const override { return value; }
};

class FakeVehicle final : public follow::IVehiclePort {
public:
    std::vector<follow::FollowCommand> sent;
    Clock::time_point deadline;
    bool send_until(const FollowCommand& command, Clock::time_point valid_until) override {
        deadline = valid_until;
        return send(command);
    }
    bool send(const follow::FollowCommand& command) override {
        sent.push_back(command);
        return true;
    }
};
}  // namespace

int main() {
    TargetSelector selector;
    FakeFlightState flight;
    flight.value.connected = flight.value.armed = flight.value.in_air = flight.value.offboard =
        true;
    FakeVehicle vehicle;
    const auto now = Clock::now();
    TracksV1 frame;
    frame.source = "guard-fixture";
    frame.frame_id = 1;
    frame.sample_monotonic_ns = 1000000000;
    frame.clock_domain = "host_monotonic";
    frame.timestamp_source = "fixture";
    frame.image_width = 640;
    frame.image_height = 480;
    frame.generation = 3;
    frame.tracks.push_back(
        {8, 5, "rc_car", 0.9, 300, 120, 380, 240, "CONFIRMED", true, 1000000000});
    assert(selector.update(frame, now));
    assert(selector.execute({"guard-select", TargetCommandAction::Select, 3, 8}, now).accepted);
    assert(selector.execute({"guard-start", TargetCommandAction::Start}, now).accepted);

    FlightCommandGuard guard(selector, flight, vehicle);
    const auto selection = selector.snapshot(now);
    FlightGuidanceResult candidate;
    candidate.send_setpoint = true;
    candidate.valid_until = now + 20ms;
    candidate.selection_command_epoch = selection.command_epoch;
    candidate.command.vx_m_s = 0.2;
    assert(guard.send(candidate, selection, now + 1ms) == FlightCommandGuard::Result::Sent);
    assert(vehicle.sent.size() == 1 && vehicle.sent.front().vx_m_s == 0.2);
    assert(vehicle.deadline == candidate.valid_until);

    FlightCommandGuard yaw_guard(selector, flight, vehicle, true);
    for (int axis = 0; axis < 3; ++axis) {
        auto translated = candidate;
        translated.command = {};
        if (axis == 0) {
            translated.command.vx_m_s = 0.1;
        }
        if (axis == 1) {
            translated.command.vy_m_s = -0.1;
        }
        if (axis == 2) {
            translated.command.vz_m_s = 0.1;
        }
        assert(yaw_guard.send(translated, selection, now + 1ms) ==
               FlightCommandGuard::Result::Rejected);
    }
    auto yaw_candidate = candidate;
    yaw_candidate.command = {};
    yaw_candidate.command.yaw_rate_deg_s = 5;
    assert(yaw_guard.send(yaw_candidate, selection, now + 1ms) == FlightCommandGuard::Result::Sent);
    assert(vehicle.sent.size() == 2 && vehicle.sent.back().yaw_rate_deg_s == 5);

    auto expired = candidate;
    expired.valid_until = now + 1ms;
    assert(guard.send(expired, selection, now + 2ms) == FlightCommandGuard::Result::Rejected);
    flight.value.offboard = false;
    assert(guard.send(candidate, selection, now + 2ms) == FlightCommandGuard::Result::Rejected);
    flight.value.offboard = true;
    assert(selector.execute({"guard-stop", TargetCommandAction::Stop}, now + 2ms).accepted);
    assert(guard.send(candidate, selection, now + 2ms) ==
           FlightCommandGuard::Result::SelectionChanged);
    assert(vehicle.sent.size() == 2);
    return 0;
}
