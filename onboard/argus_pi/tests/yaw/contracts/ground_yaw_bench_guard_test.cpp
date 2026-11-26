#include "argus/control/safety/include/ground_yaw_bench_guard.hpp"

#include <cassert>
#include <limits>

int main() {
    using namespace follow;
    using namespace std::chrono_literals;
    assert(ground_yaw_bench_request_valid(true, 10, 5));
    assert(ground_yaw_bench_request_valid(true, -10, 1));
    assert(!ground_yaw_bench_request_valid(false, 5, 3));
    assert(!ground_yaw_bench_request_valid(true, 10.01, 3));
    assert(!ground_yaw_bench_request_valid(true, 1, 5.01));
    assert(!ground_yaw_bench_request_valid(true, 1, 0));
    assert(!ground_yaw_bench_request_valid(true, std::numeric_limits<double>::quiet_NaN(), 3));
    const auto now = Clock::now();
    Pose pose;
    pose.have_position = pose.have_attitude = true;
    pose.position_at = pose.attitude_at = now;
    VehicleFlightStatus state;
    state.connected = state.ground_known = state.landed = true;
    assert(ground_yaw_bench_state_valid(state, pose, now, 100ms));
    state.ground_known = false;
    assert(!ground_yaw_bench_state_valid(state, pose, now, 100ms));
    state.ground_known = true;
    state.landed = false;
    assert(!ground_yaw_bench_state_valid(state, pose, now, 100ms));
    state.landed = state.in_air = true;
    assert(!ground_yaw_bench_state_valid(state, pose, now, 100ms));
    state.in_air = false;
    state.offboard = true;
    assert(!ground_yaw_bench_state_valid(state, pose, now, 100ms));
    state.offboard = false;
    assert(!ground_yaw_bench_state_valid(state, pose, now + 101ms, 100ms));
    state.connected = false;
    assert(!ground_yaw_bench_state_valid(state, pose, now, 100ms));
}
