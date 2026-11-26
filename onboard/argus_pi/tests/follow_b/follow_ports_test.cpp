// No sockets/devices: preserve provenance, deadline, cancellation and async semantics.
#include "argus/runtime/contracts/include/follow_ports.hpp"

#include <cassert>
#include <functional>
using namespace follow;
struct State final : IVehicleStateSource {
    Pose value;
    mutable std::int64_t requested{0};
    Pose read() const override { return value; }
    std::optional<Pose> read_aligned(std::int64_t stamp, const std::string& domain,
                                     std::chrono::nanoseconds) const override {
        requested = stamp;
        if (domain != "host_monotonic" || stamp != value.attitude_monotonic_ns) {
            return std::nullopt;
        }
        return value;
    }
};
struct Flight final : IVehicleFlightStatusSource {
    VehicleFlightStatus value;
    VehicleFlightStatus read() const override { return value; }
};
struct Velocity final : IVehiclePort {
    std::function<bool()> guard = [] { return false; };
    FollowCommand last;
    Clock::time_point deadline{};
    int sends{0}, revokes{0};
    bool send(const FollowCommand& command) override {
        if (!guard()) {
            return false;
        }
        last = command;
        ++sends;
        return true;
    }
    bool send_until(const FollowCommand& command, Clock::time_point until) override {
        deadline = until;
        return send(command);
    }
    void revoke_commands() override { ++revokes; }
};
struct Mode final : IFlightModePort {
    std::optional<bool> start_ack{}, hold_ack{};
    bool permitted{false};
    int starts{0}, stops{0};
    bool start_offboard() override {
        ++starts;
        return permitted;
    }
    bool stop_offboard() override {
        ++stops;
        return permitted;
    }
    std::optional<bool> start_result() const override { return start_ack; }
    std::optional<bool> hold_result() const override { return hold_ack; }
};
struct Backend final : IFollowBackend {
    State state;
    Flight flight;
    Velocity velocity;
    Mode modes;
    bool owner{false};
    int acquisitions{0}, headings{0};
    double yaw{0};
    bool stopping{false};
    Clock::time_point deadline{};
    FollowCommandContext context;
    IVehicleStateSource& pose() override { return state; }
    IVehicleFlightStatusSource& flight_state() override { return flight; }
    IVehiclePort& vehicle() override { return velocity; }
    IFlightModePort& mode() override { return modes; }
    bool identity_permit() const override { return owner; }
    std::string verified_uid() const override { return "test-uid"; }
    bool acquire() override {
        ++acquisitions;
        return owner;
    }
    bool owns() const override { return owner; }
    void release() override { owner = false; }
    void set_writer_guard(std::function<bool()> fn) override { velocity.guard = std::move(fn); }
    void update_context(const FollowCommandContext& c) override { context = c; }
    bool send_heading(double y, Clock::time_point until, bool stop) override {
        if (!owner) {
            return false;
        }
        yaw = y;
        deadline = until;
        stopping = stop;
        ++headings;
        return true;
    }
};
int main() {
    Backend b;
    BackendVehicleState states{b};
    BackendVelocitySetpoint speeds{b};
    BackendModeControl modes{b};
    IVehicleState& state = states;
    IVelocitySetpoint& speed = speeds;
    IModeControl& mode = modes;
    b.state.value.attitude_monotonic_ns = 123456789;
    b.state.value.position_monotonic_ns = 123450000;
    assert(state.pose().read().attitude_monotonic_ns == 123456789);
    assert(state.pose().read_aligned(123456789, "host_monotonic", std::chrono::milliseconds(50)));
    assert(!state.pose().read_aligned(123456789, "unix", std::chrono::milliseconds(50)));
    assert(!state.pose().read_aligned(123456788, "host_monotonic", std::chrono::milliseconds(50)));
    const auto until = Clock::time_point{std::chrono::nanoseconds{123999999}};
    FollowCommand command{.5, -.25, .1, -12, "FOLLOW_ACTIVE"};
    b.update_context({7, 8, 9, 123456789, false});
    assert(!speed.send_until(command, until) && b.velocity.sends == 0);
    assert(!mode.request_heading(-1.2, until, false));
    b.owner = true;
    b.set_writer_guard([&] { return b.owner; });
    assert(speed.send_until(command, until));
    assert(b.velocity.deadline == until && b.velocity.last.yaw_rate_deg_s == -12);
    assert(b.velocity.last.vx_m_s == .5 && b.velocity.last.vy_m_s == -.25 &&
           b.velocity.last.vz_m_s == .1);
    assert(b.context.command_epoch == 7 && b.context.source_sample_monotonic_ns == 123456789);
    assert(!mode.start_result() && !mode.hold_result());
    b.modes.permitted = true;
    assert(mode.start_offboard());
    b.modes.start_ack = true;
    assert(mode.start_result() == true && !state.flight_state().read().offboard);
    b.modes.hold_ack = false;
    assert(mode.hold_result() == false);
    assert(mode.request_heading(-1.2, until, true));
    assert(b.yaw == -1.2 && b.stopping && b.deadline == until);
    speed.revoke_commands();
    assert(b.velocity.revokes == 1);
    b.owner = false;
    assert(!speed.send_until(command, until));
    assert(!mode.request_heading(0, until, true) && b.headings == 1);
    assert(b.acquisitions == 0);  // Adapters never acquire authority implicitly.
}
