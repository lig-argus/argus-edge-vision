// PR33 role boundaries. No FC transport, authority acquisition, or clock rewriting.
#pragma once
#include "argus/runtime/contracts/include/i_follow_backend.hpp"

namespace follow {
class IVehicleState {
public:
    virtual ~IVehicleState() = default;
    virtual IVehicleStateSource& pose() = 0;
    virtual IVehicleFlightStatusSource& flight_state() = 0;
};
class IVelocitySetpoint : public IVehiclePort {
public:
    // Retain FollowCommand's existing deg/s boundary; wire adapters convert to rad/s.
    // Do not fall back to send(): an implementation must carry the original deadline.
    virtual bool send_until(const FollowCommand&, Clock::time_point) override = 0;
    virtual void revoke_commands() override = 0;
};
class IModeControl : public IFlightModePort {
public:
    // ACK state is independent of the actual mode returned by IVehicleState.
    virtual std::optional<bool> start_result() const override = 0;
    virtual std::optional<bool> hold_result() const override = 0;
    // Execution deadline only, NOT a promise of an onboard rotation watchdog.
    virtual bool request_heading(double yaw_rad, Clock::time_point valid_until, bool stopping) = 0;
};

class BackendVehicleState final : public IVehicleState {
public:
    explicit BackendVehicleState(IFollowBackend& backend) : backend_(backend) {}
    IVehicleStateSource& pose() override { return backend_.pose(); }
    IVehicleFlightStatusSource& flight_state() override { return backend_.flight_state(); }

private:
    IFollowBackend& backend_;
};
class BackendVelocitySetpoint final : public IVelocitySetpoint {
public:
    explicit BackendVelocitySetpoint(IFollowBackend& backend) : backend_(backend) {}
    bool send(const FollowCommand& command) override { return backend_.vehicle().send(command); }
    bool send_until(const FollowCommand& command, Clock::time_point until) override {
        return backend_.vehicle().send_until(command, until);
    }
    void revoke_commands() override { backend_.vehicle().revoke_commands(); }

private:
    IFollowBackend& backend_;
};
class BackendModeControl final : public IModeControl {
public:
    explicit BackendModeControl(IFollowBackend& backend) : backend_(backend) {}
    bool start_offboard() override { return backend_.mode().start_offboard(); }
    bool stop_offboard() override { return backend_.mode().stop_offboard(); }
    std::optional<bool> start_result() const override { return backend_.mode().start_result(); }
    std::optional<bool> hold_result() const override { return backend_.mode().hold_result(); }
    bool request_heading(double yaw_rad, Clock::time_point until, bool stopping) override {
        return backend_.send_heading(yaw_rad, until, stopping);
    }

private:
    IFollowBackend& backend_;
};
// PR45 vocabulary compatibility. These aliases do not change ABI or legacy units.
using IVehicleTelemetry = IVehicleState;
using IVelocitySetpointSink = IVelocitySetpoint;
using IFlightModeControl = IModeControl;
}  // namespace follow
