#include <cassert>
#include <cmath>
#include <type_traits>

#include "argus/control/contracts/include/i_follow_command.hpp"
#include "argus/control/contracts/include/mode_request_identity.hpp"
#include "argus/runtime/contracts/include/follow_ports.hpp"
#include "argus/runtime/follow/include/follow_status_snapshot.hpp"
using namespace follow;
using namespace std::chrono_literals;
int main() {
    static_assert(std::is_same_v<IVehicleTelemetry, IVehicleState>);
    static_assert(std::is_same_v<IVelocitySetpointSink, IVelocitySetpoint>);
    static_assert(std::is_same_v<IFlightModeControl, IModeControl>);
    static_assert(std::is_same_v<IFollowControl, IFollowCommand>);
    const ModeRequestIdentity expected{"request-1", "controller-1", "onboard-1", "uid", 7};
    assert(expected.matches(expected));
    auto wrong = expected;
    wrong.command_id = "request-2";
    assert(!expected.matches(wrong));
    wrong = expected;
    wrong.requester_session = "controller-2";
    assert(!expected.matches(wrong));
    wrong = expected;
    wrong.onboard_session = "onboard-2";
    assert(!expected.matches(wrong));
    wrong = expected;
    wrong.uid = "other";
    assert(!expected.matches(wrong));
    wrong = expected;
    wrong.fc_epoch = 8;
    assert(!expected.matches(wrong));
    assert(!ModeRequestIdentity{}.matches(ModeRequestIdentity{}));
    for (double direction : {1., -1.}) {
        YawScanConfig cfg;
        cfg.rate_deg_s = direction * 10;
        YawScanController scan(cfg);
        auto now = Clock::time_point{10s};
        assert(!scan.remaining_seconds(now));
        scan.start(0, now, now, 1);
        assert(std::abs(*scan.remaining_seconds(now) - 36) < 1e-9);
        assert(!scan.remaining_seconds(now - 1ms));
        assert(!scan.remaining_seconds(now + 201ms));  // no fresh yaw: unknown, not zero remaining.
        now += 100ms;
        scan.update(direction, now, now, true, false, 2);
        assert(std::abs(*scan.remaining_seconds(now) - 35.9) < 1e-9);
        auto status = native_tracking_status(scan, TargetPhase::Locked, now);
        assert(status["scan_remaining_s"].is_number() && status["scan_yaw_done_rad"].is_number());
        // Display calculation never completes or moves a scan.
        const auto progress = scan.progress_deg(), heading = scan.heading_deg();
        for (int i = 0; i < 10; ++i) {
            scan.remaining_seconds(now);
        }
        assert(scan.active() && scan.progress_deg() == progress && scan.heading_deg() == heading);
        scan.stop(direction);
        assert(!scan.remaining_seconds(now));
        assert(native_tracking_status(scan, TargetPhase::Lost, now)["scan_remaining_s"].is_null());
    }
    YawScanController stalled;
    auto t = Clock::time_point{10s};
    stalled.start(0, t, t, 1);
    for (int i = 1; i <= 599; ++i) {
        auto n = t + i * 100ms;
        assert(stalled.update(0, n, n, true, false, i + 1));
    }
    assert(stalled.active() && std::abs(*stalled.remaining_seconds(t + 59900ms) - .1) < 1e-8);
    assert(stalled.remaining_seconds(t + 60s) == 0.0 && stalled.active());
    stalled.update(0, t + 60s, t + 60s, true, false, 601);
    assert(!stalled.active() && !stalled.remaining_seconds(t + 60s));
}
