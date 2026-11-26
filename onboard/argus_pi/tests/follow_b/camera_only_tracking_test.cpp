#include <cassert>
#include <chrono>
#include <cmath>
#include <string>

#include "argus/control/safety/include/flight_command_guard.hpp"
#include "argus/perception/geometry/include/static_camera_projector.hpp"
#include "argus/tracking/contracts/include/tracks_v1_codec.hpp"
#include "argus/tracking/ucmc/include/ucmc_tracker_adapter.hpp"

using namespace follow;
using namespace std::chrono_literals;
namespace {
class Flight final : public IVehicleFlightStatusSource {
public:
    VehicleFlightStatus read() const override {
        VehicleFlightStatus state;
        state.connected = state.armed = state.in_air = state.offboard = true;
        return state;
    }
};
class Vehicle final : public IVehiclePort {
public:
    int sent{0};
    bool send(const FollowCommand&) override {
        ++sent;
        return true;
    }
};
}  // namespace

int main(int argc, char** argv) {
    assert(argc == 2);
    StaticCameraProjector projector(argv[1], 640, 480);
    const auto now = Clock::now();
    const auto stamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    NativeJson pixels{{"schema", "argus.perception.observations.v2"},
                      {"clock_domain", "host_monotonic"},
                      {"coordinate_frame", "camera_pixel"},
                      {"captured_monotonic_ns", stamp},
                      {"image_width", 640},
                      {"image_height", 480},
                      {"detections",
                       {{{"x1", 100},
                         {"y1", 200},
                         {"x2", 200},
                         {"y2", 300},
                         {"class_id", 2},
                         {"label", "car"},
                         {"confidence", .9}}}}};
    const auto frame = projector.project(pixels, stamp);
    assert(frame.detections.size() == 1);
    const auto& metric = frame.detections.front().metric;
    assert((metric.point - Eigen::Vector2d(-.34, .12)).norm() < 1e-10);
    assert((metric.covariance - Eigen::Matrix2d::Identity() * .0001).norm() < 1e-10);
    for (auto age : {-1LL, 250000000LL}) {
        bool rejected = false;
        try {
            (void)projector.project(pixels, stamp + age);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        assert(rejected);
    }
    auto bad = pixels;
    bad["image_width"] = 800;
    bool rejected = false;
    try {
        (void)projector.project(bad, stamp);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    assert(rejected);
    UcmcTrackerAdapter tracker(UcmcParameters{});
    std::int64_t identity = 0;
    for (int i = 0; i < 12; ++i) {
        const auto associations = tracker.update({metric}, 10. + i / 30.);
        for (const auto& association : associations) {
            if (!identity) {
                identity = association.id;
            }
            assert(identity == association.id);
        }
    }
    assert(identity > 0);
    TracksV1 preview;
    preview.source = "camera-only-test";
    preview.frame_id = 1;
    preview.sample_monotonic_ns = stamp;
    preview.clock_domain = "host_monotonic";
    preview.timestamp_source = kCameraOnlyTimestampSource;
    preview.image_width = 640;
    preview.image_height = 480;
    preview.generation = 1;
    preview.tracks.push_back(
        {identity, 2, "car", .9, 100, 200, 200, 300, "CONFIRMED", true, stamp});
    auto decoded = TracksV1Codec::decode(TracksV1Codec::encode(preview));
    assert(decoded.valid && camera_only_tracks(decoded.message));
    assert(!decoded.message.ground_pose && !decoded.message.fc_epoch);
    TargetSelector selector;
    assert(selector.update(decoded.message, now));
    assert(selector.execute({"select", TargetCommandAction::Select, 1, identity}, now).accepted);
    auto result = selector.execute({"start", TargetCommandAction::Start}, now);
    assert(!result.accepted && result.reason == "camera_only_tracking_not_flight_ready");
    TargetCommand atomic{"atomic", TargetCommandAction::SelectAndStart, 1, identity};
    atomic.class_id = 2;
    atomic.label = "car";
    atomic.sensor = preview.source;
    assert(!selector.execute(atomic, now).accepted);
    Flight flight;
    Vehicle vehicle;
    FlightCommandGuard guard(selector, flight, vehicle, true);
    auto selection = selector.snapshot(now);
    FlightGuidanceResult command;
    command.send_setpoint = true;
    command.selection_command_epoch = selection.command_epoch;
    command.valid_until = now + 100ms;
    command.command.yaw_rate_deg_s = -5;
    assert(guard.send(command, selection, now) == FlightCommandGuard::Result::Rejected);
    assert(vehicle.sent == 0);
    // A preview/control context switch invalidates an already selected target.
    TargetSelectorConfig config;
    config.recovery.enabled = true;
    TargetSelector transitions(config);
    auto flight_frame = preview;
    flight_frame.timestamp_source = "host_receive_fixture";
    assert(transitions.update(flight_frame, now));
    assert(transitions.execute({"normal-select", TargetCommandAction::Select, 1, identity}, now)
               .accepted);
    assert(transitions.execute({"normal-start", TargetCommandAction::Start}, now).accepted);
    auto fallback = preview;
    fallback.frame_id = 2;
    fallback.sample_monotonic_ns += 1;
    fallback.tracks[0].last_real_detection_monotonic_ns += 1;
    fallback.generation = 2;
    assert(!transitions.update(fallback, now + 1ns));
    assert(transitions.snapshot(now + 1ns).target_phase == TargetPhase::Lost);
    flight_frame.frame_id = 3;
    flight_frame.sample_monotonic_ns += 2;
    flight_frame.tracks[0].last_real_detection_monotonic_ns += 2;
    flight_frame.generation = 3;
    assert(transitions.update(flight_frame, now + 2ns));
    assert(!transitions.snapshot(now + 2ns).fresh_real_detection);
    return 0;
}
