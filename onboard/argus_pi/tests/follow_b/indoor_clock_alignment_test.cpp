#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "argus/control/target/include/target_manager.hpp"
#include "argus/perception/geometry/include/alignment_failure_window.hpp"
#include "argus/perception/geometry/include/device_clock_alignment.hpp"
#include "argus/perception/geometry/include/pi_ground_projector.hpp"

#define CHECK(value)                          \
    do {                                      \
        if (!(value)) {                       \
            throw std::runtime_error(#value); \
        }                                     \
    } while (false)

using namespace follow;

static void clock_contracts() {
    DeviceClockAlignment drifting;
    for (unsigned i = 0; i < 3600; ++i) {
        const auto receive = 10000000000LL + i * 33003300LL;
        drifting.observe(5000 + i * 33, receive);
        if (i > 100) {
            const auto estimate = drifting.map(5000 + i * 33);
            CHECK(estimate && receive - *estimate >= 0 && receive - *estimate < 1000000);
        }
    }
    DeviceClockAlignment clock;
    for (unsigned i = 0; i < 30; ++i) {
        clock.observe(5000 + i * 33, 10000000000LL + i * 33000000LL + 10000000 + (i % 3) * 1000000);
    }
    const auto mapped = clock.map(5957);
    CHECK(mapped);
    CHECK(std::abs(*mapped - 10967000000LL) < 1000000);
    clock.observe(5000, 12000000000LL);  // very late old packet: never re-anchor to now
    CHECK(clock.map(5957) == mapped);
    DeviceClockAlignment stopped;
    for (int i = 0; i < 30; ++i) {
        stopped.observe(0, 10000000000LL + i * 33000000LL);
    }
    CHECK(!stopped.map(0));
    DeviceClockAlignment wrap;
    for (unsigned i = 0; i < 30; ++i) {
        wrap.observe(std::uint32_t(UINT32_MAX - 300U + i * 33U), 10000000000LL + i * 33000000LL);
    }
    CHECK(wrap.map(std::uint32_t(UINT32_MAX - 300U + 29U * 33U)));
}

static TracksV1 sample(unsigned frame, double time, double vn, double ve) {
    TracksV1 message;
    message.schema = kTracksV2Schema;
    message.source = "deadzone-test";
    message.frame_id = frame;
    message.generation = 1;
    message.clock_domain = "host_monotonic";
    message.timestamp_source = "test-real-sample";
    message.sample_monotonic_ns = 1000000000LL + std::llround(time * 1e9);
    message.image_width = 640;
    message.image_height = 480;
    TrackRecordV1 car;
    car.track_id = 1;
    car.class_id = 0;
    car.label = "vehicle";
    car.confidence = .9;
    car.x1 = 290;
    car.x2 = 350;
    car.y1 = 200;
    car.y2 = 260;
    car.real_detection = true;
    car.state = "CONFIRMED";
    car.last_real_detection_monotonic_ns = message.sample_monotonic_ns;
    car.ground = std::array<double, 6>{3 + vn * time, ve * time, .05, 0, .05, 0};
    message.tracks = {car};
    return message;
}
static TargetMemory velocity(double vn, double ve, double threshold) {
    TargetRecoveryConfig config;
    config.enabled = true;
    config.short_motion.enabled = true;
    config.estimated_speed_deadzone_m_s = threshold;
    TargetManager manager(config);
    for (unsigned i = 0; i < 40; ++i) {
        const auto frame = sample(i + 1, i * .05, vn, ve);
        const auto now = Clock::time_point{std::chrono::nanoseconds{frame.sample_monotonic_ns}};
        if (i == 0) {
            manager.select(frame, frame.tracks[0], now);
        } else {
            manager.observe(frame, now, std::chrono::milliseconds{250});
        }
    }
    const auto now = Clock::time_point{std::chrono::nanoseconds{2950000000LL}};
    const auto view = manager.snapshot(now, std::chrono::milliseconds{250});
    CHECK(view.memory && view.phase == TargetPhase::Locked);
    return *view.memory;
}
static void deadzone_contracts() {
    auto small = velocity(.03, 0, .05);
    CHECK(small.velocity_north_m_s == 0 && small.velocity_east_m_s == 0);
    auto baseline = velocity(.03, 0, 0);
    CHECK(baseline.velocity_north_m_s > .02);
    auto negative = velocity(-.10, 0, .05);
    CHECK(negative.velocity_north_m_s < -.09);
    auto diagonal = velocity(.04, .04, .05);
    CHECK(diagonal.velocity_north_m_s > .03 && diagonal.velocity_east_m_s > .03);
}
// 2026-11-12 임시 카메라 자세 보정: 꺼지면 기존과 같고, 켜면 지면 점만 바뀌며 발행 pose는 같다.
static void attitude_correction_contracts(const char* camera, const char* settings) {
    const auto calibration = read_native_json(camera);
    const auto geometry = calibration.at("geometry");
    const std::string uid = calibration.at("expected_uid");
    auto measurement = read_native_json(settings);
    measurement["navigation_policy"] = "optical_flow";
    measurement["sdk_time_alignment"] = true;
    PiGroundProjector base(geometry, measurement, uid);
    PiGroundProjector explicit_off(geometry, measurement, uid);
    PiGroundProjector scaled(geometry, measurement, uid);
    PiGroundProjector shifted(geometry, measurement, uid);
    explicit_off.set_attitude_correction(1., 0);
    scaled.set_attitude_correction(1.4, 0);
    shifted.set_attitude_correction(1., -50000000LL);
    CHECK(!base.attitude_correction_active() && !explicit_off.attitude_correction_active());
    CHECK(scaled.attitude_correction_active() && shifted.attitude_correction_active());
    for (const double bad : {.4, 2.1, std::nan("")}) {
        bool thrown = false;
        try {
            scaled.set_attitude_correction(bad, 0);
        } catch (const std::invalid_argument&) {
            thrown = true;
        }
        CHECK(thrown);
    }
    bool thrown = false;
    try {
        scaled.set_attitude_correction(1., 201000000LL);
    } catch (const std::invalid_argument&) {
        thrown = true;
    }
    CHECK(thrown);
    CHECK(scaled.clock_alignment_state().at("attitude_correction").at("pitch_scale") == 1.4);
    unsigned compared = 0;
    for (unsigned i = 0; i < 40; ++i) {
        const auto camera_ns = 10000000000LL + i * 33000000LL;
        const auto receipt = camera_ns + 100000000LL;
        const NativeJson pixels = {
            {"schema", "argus.perception.observations.v2"},
            {"clock_domain", "host_monotonic"},
            {"coordinate_frame", "camera_pixel"},
            {"source", "ir_sdk:synthetic"},
            {"captured_monotonic_ns", receipt},
            {"image_width", 640},
            {"image_height", 480},
            {"camera_clock", {123, 5000 + i * 33, i + 1, camera_ns + 10000000LL}},
            {"detections",
             {{{"x1", 300.},
               {"y1", 300.},
               {"x2", 340.},
               {"y2", 360.},
               {"confidence", .9},
               {"class_id", 0},
               {"label", "car"}}}}};
        NativeJson history = {{"ATTITUDE", NativeJson::array()},
                              {"LOCAL_POSITION_NED", NativeJson::array()},
                              {"DISTANCE_SENSOR", NativeJson::array()}};
        for (unsigned j = 0; j < 100; ++j) {
            const auto measured = 10000000000LL + j * 16000000LL;
            const auto arrived = measured + 10000000LL;
            if (arrived > receipt + 1000000LL || receipt + 1000000LL - arrived > 200000000) {
                continue;
            }
            for (const auto* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
                NativeJson data;
                if (std::string(kind) == "ATTITUDE") {
                    // 기수 들기 약 3°, 시간에 따라 조금씩 변하는 pitch와 1 rad/s yaw
                    data = {{"roll", 0.}, {"pitch", .05 + j * .0005}, {"yaw", j * .016}};
                } else if (std::string(kind) == "LOCAL_POSITION_NED") {
                    data = {{"x", j * .0016}, {"y", 0.}, {"z", -1.7}};
                } else {
                    data = {{"orientation", 25},
                            {"current_distance", 170},
                            {"min_distance", 10},
                            {"max_distance", 1000}};
                }
                data["time_boot_ms"] = 1000 + j * 16;
                history[kind].push_back({{"received_monotonic_ns", arrived}, {"data", data}});
            }
        }
        const NativeJson fc = {{"schema", "argus.fc.observations.v1"},
                               {"clock_domain", "host_monotonic"},
                               {"epoch", 1},
                               {"state",
                                {{"uid", uid},
                                 {"identity_ok", true},
                                 {"fresh", true},
                                 {"local_ok", true},
                                 {"flight_environment", "indoor"},
                                 {"flow_quality", 100}}},
                               {"sensors", {{"flow_valid", true}, {"gps_valid", false}}},
                               {"history", history}};
        const auto now = receipt + 1000000LL;
        // 네 투영기가 매 프레임을 모두 받아야 시계 정렬이 같은 시점에 준비된다.
        PiProjectedFrame a{}, b{}, c{}, d{};
        bool all_ready = true;
        const auto run = [&](const PiGroundProjector& projector, PiProjectedFrame& out) {
            try {
                out = projector.project(pixels, fc, now);
            } catch (const std::runtime_error& error) {
                CHECK(std::string(error.what()) == "SDK/FC clocks warming up");
                all_ready = false;
            }
        };
        run(base, a);
        run(explicit_off, b);
        run(scaled, c);
        run(shifted, d);
        if (!all_ready) {
            continue;
        }
        CHECK(a.detections.size() == 1 && b.detections.size() == 1 && c.detections.size() == 1 &&
              d.detections.size() == 1);
        const auto& pa = a.detections[0].metric.point;
        // 꺼짐: 기존과 완전히 같다.
        CHECK((b.detections[0].metric.point - pa).norm() == 0 &&
              (b.pose.rpy - a.pose.rpy).norm() == 0);
        // 켜짐: 발행 pose는 그대로, 지면 점만 바뀐다.
        CHECK((c.pose.rpy - a.pose.rpy).norm() == 0 && (d.pose.rpy - a.pose.rpy).norm() == 0);
        CHECK((c.detections[0].metric.point - pa).norm() > .001);
        CHECK((d.detections[0].metric.point - pa).norm() > .001);
        // 기수 들기에서 pitch 보정을 키우면 표적이 더 멀리 투영된다 (Replay에서 짧게 잡히던 방향).
        const Eigen::Vector2d drone(a.pose.position[0], a.pose.position[1]);
        CHECK((c.detections[0].metric.point - drone).norm() > (pa - drone).norm());
        // 2026-11-19 PAMPC #5: 발행용 투영 자세/평면 down은 실제 투영 값과 같다.
        // 보정이 꺼지면 투영 자세는 발행 pose 자세와 같고, 켜면 보정된 자세다. 평면은 보정과
        // 무관하다.
        CHECK((a.projection_rpy - a.pose.rpy).norm() == 0 &&
              (b.projection_rpy - b.pose.rpy).norm() == 0);
        CHECK(std::abs(c.projection_rpy[1] - 1.4 * a.pose.rpy[1]) < 1e-12 &&
              c.projection_rpy[0] == a.pose.rpy[0] && c.projection_rpy[2] == a.pose.rpy[2]);
        CHECK((d.projection_rpy - a.pose.rpy).norm() > 1e-6);
        CHECK(std::isfinite(a.ground_plane_down_m) && std::abs(a.ground_plane_down_m) < .2);
        CHECK(a.ground_plane_down_m == b.ground_plane_down_m &&
              a.ground_plane_down_m == c.ground_plane_down_m &&
              a.ground_plane_down_m == d.ground_plane_down_m);
        ++compared;
    }
    CHECK(compared >= 20);
}
static void projection_contracts(const char* camera, const char* settings, bool outdoor) {
    const auto calibration = read_native_json(camera);
    const auto geometry = calibration.at("geometry");
    const std::string uid = calibration.at("expected_uid");
    auto measurement = read_native_json(settings);
    measurement["navigation_policy"] = outdoor ? "gps" : "optical_flow";
    measurement["sdk_time_alignment"] = true;
    PiGroundProjector corrected(geometry, measurement, uid);
    measurement["sdk_time_alignment"] = false;
    PiGroundProjector legacy(geometry, measurement, uid);
    NativeJson pixels, fc;
    PiProjectedFrame latest{};
    unsigned corrected_count = 0;
    for (unsigned i = 0; i < 40; ++i) {
        const auto camera_ns = 10000000000LL + i * 33000000LL;
        const auto receipt = camera_ns + 100000000LL;
        pixels = {{"schema", "argus.perception.observations.v2"},
                  {"clock_domain", "host_monotonic"},
                  {"coordinate_frame", "camera_pixel"},
                  {"source", "ir_sdk:synthetic"},
                  {"captured_monotonic_ns", receipt},
                  {"image_width", 640},
                  {"image_height", 480},
                  {"camera_clock", {123, 5000 + i * 33, i + 1, camera_ns + 10000000LL}},
                  {"detections", NativeJson::array()}};
        NativeJson history = {{"ATTITUDE", NativeJson::array()},
                              {"LOCAL_POSITION_NED", NativeJson::array()},
                              {"DISTANCE_SENSOR", NativeJson::array()}};
        for (unsigned j = 0; j < 100; ++j) {
            const auto measured = 10000000000LL + j * 16000000LL;
            const auto arrived = measured + 10000000LL;
            if (arrived > receipt + 1000000LL || receipt + 1000000LL - arrived > 200000000) {
                continue;
            }
            for (const auto* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
                NativeJson data;
                if (std::string(kind) == "ATTITUDE") {
                    data = {{"roll", 0.}, {"pitch", 0.}, {"yaw", j * .016}};
                } else if (std::string(kind) == "LOCAL_POSITION_NED") {
                    data = {{"x", j * .0016}, {"y", 0.}, {"z", -1.7}};
                } else {
                    data = {{"orientation", 25},
                            {"current_distance", 170},
                            {"min_distance", 10},
                            {"max_distance", 1000}};
                }
                data["time_boot_ms"] = 1000 + j * 16;
                history[kind].push_back({{"received_monotonic_ns", arrived}, {"data", data}});
            }
        }
        fc = {{"schema", "argus.fc.observations.v1"},
              {"clock_domain", "host_monotonic"},
              {"epoch", 1},
              {"state",
               {{"uid", uid},
                {"identity_ok", true},
                {"fresh", true},
                {"local_ok", true},
                {"flight_environment", outdoor ? "outdoor" : "indoor"},
                {"flow_quality", 100}}},
              {"sensors", {{"flow_valid", !outdoor}, {"gps_valid", outdoor}}},
              {"history", history}};
        try {
            latest = corrected.project(pixels, fc, receipt + 1000000LL);
            ++corrected_count;
            CHECK(latest.sample_ns == receipt && latest.motion_sample_ns < receipt);
            CHECK(std::abs(latest.pose.position[0] - i * .0033) < .001);
            CHECK(std::abs(latest.pose.rpy[2] - i * .033) < .001);
            const auto old = legacy.project(pixels, fc, receipt + 1000000LL);
            CHECK(old.pose.position[0] - latest.pose.position[0] > .005);
        } catch (const std::runtime_error& error) {
            CHECK(std::string(error.what()) == "SDK/FC clocks warming up");
        }
    }
    CHECK(corrected_count >= 20);
    // Old FC measurement delivered now: host-receipt alignment accepts it;
    // the calibrated original FC boot timestamp must reject it.
    const auto receipt = pixels.at("captured_monotonic_ns").get<std::int64_t>();
    for (const auto* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
        auto old = fc["history"][kind].front();
        old["received_monotonic_ns"] = receipt;
        fc["history"][kind] = NativeJson::array({old});
    }
    legacy.project(pixels, fc, receipt + 1000000LL);
    bool rejected = false;
    try {
        corrected.project(pixels, fc, receipt + 1000000LL);
    } catch (const std::runtime_error& error) {
        rejected = std::string(error.what()) == "SDK/FC clock alignment exceeds 50ms" ||
                   std::string(error.what()) ==
                       "FC attitude interpolation requires fresh brackets within 50ms";
    }
    CHECK(rejected);
    auto missing = pixels;
    missing.erase("camera_clock");
    rejected = false;
    try {
        corrected.project(missing, fc, receipt + 1000000LL);
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    CHECK(rejected);
    // Non-SDK sources retain receipt policy. Environment-specific sensor guards remain.
    missing["source"] = "rgb_camera:test";
    corrected.project(missing, fc, receipt + 1000000LL);
    if (outdoor) {
        fc["sensors"]["gps_valid"] = false;
        rejected = false;
        try {
            corrected.project(pixels, fc, receipt + 1000000LL);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        CHECK(rejected);
    }
}
// 2026-11-12: 시작 창에서 늦게(40ms) 잡힌 추정은 더 이른 표본이 오면 바로 내려가고,
// 늦게 온 묶음(burst)은 추정을 갑자기 올리지 못한다.
static void late_anchor_contracts() {
    DeviceClockAlignment clock;
    // 첫 200ms: 모든 표본이 40ms 늦게 도착 (연결 직후 FC 묶음 전달)
    unsigned i = 0;
    for (; i < 12; ++i) {
        clock.observe(5000 + i * 20, 10000000000LL + i * 20000000LL + 40000000LL);
    }
    CHECK(clock.ready());
    CHECK(clock.offset_ns() == 10000000000LL - 5000LL * 1000000LL + 40000000LL);
    // 이후 정상 지연 1ms 표본: 추정이 바로 1ms 쪽으로 내려간다 (예전에는 100ppm, 수백 초)
    for (; i < 20; ++i) {
        clock.observe(5000 + i * 20, 10000000000LL + i * 20000000LL + 1000000LL);
    }
    const auto mapped = clock.map(5000 + 19 * 20);
    CHECK(mapped);
    const auto receipt = 10000000000LL + 19 * 20000000LL + 1000000LL;
    CHECK(receipt - *mapped >= 0 && receipt - *mapped < 2000000);
    // 늦은 묶음(60ms 지연) 3초: 최소값 창(2초)이 지나도 추정은 100ppm보다 빨리 오르지 않는다.
    const auto before = clock.offset_ns();
    for (; i < 170; ++i) {
        clock.observe(5000 + i * 20, 10000000000LL + i * 20000000LL + 60000000LL);
    }
    CHECK(clock.offset_ns() - before <= 400000);  // 3초 x 100ppm = 0.3ms (+반올림)
}

static void failure_window_contracts() {
    // 실패와 성공이 섞여도 1초에 실패 8회 이상·절반 이상이면 다시 잡는다.
    AlignmentFailureWindow mixed;
    bool fired = false;
    std::int64_t now = 1000000000LL;
    for (int k = 0; k < 30 && !fired; ++k) {
        now += 33000000LL;
        fired = mixed.record(now, k % 3 != 0);  // 2/3 실패
    }
    CHECK(fired);
    mixed.reanchored(now);
    CHECK(mixed.failures() == 0 && mixed.attempts() == 0);
    // 다시 잡은 뒤 2초 동안은 판단하지 않고, 2초가 지나면 다시 가능하다.
    const auto reanchored_at = now;
    bool again = false;
    for (int k = 0; k < 90 && !again; ++k) {
        now += 33000000LL;
        again = mixed.record(now, true);
        CHECK(!again || now - reanchored_at >= AlignmentFailureWindow::kCooldownNs);
    }
    CHECK(again);
    // 가끔 실패(1/5)는 다시 잡지 않는다.
    AlignmentFailureWindow sparse;
    now = 1000000000LL;
    for (int k = 0; k < 300; ++k) {
        now += 33000000LL;
        CHECK(!sparse.record(now, k % 5 == 0));
    }
    // 실패 7회뿐이면 다시 잡지 않는다.
    AlignmentFailureWindow few;
    now = 1000000000LL;
    for (int k = 0; k < 7; ++k) {
        now += 33000000LL;
        CHECK(!few.record(now, true));
    }
}

int main(int argc, char** argv) {
    try {
        CHECK(argc == 3);
        clock_contracts();
        late_anchor_contracts();
        failure_window_contracts();
        deadzone_contracts();
        projection_contracts(argv[1], argv[2], false);
        projection_contracts(argv[1], argv[2], true);
        attitude_correction_contracts(argv[1], argv[2]);
        std::cout << "indoor/outdoor SDK/FC original-clock, late-sample rejection, and speed "
                     "deadzone passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
