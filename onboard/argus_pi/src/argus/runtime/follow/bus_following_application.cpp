// 버스 worker와 FC adapter를 수명 관리하고 추적 입력과 독립된 제어 주기를 실행한다.
#include "argus/runtime/follow/include/bus_following_application.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

#include "argus/commands/target/include/zeromq_target_command_gateway.hpp"
#include "argus/control/contracts/include/argus_state_codes.hpp"
#include "argus/control/contracts/include/follow_b_request.hpp"
#include "argus/control/contracts/include/follow_b_status.hpp"
#include "argus/control/contracts/include/icd_command_progress.hpp"
#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/control/guidance/include/yaw_scan_controller.hpp"
#include "argus/control/runtime/include/flight_control_runtime.hpp"
#include "argus/control/safety/include/ground_yaw_bench_guard.hpp"
#include "argus/diagnostics/status/include/system_status_v1.hpp"
#include "argus/diagnostics/status/include/zeromq_system_status_publisher.hpp"
#include "argus/runtime/contracts/include/follow_ports.hpp"
#include "argus/runtime/contracts/include/i_follow_backend.hpp"
#include "argus/runtime/follow/include/bounded_discard_recheck.hpp"
#include "argus/runtime/follow/include/control_tick_wakeup.hpp"
#include "argus/runtime/follow/include/decision_log.hpp"
#include "argus/runtime/follow/include/follow_operator_gateway.hpp"
#include "argus/runtime/follow/include/follow_status_snapshot.hpp"
#include "argus/runtime/follow/include/follow_target_request.hpp"
#include "argus/transport/zeromq/include/target_tracks_worker.hpp"

namespace follow {

namespace {
// Runtime 내부 Stop 전이도 전송 직전의 selector epoch를 backend에 전달한다.
class ContextVehiclePort final : public IVehiclePort {
public:
    ContextVehiclePort(IVehiclePort& port, std::recursive_mutex& context_mutex,
                       std::function<void()> sync)
        : port_(port), context_mutex_(context_mutex), sync_(std::move(sync)) {}
    bool send(const FollowCommand& command) override {
        std::lock_guard<std::recursive_mutex> lock(context_mutex_);
        sync_();
        return port_.send(command);
    }
    bool send_until(const FollowCommand& command, Clock::time_point until) override {
        std::lock_guard<std::recursive_mutex> lock(context_mutex_);
        sync_();
        return port_.send_until(command, until);
    }
    void revoke_commands() override {
        std::lock_guard<std::recursive_mutex> lock(context_mutex_);
        sync_();
        port_.revoke_commands();
    }

private:
    IVehiclePort& port_;
    std::recursive_mutex& context_mutex_;
    std::function<void()> sync_;
};
class ContextModePort final : public IFlightModePort {
public:
    ContextModePort(IFlightModePort& port, std::recursive_mutex& context_mutex,
                    std::function<void()> sync)
        : port_(port), context_mutex_(context_mutex), sync_(std::move(sync)) {}
    bool start_offboard() override {
        std::lock_guard<std::recursive_mutex> lock(context_mutex_);
        sync_();
        return port_.start_offboard();
    }
    bool stop_offboard() override {
        std::lock_guard<std::recursive_mutex> lock(context_mutex_);
        sync_();
        return port_.stop_offboard();
    }
    std::optional<bool> start_result() const override { return port_.start_result(); }
    std::optional<bool> hold_result() const override { return port_.hold_result(); }

private:
    IFlightModePort& port_;
    std::recursive_mutex& context_mutex_;
    std::function<void()> sync_;
};
bool write_guidance_diagnostic(const std::filesystem::path& path,
                               const FlightControlRuntimeStatus& tick, Clock::time_point now,
                               double deadband) {
    const auto next = path.string() + ".next";
    std::ofstream file(next);
    file << std::setprecision(12);
    const bool valid = tick.command_sent && tick.mission == "FOLLOW_ACTIVE" && tick.rear_goal &&
                       tick.vehicle_reference_ned && tick.guidance_pose;
    file << "{\"schema\":\"argus.guidance.diagnostic.v1\",\"monotonic_ns\":"
         << std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()
         << ",\"sample_monotonic_ns\":" << tick.command_sample_ns
         << ",\"command_epoch\":" << tick.selection_command_epoch << ",\"deadband_m\":" << deadband
         << ",\"valid\":" << (valid ? "true" : "false");
    file << ",\"command_valid_until_ns\":"
         << std::chrono::duration_cast<std::chrono::nanoseconds>(
                tick.command_valid_until.time_since_epoch())
                .count();
    if (valid) {
        const auto& goal = *tick.rear_goal;
        const auto& point = *tick.vehicle_reference_ned;
        const auto& pose = *tick.guidance_pose;
        file << ",\"goal_ned\":[" << goal.north_m << ',' << goal.east_m << "],\"reference_ned\":["
             << point[0] << ',' << point[1] << "],\"goal_velocity_ned\":["
             << goal.velocity_north_m_s << ',' << goal.velocity_east_m_s << "],\"pose_ned\":["
             << pose.north_m << ',' << pose.east_m << "],\"pose_at_ns\":"
             << std::chrono::duration_cast<std::chrono::nanoseconds>(
                    pose.position_at.time_since_epoch())
                    .count()
             << ",\"initial_extension\":" << (goal.initial_extension ? "true" : "false");
        if (goal.standoff_m > 0) {
            file << ",\"goal_standoff_m\":" << goal.standoff_m;
        }
        if (tick.altitude_target_down_m) {
            file << ",\"altitude_target_down_m\":" << *tick.altitude_target_down_m;
        }
        if (tick.pid_terms) {
            file << ",\"pid_terms\":[";
            for (std::size_t i = 0; i < tick.pid_terms->size(); ++i) {
                const auto& p = (*tick.pid_terms)[i];
                if (i) {
                    file << ',';
                }
                file << "{\"p\":" << p.p << ",\"i\":" << p.i << ",\"d\":" << p.d
                     << ",\"output\":" << p.output << '}';
            }
            file << ']';
        }
    }
    file << "}\n";
    file.close();
    if (!file) {
        return false;
    }
    std::error_code error;
    std::filesystem::rename(next, path, error);
    return !error;
}
const char* target_state_name(TargetState state) {
    switch (state) {
        case TargetState::Confirmed:
            return "CONFIRMED";
        case TargetState::Predicted:
            return "PREDICTED";
        case TargetState::Lost:
            return "LOST";
        case TargetState::Ambiguous:
            return "AMBIGUOUS";
    }
    return "LOST";
}

SystemStatusV1 make_status(const FlightControlRuntimeStatus& runtime,
                           const TargetSelectionSnapshot& selection,
                           const VehicleFlightStatus& flight, Clock::time_point now) {
    SystemStatusV1 status;
    status.source = "bus-follow";
    status.monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    status.mission = runtime.mission;
    status.reason = runtime.reason;
    status.command_epoch = selection.command_epoch;
    if (selection.target) {
        status.target_generation = selection.target->generation;
        status.target_track_id = selection.target->track_id;
    }
    status.target_state = target_state_name(selection.state);
    status.age_since_real_s = selection.age_since_real_s;
    status.connected = flight.connected;
    status.armed = flight.armed;
    status.in_air = flight.in_air;
    status.offboard = flight.offboard;
    status.hold = flight.hold;
    status.command_sent = runtime.command_sent;
    status.offboard_start_requested = runtime.offboard_start_requested;
    status.offboard_stop_requested = runtime.offboard_stop_requested;
    return status;
}

// 실행 중 추종기의 정비 선택값 이름. 2026-11-12부터 설정의 follow_controller_profile을 쓰고,
// 키가 없는 예전 설정은 예전 판별(높이 3.0 = 3 m)을 따른다.
std::string configured_follow_controller(const BusFollowingConfig& profile) {
    if (!profile.follow_controller_profile.empty()) {
        return profile.follow_controller_profile;
    }
    if (profile.guidance.outdoor_image_pid) {
        return "pid";
    }
    if (profile.guidance.outdoor_profile && profile.guidance.predictive_target_height_m == 3.0) {
        return "predictive_low";
    }
    return "predictive";
}
}  // namespace

BusFollowingApplication::BusFollowingApplication(BusFollowingConfig config,
                                                 std::function<bool()> interrupted)
    : config_(std::move(config)), interrupted_(std::move(interrupted)) {}

void BusFollowingApplication::enable_vmy(BusFollowingConfig moving, BusFollowingConfig yaw,
                                         std::string endpoint, double scan_rate,
                                         std::chrono::milliseconds timeout,
                                         std::map<std::uint32_t, std::string> sensor_sources) {
    // 2026-11-18 데모 모드(demo)는 moving, yaw 두 설정이 모두 yaw 전용이다. 나머지 일치 검사는
    // 같다.
    const bool demo = demo_follow_controller_profile(moving.follow_controller_profile);
    const bool moving_kind_ok =
        demo ? moving.guidance.yaw_only && !moving.guidance.outdoor_profile
             : !moving.guidance.yaw_only &&
                   (moving.guidance.predictive_follow || moving.guidance.outdoor_image_pid);
    if (!moving.external_tracks || !yaw.external_tracks || !moving_kind_ok ||
        !yaw.guidance.yaw_only || moving.guidance.outdoor_profile != yaw.guidance.outdoor_profile ||
        moving.expected_fc_uid != yaw.expected_fc_uid ||
        moving.expected_fc_uid != config_.expected_fc_uid ||
        moving.expected_fc_system_id != yaw.expected_fc_system_id ||
        moving.expected_fc_component_id != yaw.expected_fc_component_id ||
        moving.px4_connection_url != yaw.px4_connection_url ||
        moving.tracks_sub_endpoint != yaw.tracks_sub_endpoint ||
        moving.geometry.camera_to_body != yaw.geometry.camera_to_body ||
        moving.geometry.camera_offset_body_m != yaw.geometry.camera_offset_body_m ||
        moving.geometry.fx != yaw.geometry.fx || moving.geometry.fy != yaw.geometry.fy ||
        moving.geometry.cx != yaw.geometry.cx || moving.geometry.cy != yaw.geometry.cy ||
        moving.geometry.image_width != yaw.geometry.image_width ||
        moving.geometry.image_height != yaw.geometry.image_height ||
        moving.runtime.ground_down_m != yaw.runtime.ground_down_m ||
        moving.runtime.hold_down_m != yaw.runtime.hold_down_m ||
        moving.guidance.command_ttl != yaw.guidance.command_ttl ||
        moving.guidance.max_pose_age != yaw.guidance.max_pose_age ||
        moving.follow_controller_profile != yaw.follow_controller_profile ||
        moving.enable_rotation_cmc || yaw.enable_rotation_cmc ||
        endpoint.rfind("tcp://127.0.0.1:", 0) != 0) {
        throw std::invalid_argument("vMy profiles require matching identity, geometry and runtime");
    }
    if (!moving.follow_controller_profile.empty()) {
        // 2026-11-12: 모드 이름의 장착각(30°/45°)과 카메라 보정 회전이 같아야 한다.
        // 실제 장착은 알 수 없으므로 설정끼리의 일치만 본다. 기울기 = atan2(Rzz, Rxz).
        const auto& mount = moving.geometry.camera_to_body;
        const double tilt_deg =
            std::atan2(mount.at(8), mount.at(2)) * 180. / 3.14159265358979323846;
        const double expected =
            mount30_follow_controller_profile(moving.follow_controller_profile) ? 30. : 45.;
        if (std::abs(tilt_deg - expected) > 1.) {
            throw std::invalid_argument(
                "camera mount tilt does not match follow_controller_profile");
        }
    }
    (void)YawScanController(YawScanConfig{scan_rate, timeout});
    vmy_enabled_ = true;
    moving_profile_ = std::move(moving);
    yaw_profile_ = std::move(yaw);
    // One latest manager is shared by both modes, including short-motion and anchor policy.
    config_.selector = moving_profile_.selector;
    vmy_endpoint_ = std::move(endpoint);
    scan_rate_ = scan_rate;
    scan_timeout_ = timeout;
    sensor_sources_ = std::move(sensor_sources);
}

int BusFollowingApplication::run_with_backend(IFollowBackend& backend) {
    // SDK와 직렬 링크의 수명/송신권은 backend가 소유한다.
    BackendVehicleState state_port{backend};
    BackendVelocitySetpoint velocity_port{backend};
    BackendModeControl mode_port{backend};
    auto& pose = state_port.pose();
    auto& flight_state = state_port.flight_state();
    std::recursive_mutex target_context_mutex;
    std::function<void(Clock::time_point)> update_context = [](Clock::time_point) {};
    ContextVehiclePort vehicle{velocity_port, target_context_mutex,
                               [&] { update_context(Clock::now()); }};
    ContextModePort mode{mode_port, target_context_mutex, [&] { update_context(Clock::now()); }};
    std::atomic<bool> bench_active{false};
    double bench_rate = 0;
    Clock::time_point bench_until{};
    bool bench_sdk_accepted = false;
    bool bench_stopping = false;
    bool bench_zero_sdk_accepted = false;
    std::string bench_reason = "not_started";
    const auto bench_ground = [&] {
        const auto state = flight_state.read();
        const auto sample = pose.read();
        const auto now = Clock::now();
        return ground_yaw_bench_state_valid(state, sample, now, config_.guidance.max_pose_age);
    };
    std::mutex transition_mutex;
    const auto identity_permit = [&backend] { return backend.identity_permit(); };
    const auto writer_permit = [&backend, &identity_permit, &bench_active, &bench_ground] {
        return identity_permit() && backend.owns() && (!bench_active.load() || bench_ground());
    };
    backend.set_writer_guard(writer_permit);
    struct WriterGuardScope {
        IFollowBackend& backend;
        ~WriterGuardScope() {
            backend.set_writer_guard({});
            backend.vehicle().revoke_commands();
            backend.release();
        }
    } writer_guard_scope{backend};
    const auto close_bench = [&] {
        vehicle.revoke_commands();
        bench_active.store(false);
        bench_stopping = false;
        bench_rate = 0;
        backend.release();
    };
    const auto stop_bench = [&](const std::string& reason) {
        bench_rate = 0;
        bench_reason = reason;
        if (!bench_stopping && bench_active.load() && bench_ground()) {
            update_context(Clock::now());
            bench_zero_sdk_accepted =
                vehicle.send_until(FollowCommand{}, Clock::now() + std::chrono::milliseconds(150));
            if (bench_zero_sdk_accepted) {
                bench_stopping = true;
                bench_until = Clock::now() + std::chrono::milliseconds(100);
                return;
            }
        }
        close_bench();
    };

    // 2. 관측 → 추적 → 선택 → 제어 부품과 독립 감시자를 조립한다.
    ZeroMqContext bus_context;
    if (!config_.external_tracks || config_.enable_rotation_cmc) {
        throw std::invalid_argument(
            "external UCMC tracks required; internal image CMC was removed");
    }
    const YawScanConfig scan_defaults{scan_rate_, scan_timeout_};
    YawScanController scan{scan_defaults};
    bool heading_stop_pending = false;
    auto next_heading_at = Clock::time_point{};
    auto selector_config = config_.selector;
    selector_config.allow_idle_camera_context_switch = backend.remote_fc_owner();
    TargetSelector selector{selector_config};
    update_context = [&](Clock::time_point now) {
        std::lock_guard<std::recursive_mutex> lock(target_context_mutex);
        const auto current = selector.snapshot(std::max(now, Clock::now()));
        FollowCommandContext context;
        context.command_epoch = current.command_epoch;
        if (current.target) {
            context.generation = current.target->generation;
            context.track_id = current.target->track_id;
        } else if (current.latest_frame) {
            context.generation = current.latest_frame->generation;
        }
        if (current.latest_frame) {
            context.source_sample_monotonic_ns = current.latest_frame->sample_monotonic_ns;
        }
        if (current.cv_gap_active && current.cv_gap_deadline) {
            context.cv_gap_active = true;
            context.cv_origin_sample_ns = current.cv_origin_sample_ns;
            context.cv_gap_deadline_monotonic_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    current.cv_gap_deadline->time_since_epoch())
                    .count();
            context.source_sample_monotonic_ns = current.cv_origin_sample_ns;
        }
        context.bench = bench_active.load();
        backend.update_context(context);
    };
    update_context(Clock::now());
    // 2026-11-16 제어 이벤트 구동 (ARGUS_CONTROL_EVENT_DRIVEN=1): 받아들인 추적 결과마다 tick을
    // 깨운다. 꺼져 있으면 기존 고정 주기(control_period)만 쓴다.
    const char* event_env = std::getenv("ARGUS_CONTROL_EVENT_DRIVEN");
    const bool event_driven = event_env != nullptr && std::string(event_env) == "1";
    constexpr auto kMinEventTickInterval = std::chrono::milliseconds(20);  // 최대 50Hz
    ControlTickWakeup tick_wakeup;
    // 2026-11-17 사용자 지시 "다시 내가 클릭하고 추종시작 누르면 추종돼야지": 누른 순간의 최신
    // 프레임 한 장에 대상이 없다고 추종 시작을 거부하지 않는다. 같은 추적기 generation/source에서
    // 실제로 확인된 CONFIRMED 트랙을 기억해 후보로 내보내고, 클릭한 대상이 지금 프레임에 없으면
    // 시작 요청을 진행 중으로 두었다가 그 대상이 다시 잡히는 프레임에서 기존 시작 검사를 그대로
    // 거쳐 시작한다. 기억은 generation/source가 바뀌면 지운다(트랙 ID가 다른 대상을 가리킬 수
    // 있음).
    struct RecentTrack {
        TrackRecordV1 track;
        std::uint64_t generation{0};
        std::string source;
        Clock::time_point seen_at{};
    };
    std::mutex recent_mutex;
    std::map<std::int64_t, RecentTrack> recent_tracks;
    constexpr std::size_t kRecentTrackCapacity = 256;
    const auto remember_tracks = [&](const TracksV1& frame, Clock::time_point received_at) {
        std::lock_guard<std::mutex> lock(recent_mutex);
        if (!recent_tracks.empty()) {
            const auto& any = recent_tracks.begin()->second;
            if (any.generation != frame.generation || any.source != frame.source) {
                recent_tracks.clear();
            }
        }
        for (const auto& track : frame.tracks) {
            if (track.real_detection && track.state == "CONFIRMED") {
                recent_tracks[track.track_id] =
                    RecentTrack{track, frame.generation, frame.source, received_at};
            }
        }
        while (recent_tracks.size() > kRecentTrackCapacity) {
            auto oldest = recent_tracks.begin();
            for (auto it = recent_tracks.begin(); it != recent_tracks.end(); ++it) {
                if (it->second.seen_at < oldest->second.seen_at) {
                    oldest = it;
                }
            }
            recent_tracks.erase(oldest);
        }
    };
    const auto remembered_track = [&](std::uint64_t generation, const std::string& source,
                                      std::int64_t track_id) -> std::optional<RecentTrack> {
        std::lock_guard<std::mutex> lock(recent_mutex);
        const auto found = recent_tracks.find(track_id);
        if (found == recent_tracks.end() || found->second.generation != generation ||
            found->second.source != source) {
            return std::nullopt;
        }
        return found->second;
    };
    TargetTracksWorker tracks{
        bus_context,
        selector,
        config_.tracks_sub_endpoint,
        std::chrono::milliseconds(25),
        BusSocketEndpoint::Connect,
        config_.diagnostics_csv.empty() ? std::string{} : config_.diagnostics_csv + ".target.csv",
        [&](const TracksV1& frame, Clock::time_point received_at) {
            std::lock_guard<std::recursive_mutex> lock(target_context_mutex);
            // Preserve receipt time: waiting for an enqueue must not refresh a frame.
            const auto checked_at = Clock::now();
            if (checked_at < received_at ||
                checked_at - received_at > config_.selector.max_tracks_age) {
                return false;
            }
            const bool accepted = selector.update(frame, received_at);
            if (accepted) {
                remember_tracks(frame, received_at);
            }
            // A rejected context-change frame can still invalidate recovery.
            update_context(checked_at);
            if (accepted && event_driven) {
                tick_wakeup.notify();
            }
            return accepted;
        }};
    ZeroMqTargetCommandGateway commands{
        bus_context,
        selector,
        config_.command_rep_endpoint,
        BusSocketEndpoint::Bind,
        std::chrono::milliseconds(25),
        config_.expected_fc_uid,
        identity_permit,
        [&](const TargetCommand& command) {
            std::lock_guard<std::mutex> lock(transition_mutex);
            if (vmy_enabled_) {
                return TargetCommandResult{command.request_id, false, "REJECTED",
                                           "use_native_vmy_operator_endpoint"};
            }
            const bool was_owned = backend.owns();
            if (command.action == TargetCommandAction::Start && !backend.acquire()) {
                return TargetCommandResult{command.request_id, false, "REJECTED",
                                           "another_setpoint_writer_owns_fc"};
            }
            const auto result = selector.execute(command, Clock::now());
            update_context(Clock::now());
            if (!result.accepted && !was_owned) {
                backend.release();
            }
            return result;
        }};
    // Standalone Pi has no XSUB/XPUB broker: native PUB binds and HTTP SUB connects.
    ZeroMqSystemStatusPublisher status_publisher{bus_context, config_.status_pub_endpoint,
                                                 BusSocketEndpoint::Bind};
    CalibratedGroundGeometry geometry{config_.geometry};
    FlightGuidanceController guidance{config_.guidance};
    // 2026-11-16 요 시드 시선각 변화율 항 저역 필터 (config/follow/control-timing.env, 0=꺼짐)
    double los_rate_filter_s = 0;
    if (const char* los_env = std::getenv("ARGUS_LOS_RATE_FILTER_S");
        los_env != nullptr && *los_env != '\0') {
        try {
            los_rate_filter_s = std::stod(los_env);
        } catch (const std::exception&) {
            throw std::runtime_error("ARGUS_LOS_RATE_FILTER_S must be a number (0..1 s)");
        }
    }
    guidance.set_los_rate_filter(los_rate_filter_s);
    FlightSafetyEventLatch safety_events;
    auto runtime_config = config_.runtime;
    runtime_config.require_explicit_stop_recovery = backend.remote_fc_owner();
    FlightControlRuntime control{
        selector, pose, flight_state,   geometry,       guidance,
        vehicle,  mode, runtime_config, &safety_events, &target_context_mutex};
    FlightControlWatchdog watchdog{selector,
                                   flight_state,
                                   pose,
                                   safety_events,
                                   control.following_active_flag(),
                                   config_.watchdog_period,
                                   config_.guidance.max_pose_age};

    std::unique_ptr<VMyOperatorGateway> native_operator;
    if (vmy_enabled_) {
        native_operator = std::make_unique<VMyOperatorGateway>(
            bus_context, vmy_endpoint_, backend.remote_fc_owner(),
            [&backend](const NativeJson& request) {
                if (backend.remote_fc_owner() && request.value("action", "") == "select" &&
                    !backend.identity_permit()) {
                    return request.value("controller_session", "") ==
                               backend.controller_session() &&
                           request.value("onboard_session", "") == "observation-only" &&
                           request.value("fc_epoch", std::uint64_t{1}) == 0;
                }
                return backend.validate_operator_request(request);
            },
            sensor_sources_);
    }
    const auto scan_health = [&](Clock::time_point now) {
        auto flight = flight_state.read();
        auto sample = pose.read();
        auto frame = selector.snapshot(now).latest_frame;
        // 송신권 RPC 중 도착한 새 표본을 이전 tick 시각으로 '미래' 판정하지 않는다.
        // 표본을 읽은 뒤의 실제 시각으로 기존 신선도 한도를 그대로 검사한다.
        now = Clock::now();
        const bool frame_fresh =
            frame && frame->clock_domain == "host_monotonic" &&
            now >= Clock::time_point{std::chrono::nanoseconds{frame->sample_monotonic_ns}} &&
            now - Clock::time_point{std::chrono::nanoseconds{frame->sample_monotonic_ns}} <=
                config_.selector.max_tracks_age;
        return identity_permit() && writer_permit() && flight.connected && flight.armed &&
               flight.in_air && flight.hold && !flight.offboard && !flight.rc_override &&
               sample.fresh(now) && now >= sample.position_at && now >= sample.attitude_at &&
               now - sample.position_at <= config_.guidance.max_pose_age &&
               now - sample.attitude_at <= config_.guidance.max_pose_age && frame_fresh;
    };
    const auto heading_stop_health = [&](Clock::time_point now) {
        const auto flight = flight_state.read();
        const auto sample = pose.read();
        now = Clock::now();
        return identity_permit() && writer_permit() && flight.connected && flight.armed &&
               flight.in_air && flight.hold && !flight.offboard && !flight.rc_override &&
               sample.have_attitude && std::isfinite(sample.yaw_deg) && now >= sample.attitude_at &&
               now - sample.attitude_at <= config_.guidance.max_pose_age;
    };
    const auto send_heading = [&](double degrees, bool stopping = false) {
        const auto now = Clock::now();
        if (!vmy_enabled_ || !std::isfinite(degrees) ||
            !(stopping ? heading_stop_health(now) : scan_health(now))) {
            return false;
        }
        update_context(now);
        return mode_port.request_heading(degrees * std::acos(-1.0) / 180,
                                         now + config_.guidance.max_pose_age, stopping);
    };
    bool loop_ok = true;
    FlightControlRuntimeStatus latest_runtime_status;
    IcdCommandProgress command_progress;
    // 2026-11-17 클릭한 대상이 다시 잡힐 때까지 미룬 추종 시작. 기다리는 한도는 GCS가 시작 직전에
    // 준비하는 Position 인계 유효 시간(15초) 안인 10초. 새 운용자 명령이 오면 바로 버린다.
    struct DeferredStart {
        NativeJson request;
        Clock::time_point since;
    };
    std::optional<DeferredStart> deferred_start;
    bool replaying_deferred_start = false;
    constexpr auto kDeferredStartWait = std::chrono::seconds(10);
    const auto projected_status = [&](const TargetSelectionSnapshot& current,
                                      const VehicleFlightStatus& flight, Clock::time_point now) {
        FollowBStatusInput input;
        input.mission = latest_runtime_status.mission;
        input.rc_override = flight.rc_override;
        input.fault = latest_runtime_status.mission == "FAULT" ||
                      scan.state() == YawScanController::State::Fault;
        input.actual_hold =
            confirmed_stop_hold(flight.hold, flight.offboard, flight.offboard_hover);
        input.hold_transition = heading_stop_pending;
        if (input.mission == "IDLE" && control.inactive() && !scan.active() &&
            scan.state() != YawScanController::State::Idle &&
            scan.state() != YawScanController::State::Fault) {
            input.mission = "HOLD_CONFIRMED";
        }
        input.scan_active = scan.active();
        input.following = control.following_active_flag().load();
        input.target_selected = current.target.has_value();
        input.target_confirmed = current.fresh_real_detection;
        input.target_prediction_valid = prediction_report_valid(current, now);
        input.target_phase = current.target_phase;
        input.recovery_reason = current.recovery_reason;
        input.ambiguous = current.recovery_reason.find("ambiguous") != std::string::npos;
        const auto projected = project_follow_b_status(input);
        NativeJson selected = nullptr;
        if (current.target) {
            selected = {{"generation", current.target->generation},
                        {"track_id", current.target->track_id}};
        }
        command_progress.update(projected.mission, flight.offboard, flight.hold, control.inactive(),
                                scan.active(), heading_stop_pending,
                                yaw_scan_state_name(scan.state()), scan.reason(), selected,
                                flight.offboard_hover);
        return NativeJson{{"mission_state", projected.mission},
                          {"target_state", projected.target},
                          {"prediction_valid_until_monotonic_ns",
                           status_deadline(current.prediction_valid_until)},
                          {"observation_valid_until_monotonic_ns",
                           status_deadline(current.observation_valid_until)},
                          {"mission_state_code", mission_state_code(projected.mission).value_or(7)},
                          {"target_state_code", target_state_code(projected.target).value_or(3)},
                          {"target_phase", projected.target_phase},
                          {"recovery_reason", projected.recovery_reason},
                          {"ambiguous", projected.ambiguous},
                          {"command_results", command_progress.snapshot()}};
    };
    std::ofstream diagnostics;
    if (!config_.diagnostics_csv.empty()) {
        diagnostics.open(config_.diagnostics_csv);
        if (!diagnostics) {
            throw std::runtime_error("cannot create diagnostics CSV");
        }
        diagnostics
            << "monotonic_ns,sample_ns,mission,reason,mission_id,native_id,target_phase,command_"
               "sent,north,east,down,yaw,vx,vy,vz,yaw_rate,control_ms,selector_ms,guidance_ms,"
               "command_sample_ns,command_epoch,manager_advance_ms,command_generated_ns\n";
        diagnostics << std::setprecision(12);
    }
    // The separate bounded logger is best-effort; it never owns or gates FC commands.
    DecisionLog decision_log(
        config_.diagnostics_csv.empty()
            ? std::filesystem::path{}
            : std::filesystem::path(config_.diagnostics_csv + ".decision.jsonl"));
    // Keep diagnostics alive through both the normal loop and shutdown sequence.
    std::deque<std::function<NativeJson()>> pending_hold_diagnostics;
    std::uint64_t pending_hold_dropped = 0;
    const auto decision_session_monotonic_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
            .count();
    const auto decision_session_unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::system_clock::now().time_since_epoch())
                                              .count();
    const auto drain_hold_diagnostics = [&](Clock::time_point at) noexcept {
        if (config_.diagnostics_csv.empty()) {
            return;
        }
        try {
            for (auto& event : backend.take_hold_diagnostics()) {
                if (pending_hold_diagnostics.size() >= 128) {
                    ++pending_hold_dropped;
                    continue;
                }
                pending_hold_diagnostics.push_back([event = std::move(event)] { return event; });
            }
            if (pending_hold_dropped && pending_hold_diagnostics.size() < 128) {
                NativeJson loss = {
                    {"schema", "argus.follow.hold_diagnostic.loss.v1"},
                    {"monotonic_ns",
                     std::chrono::duration_cast<std::chrono::nanoseconds>(at.time_since_epoch())
                         .count()},
                    {"dropped_before", pending_hold_dropped},
                    {"stage", "application_queue"}};
                pending_hold_diagnostics.push_back([loss = std::move(loss)] { return loss; });
                pending_hold_dropped = 0;
            }
            while (!pending_hold_diagnostics.empty()) {
                if (!decision_log.try_record(pending_hold_diagnostics.front(), true, false)) {
                    break;
                }
                pending_hold_diagnostics.pop_front();
            }
        } catch (...) {
            decision_log.note_dropped();
        }
    };
    std::exception_ptr failure;
    try {
        // 구독자부터 준비해 PUB/SUB 시작 직후의 slow-joiner 유실 가능성을 줄인다.
        tracks.start();
        commands.start();
        watchdog.start();
        std::cout << "bus-follow ready; no automatic arm/takeoff; command endpoint="
                  << config_.command_rep_endpoint << '\n';
        std::cout << "control timing: "
                  << (event_driven ? "event-driven (new tracks, min 20ms, fallback period)"
                                   : "fixed period")
                  << " period_ms=" << config_.control_period.count()
                  << " los_rate_filter_s=" << los_rate_filter_s << '\n';

        // 3. 제어 tick을 실행하고 결과 상태를 게시한다.
        auto next_tick = Clock::now();
        std::string last_report;
        std::string last_decision_mission, last_decision_reason;
        std::string last_tracks_error;
        std::string last_backend_status_error;
        auto next_guidance_diagnostic = Clock::time_point{};
        NativeJson observed_fc_context;
        while (!interrupted_()) {
            const auto now = Clock::now();
            if (!backend.remote_fc_owner() && backend.verified_uid().empty()) {
                throw std::runtime_error("FC identity changed; writer permit revoked");
            }
            std::unique_lock<std::mutex> transition_lock(transition_mutex);
            update_context(now);
            if (backend.remote_fc_owner()) {
                const auto context = backend.operator_context();
                // 2026-11-06 실기: FAULT·MANUAL 고정이 착륙 뒤에도 남아
                // 메인 GUI가 계속 "추종 중"으로 보았다. 운용 context가 같아도
                // 고정 상태면 지상·DISARM 확인으로 초기화한다.
                if ((context != observed_fc_context || control.recovery_latched()) &&
                    identity_permit() && control.reset_ground_context(now)) {
                    vehicle.revoke_commands();
                    backend.release();
                    observed_fc_context = context;
                    std::cerr
                        << "ground FC context reset; selection cleared; explicit START required\n";
                    update_context(now);
                }
            }
            if (native_operator) {
                const auto execute_operator = [&](const NativeJson& request) {
                    const auto action_name = request.at("action").get<std::string>();
                    const bool observation_action =
                        backend.remote_fc_owner() &&
                        (action_name == "status" || action_name == "select");
                    if (request.value("expected_uid", "") != config_.expected_fc_uid ||
                        (!observation_action && !identity_permit())) {
                        throw std::runtime_error("expected UID/source identity mismatch");
                    }
                    if (action_name != "status" && !(observation_action && !identity_permit()) &&
                        !backend.validate_operator_request(request)) {
                        throw std::runtime_error(
                            "onboard session/FC epoch command context rejected");
                    }
                    if (action_name == "status") {
                        auto current = selector.snapshot(now);
                        NativeJson candidates = NativeJson::array();
                        if (current.latest_frame) {
                            for (const auto& track : current.latest_frame->tracks) {
                                if (track.real_detection && track.state == "CONFIRMED") {
                                    candidates.push_back(
                                        {{"generation", current.latest_frame->generation},
                                         {"sensor", current.latest_frame->source},
                                         {"track_id", track.track_id},
                                         {"class_id", track.class_id},
                                         {"label", track.label}});
                                }
                            }
                            // 2026-11-17: 같은 generation/source에서 확인됐지만 이번 프레임에서
                            // 잠깐 빠진 대상도 후보로 둔다(클릭한 순간 깜빡여도 GCS 신원 확인이
                            // 통과).
                            std::lock_guard<std::mutex> lock(recent_mutex);
                            for (const auto& [id, recent] : recent_tracks) {
                                if (recent.generation != current.latest_frame->generation ||
                                    recent.source != current.latest_frame->source) {
                                    continue;
                                }
                                const bool listed =
                                    std::any_of(candidates.begin(), candidates.end(),
                                                [id = id](const NativeJson& c) {
                                                    return c.at("track_id") == id;
                                                });
                                if (!listed) {
                                    candidates.push_back({{"generation", recent.generation},
                                                          {"sensor", recent.source},
                                                          {"track_id", id},
                                                          {"class_id", recent.track.class_id},
                                                          {"label", recent.track.label}});
                                }
                            }
                        }
                        const auto flight = flight_state.read();
                        const bool tracks_fresh =
                            current.latest_frame &&
                            now >= Clock::time_point{std::chrono::nanoseconds{
                                       current.latest_frame->sample_monotonic_ns}} &&
                            now - Clock::time_point{std::chrono::nanoseconds{
                                      current.latest_frame->sample_monotonic_ns}} <=
                                config_.selector.max_tracks_age;
                        const bool stopped_hold = control.inactive() &&
                                                  confirmed_stop_hold(flight.hold, flight.offboard,
                                                                      flight.offboard_hover) &&
                                                  !flight.rc_override && !scan.active() &&
                                                  !heading_stop_pending;
                        if (!tracks_fresh) {
                            candidates.clear();
                        }
                        NativeJson selected = nullptr;
                        if (current.target) {
                            selected = {{"generation", current.target->generation},
                                        {"track_id", current.target->track_id}};
                        }
                        auto status = NativeJson{
                            {"accepted", true},
                            {"observation_ready", true},
                            {"follow_environment", (guidance.yaw_only() ? yaw_profile_.guidance
                                                                        : moving_profile_.guidance)
                                                           .outdoor_profile
                                                       ? "outdoor"
                                                       : "indoor"},
                            {"follow_controller", guidance.yaw_only() ? "yaw_only"
                                                  : moving_profile_.guidance.outdoor_image_pid
                                                      ? "image_pid"
                                                      : "predictive"},
                            {"configured_follow_environment",
                             moving_profile_.guidance.outdoor_profile ? "outdoor" : "indoor"},
                            {"configured_follow_controller",
                             configured_follow_controller(moving_profile_)},
                            {"profile_replace_ready", control.inactive() && !scan.active() &&
                                                          !heading_stop_pending &&
                                                          !bench_active.load()},
                            {"selection_revision", current.target_revision},
                            {"tracking_only",
                             current.latest_frame && camera_only_tracks(*current.latest_frame)},
                            {"expected_uid", config_.expected_fc_uid},
                            {"controller_session", backend.controller_session()},
                            {"identity_ok", identity_permit()},
                            {"connected_uid", backend.verified_uid()},
                            {"connected", flight.connected},
                            {"armed", flight.armed},
                            {"in_air", flight.in_air},
                            {"ground", !flight.in_air},
                            {"ground_known", flight.ground_known},
                            {"landed", flight.landed},
                            {"actual_hold", flight.hold},
                            {"active", control.following_active_flag().load()},
                            {"bench_active", bench_active.load()},
                            {"bench_yaw_rate_deg_s", bench_rate},
                            {"bench_sdk_send_accepted", bench_sdk_accepted},
                            {"bench_zero_sdk_send_accepted", bench_zero_sdk_accepted},
                            {"bench_reason", bench_reason},
                            {"bench_fc_receive_confirmed", false},
                            {"mode", guidance.yaw_only() ? "yaw" : "moving"},
                            {"tracks_fresh", tracks_fresh},
                            {"offboard_hover", flight.offboard && flight.offboard_hover},
                            {"input_contract_v5",
                             current.latest_frame &&
                                 current.latest_frame->schema == kTracksV5Schema},
                            {"alignment_verified",
                             current.latest_frame && current.latest_frame->timestamp_source.find(
                                                         "unverified") == std::string::npos},
                            {"selected", selected},
                            {"controls",
                             {{"mode", stopped_hold || control.handoff_ready(now)},
                              {"select", control.inactive() && tracks_fresh && !scan.active() &&
                                             !heading_stop_pending},
                              {"start", flight.armed && flight.in_air},
                              {"stop", true},
                              {"search", stopped_hold && tracks_fresh},
                              {"reset_input", stopped_hold}}},
                            {"reasons",
                             {{"start", "비행 상태 확인 후 명시 요청"},
                              {"mode", "Stop 완료와 실제 HOLD 또는 명시 Position 인계 준비 필요"}}},
                            {"scan_progress_deg", scan.progress_deg()},
                            {"heading_stop_pending", heading_stop_pending},
                            {"reason", scan.reason()},
                            {"candidates", candidates}};
                        status.update(backend.operator_context());
                        if (backend.remote_fc_owner() && !identity_permit()) {
                            status["onboard_session"] = "observation-only";
                            status["fc_epoch"] = 0;
                        }
                        status.update(native_tracking_status(scan, current.target_phase));
                        status.update(projected_status(current, flight, now));
                        return status;
                    }
                    if (action_name == "bench_stop") {
                        if (!bench_active.load()) {
                            return NativeJson{{"accepted", false}, {"reason", "bench_not_active"}};
                        }
                        stop_bench("operator_stopped");
                        return NativeJson{{"accepted", true}, {"reason", bench_reason}};
                    }
                    if (action_name == "bench_yaw") {
                        const auto rate = request.value("yaw_rate_deg_s", 5.0);
                        const auto duration = request.value("duration_s", 3.0);
                        if (!ground_yaw_bench_request_valid(request.value("props_removed", false),
                                                            rate, duration) ||
                            bench_active.load() || !control.inactive() || scan.active() ||
                            heading_stop_pending || !identity_permit() || !bench_ground() ||
                            !backend.acquire()) {
                            throw std::runtime_error(
                                "bench requires props removed, ground fresh pose, inactive free "
                                "writer; rate <=10 and duration <=5");
                        }
                        bench_active.store(true);
                        bench_stopping = false;
                        bench_zero_sdk_accepted = false;
                        bench_rate = rate;
                        bench_until = now + std::chrono::duration_cast<Clock::duration>(
                                                std::chrono::duration<double>(duration));
                        FollowCommand command{};
                        command.yaw_rate_deg_s = rate;
                        update_context(now);
                        bench_sdk_accepted = vehicle.send_until(
                            command, std::min(bench_until, now + std::chrono::milliseconds(200)));
                        bench_reason = bench_sdk_accepted ? "sdk_accepted_fc_receipt_unconfirmed"
                                                          : "sdk_send_failed";
                        if (!bench_sdk_accepted) {
                            stop_bench(bench_reason);
                        }
                        return NativeJson{{"accepted", bench_sdk_accepted},
                                          {"reason", bench_reason},
                                          {"fc_receive_confirmed", false}};
                    }
                    if (bench_active.load() && action_name != "stop") {
                        throw std::runtime_error("stop ground yaw bench before other commands");
                    }
                    if (action_name == "stop") {
                        if (bench_active.load()) {
                            stop_bench("operator_stopped");
                        }
                        const bool recovery_requested =
                            backend.remote_fc_owner() && control.recovery_latched();
                        bool frozen = true;
                        if (scan.active() || heading_stop_pending ||
                            (backend.remote_fc_owner() &&
                             scan.state() == YawScanController::State::Fault)) {
                            scan.stop(pose.read().yaw_deg);
                            frozen = backend.acquire() && send_heading(scan.heading_deg(), true);
                            heading_stop_pending = !frozen;
                            if (!frozen) {
                                scan.fail("stop_heading_not_acknowledged");
                            }
                        }
                        auto result = selector.execute(
                            TargetCommand{request.at("request_id"), TargetCommandAction::Stop, 0, 0,
                                          config_.expected_fc_uid},
                            now);
                        update_context(now);
                        if (recovery_requested && result.accepted && frozen) {
                            const bool recovered =
                                identity_permit() && control.recover_after_explicit_stop(now);
                            return NativeJson{
                                {"accepted", recovered},
                                {"reason",
                                 recovered
                                     ? "explicit_stop_recovery_confirmed"
                                     : "recovery_requires_fresh_actual_hold_without_rc_override"}};
                        }
                        return NativeJson{{"accepted", result.accepted && frozen},
                                          {"reason", frozen ? result.reason : scan.reason()}};
                    }
                    if (heading_stop_pending) {
                        throw std::runtime_error(
                            "heading stop pending; explicit stop acknowledgment required");
                    }
                    if (scan.active()) {
                        throw std::runtime_error(
                            "stop scan before changing mode or starting follow");
                    }
                    if (action_name == "prepare_handoff") {
                        if (!identity_permit() || !control.inactive() || !backend.acquire()) {
                            throw std::runtime_error(
                                "handoff requires verified FC and free writer");
                        }
                        const bool ready = control.prepare_handoff(now);
                        backend.release();
                        return NativeJson{
                            {"accepted", ready},
                            {"reason", ready ? "handoff_ready_15_seconds"
                                             : "handoff_requires_fresh_Hold_or_Position"}};
                    }
                    if (action_name == "reset_input") {
                        const auto flight = flight_state.read();
                        const auto sample = pose.read();
                        if (!control.inactive() || !flight.connected || !flight.hold ||
                            flight.offboard || flight.rc_override || !sample.fresh(Clock::now())) {
                            throw std::runtime_error(
                                "input reset requires inactive fresh actual Hold");
                        }
                        const auto stopped = selector.execute(
                            TargetCommand{request.at("request_id"), TargetCommandAction::Stop, 0, 0,
                                          config_.expected_fc_uid},
                            now);
                        const bool reset = stopped.accepted && selector.reset_input_context();
                        update_context(now);
                        return NativeJson{
                            {"accepted", reset},
                            {"reason", reset ? "input_context_reset_explicit_reselection_required"
                                             : "input_reset_rejected"}};
                    }
                    if (action_name == "mode") {
                        const auto mode_name = request.at("mode").get<std::string>();
                        // 2026-11-18 데모 모드는 회전 추종(yaw)만 허용한다.
                        const auto demo_rejection = follow_mode_change_rejection(
                            moving_profile_.follow_controller_profile, mode_name);
                        if (!demo_rejection.empty()) {
                            throw std::runtime_error(demo_rejection);
                        }
                        const auto flight = flight_state.read();
                        if ((mode_name != "yaw" && mode_name != "moving") || !control.inactive() ||
                            !flight.connected || (!flight.hold && !control.handoff_ready(now)) ||
                            (flight.offboard && !control.handoff_ready(now)) ||
                            (flight.rc_override && !control.handoff_ready(now))) {
                            throw std::runtime_error(
                                "mode change requires inactive Hold or explicitly prepared "
                                "Position");
                        }
                        if (!backend.acquire()) {
                            throw std::runtime_error("another writer owns FC");
                        }
                        vehicle.revoke_commands();
                        const auto stopped = selector.execute(
                            TargetCommand{request.at("request_id"), TargetCommandAction::Stop, 0, 0,
                                          config_.expected_fc_uid},
                            now);
                        if (!stopped.accepted) {
                            backend.release();
                            throw std::runtime_error(stopped.reason);
                        }
                        guidance = FlightGuidanceController(
                            mode_name == "yaw" ? yaw_profile_.guidance : moving_profile_.guidance);
                        control.refresh_mode_guard();
                        backend.release();
                        return NativeJson{
                            {"accepted", true},
                            {"reason", "mode_selected_explicit_selection_and_start_required"}};
                    }
                    if (action_name == "search") {
                        ScanRequest scan_request;
                        const auto read_scan_parameter =
                            [&](const char* key) -> std::optional<double> {
                            if (!request.contains(key)) {
                                return std::nullopt;
                            }
                            if (!request.at(key).is_number()) {
                                throw std::invalid_argument("scan parameter must be numeric");
                            }
                            return request.at(key).get<double>();
                        };
                        scan_request.yaw_range_rad = read_scan_parameter("yaw_range_rad");
                        scan_request.yaw_rate_rad_s = read_scan_parameter("yaw_rate_rad_s");
                        scan_request.timeout_s = read_scan_parameter("timeout_s");
                        const auto requested_scan = parse_scan_request(scan_request, scan_defaults);
                        auto current = selector.snapshot(now);
                        if (!control.inactive() ||
                            current.phase == TargetSelectionSnapshot::Phase::Following ||
                            !current.latest_frame || !backend.acquire()) {
                            throw std::runtime_error(
                                "search requires completed Stop and free writer ownership");
                        }
                        if (!scan_health(now)) {
                            backend.release();
                            throw std::runtime_error("scan requires fresh actual Hold/video/pose");
                        }
                        vehicle.revoke_commands();
                        selector.execute(
                            TargetCommand{request.at("request_id"), TargetCommandAction::Stop, 0, 0,
                                          config_.expected_fc_uid},
                            now);
                        const auto yaw_pose = pose.read();
                        scan = YawScanController{requested_scan};
                        try {
                            scan.start(yaw_pose.yaw_deg, yaw_pose.attitude_at,
                                       std::max(Clock::now(), yaw_pose.snapshot_at),
                                       current.latest_frame->frame_id);
                        } catch (...) {
                            vehicle.revoke_commands();
                            backend.release();
                            throw;
                        }
                        next_heading_at = Clock::time_point{};
                        return NativeJson{
                            {"accepted", true},
                            {"reason", "manual_yaw_scan_started"},
                            {"applied_yaw_range_rad",
                             requested_scan.sweep_deg * 3.14159265358979323846 / 180.0},
                            {"applied_yaw_rate_rad_s",
                             requested_scan.rate_deg_s * 3.14159265358979323846 / 180.0},
                            {"applied_timeout_s", requested_scan.timeout.count() / 1000.0}};
                    }
                    if (action_name != "select" && action_name != "start" &&
                        action_name != "track_start") {
                        throw std::invalid_argument("unknown vMy command");
                    }
                    auto resolved_request = request;
                    if (action_name == "select" && (!control.inactive() || scan.active() ||
                                                    heading_stop_pending || bench_active.load())) {
                        throw std::runtime_error("selection requires inactive control");
                    }
                    if (action_name == "track_start" && !request.contains("label")) {
                        const auto current = selector.snapshot(now);
                        if (!current.latest_frame) {
                            throw std::runtime_error("ICD target requires current tracks");
                        }
                        try {
                            resolved_request = resolve_icd_target(request, *current.latest_frame);
                        } catch (const std::invalid_argument&) {
                            // 2026-11-17: 이번 프레임에서 빠진 대상은 같은 generation/source의
                            // 기억에서 label을 찾는다. 기억에도 없으면 기존 오류 그대로.
                            const auto recent =
                                request.at("generation").is_number_integer() &&
                                        request.at("track_id").is_number_integer() &&
                                        request.at("sensor").is_string()
                                    ? remembered_track(
                                          request.at("generation").get<std::uint64_t>(),
                                          request.at("sensor").get<std::string>(),
                                          request.at("track_id").get<std::int64_t>())
                                    : std::nullopt;
                            if (!recent || request.at("class_id") != recent->track.class_id) {
                                throw;
                            }
                            resolved_request["label"] = recent->track.label;
                        }
                    }
                    // selector.execute가 잠금 안에서 최신 프레임·label·클래스를 다시 검사한다.
                    const auto target_request =
                        native_target_request(resolved_request, config_.expected_fc_uid);
                    if (action_name == "track_start") {
                        const auto current = selector.snapshot(now);
                        const auto flight = flight_state.read();
                        if (!control.inactive() || !current.latest_frame ||
                            current.latest_frame->schema != kTracksV5Schema || !flight.connected ||
                            !flight.armed || !flight.in_air ||
                            (flight.offboard && !control.handoff_ready(now)) ||
                            (!flight.hold && !control.handoff_ready(now)) ||
                            (flight.rc_override && !control.handoff_ready(now))) {
                            throw std::runtime_error(
                                "atomic track start requires inactive fresh native target and "
                                "actual Hold or explicit Position handoff");
                        }
                        // 2026-11-17: 클릭한 대상이 이번 프레임에 없지만 같은 generation/source에서
                        // 확인된 대상이면 거부하지 않고 다시 잡힐 때까지 시작을 미룬다. 송신권은
                        // 실제 시작 때 잡는다.
                        if (!replaying_deferred_start && target_request.sensor &&
                            target_request.class_id && target_request.label) {
                            const auto& frame = *current.latest_frame;
                            const bool present =
                                frame.generation == target_request.generation &&
                                frame.source == *target_request.sensor &&
                                std::any_of(frame.tracks.begin(), frame.tracks.end(),
                                            [&](const TrackRecordV1& track) {
                                                return track.track_id == target_request.track_id &&
                                                       track.state == "CONFIRMED" &&
                                                       track.real_detection;
                                            });
                            const auto recent =
                                remembered_track(target_request.generation, *target_request.sensor,
                                                 target_request.track_id);
                            if (!present && recent &&
                                recent->track.class_id == *target_request.class_id &&
                                recent->track.label == *target_request.label) {
                                deferred_start = DeferredStart{resolved_request, now};
                                std::cerr
                                    << "TRACK_START deferred track_id=" << target_request.track_id
                                    << " waiting_for_selected_target_detection" << std::endl;
                                return NativeJson{
                                    {"accepted", true},
                                    {"reason", "waiting_for_selected_target_detection"}};
                            }
                        }
                        if (!backend.acquire()) {
                            throw std::runtime_error("another writer owns FC");
                        }
                    }
                    if (action_name == "start") {
                        auto current = selector.snapshot(now);
                        if (current.phase != TargetSelectionSnapshot::Phase::Pending ||
                            !current.latest_frame ||
                            current.latest_frame->schema != kTracksV5Schema) {
                            throw std::runtime_error(
                                "explicit selection and latest native v5 input required");
                        }
                        if (!backend.acquire()) {
                            throw std::runtime_error("another writer owns FC");
                        }
                    }
                    const auto result = selector.execute(target_request, now);
                    update_context(now);
                    if (!result.accepted &&
                        (action_name == "start" || action_name == "track_start")) {
                        backend.release();
                    }
                    return NativeJson{{"accepted", result.accepted}, {"reason", result.reason}};
                };
                native_operator->poll([&](const NativeJson& request) {
                    const auto action = request.value("action", "");
                    if (action != "status" && deferred_start) {
                        // 새 운용자 명령(중단, 다른 대상 시작 등)이 미룬 시작보다 우선한다.
                        command_progress.fail_pending(deferred_start->request,
                                                      "superseded_by_explicit_command");
                        deferred_start.reset();
                    }
                    const bool stop_pending = !control.inactive() || scan.active() ||
                                              heading_stop_pending || bench_active.load();
                    const bool tracked_command =
                        backend.remote_fc_owner() && request.contains("command_id") &&
                        (action == "track_start" || action == "search" || action == "stop");
                    NativeJson result;
                    try {
                        result = execute_operator(request);
                    } catch (const std::exception& error) {
                        if (tracked_command) {
                            command_progress.reject(request, error.what());
                        }
                        throw;
                    }
                    if (tracked_command && !result.value("accepted", false)) {
                        command_progress.reject(request,
                                                result.value("reason", "request_rejected"));
                    }
                    if (backend.remote_fc_owner() && request.contains("command_id") &&
                        result.value("accepted", false) &&
                        (action == "track_start" || action == "search" || action == "stop")) {
                        const bool pending = action != "stop" || stop_pending;
                        command_progress.begin(request, pending);
                        result["pending"] = pending;
                        result["execution_complete"] = !pending;
                    }
                    return result;
                });
                // 2026-11-17 미룬 추종 시작: 클릭한 대상이 다시 잡히면 같은 요청을 기존 검사
                // 그대로 다시 실행한다. 대상 문맥(generation/source)이 바뀌었거나 한도를 넘기면
                // 진행 중 결과를 실패로 닫는다.
                if (deferred_start) {
                    const auto& pending_request = deferred_start->request;
                    const auto generation = pending_request.at("generation").get<std::uint64_t>();
                    const auto source = pending_request.at("sensor").get<std::string>();
                    const auto track_id = pending_request.at("track_id").get<std::int64_t>();
                    const auto current = selector.snapshot(now);
                    const bool present =
                        current.latest_frame && current.latest_frame->generation == generation &&
                        current.latest_frame->source == source &&
                        std::any_of(current.latest_frame->tracks.begin(),
                                    current.latest_frame->tracks.end(),
                                    [&](const TrackRecordV1& track) {
                                        return track.track_id == track_id &&
                                               track.state == "CONFIRMED" && track.real_detection;
                                    });
                    if (present) {
                        const auto replay = pending_request;
                        deferred_start.reset();
                        replaying_deferred_start = true;
                        try {
                            const auto replayed = execute_operator(replay);
                            if (!replayed.value("accepted", false)) {
                                command_progress.fail_pending(
                                    replay, replayed.value("reason", "deferred_start_rejected"));
                            }
                            std::cerr << "TRACK_START deferred_replayed track_id=" << track_id
                                      << " accepted=" << replayed.value("accepted", false)
                                      << std::endl;
                        } catch (const std::exception& error) {
                            command_progress.fail_pending(replay, error.what());
                            std::cerr << "TRACK_START deferred_replay_failed track_id=" << track_id
                                      << " reason=" << error.what() << std::endl;
                        }
                        replaying_deferred_start = false;
                    } else if (!remembered_track(generation, source, track_id) ||
                               now - deferred_start->since > kDeferredStartWait) {
                        const char* reason = !remembered_track(generation, source, track_id)
                                                 ? "selected_target_context_changed"
                                                 : "selected_target_not_detected_within_wait";
                        command_progress.fail_pending(pending_request, reason);
                        std::cerr << "TRACK_START deferred_expired track_id=" << track_id
                                  << " reason=" << reason << std::endl;
                        deferred_start.reset();
                    }
                }
            }
            if (scan.active()) {
                const auto sampled_pose = pose.read();
                const auto current = selector.snapshot(now);
                bool candidate = false;
                if (current.latest_frame) {
                    for (const auto& track : current.latest_frame->tracks) {
                        candidate =
                            candidate || (track.real_detection && track.state == "CONFIRMED" &&
                                          track.last_real_detection_monotonic_ns ==
                                              current.latest_frame->sample_monotonic_ns);
                    }
                }
                const auto scan_now = std::max(Clock::now(), sampled_pose.snapshot_at);
                const bool heading_needed = scan.update(
                    sampled_pose.yaw_deg, sampled_pose.attitude_at, scan_now, scan_health(scan_now),
                    candidate, current.latest_frame ? current.latest_frame->frame_id : 0);
                bool freeze_attempted = false;
                bool freeze_acknowledged = false;
                if (heading_needed && (!scan.active() || scan_now >= next_heading_at)) {
                    freeze_attempted = !scan.active();
                    const bool acknowledged = send_heading(scan.heading_deg(), freeze_attempted);
                    freeze_acknowledged = freeze_attempted && acknowledged;
                    if (!acknowledged) {
                        scan.fail("scan_heading_not_acknowledged");
                    }
                    next_heading_at = Clock::now() + std::chrono::milliseconds{500};
                }
                if (!freeze_attempted && scan.state() == YawScanController::State::Fault) {
                    // A video failure must not prevent replacing the last heading once.
                    freeze_acknowledged = send_heading(pose.read().yaw_deg, true);
                    if (!freeze_acknowledged) {
                        scan.fail("scan_fault_heading_not_acknowledged");
                    }
                }
                if (!scan.active()) {
                    heading_stop_pending = !freeze_acknowledged;
                    if (heading_stop_pending) {
                        scan.fail("scan_heading_stop_pending_explicit_stop_required");
                    }
                    // No cached follow setpoint may survive a scan terminal event.
                    vehicle.revoke_commands();
                    backend.release();
                }
            }
            FlightControlRuntimeStatus tick;
            if (bench_active.load()) {
                if (bench_stopping) {
                    if (now >= bench_until || !bench_ground() || !identity_permit() ||
                        !backend.owns()) {
                        close_bench();
                    }
                } else if (now >= bench_until || !bench_ground() || !identity_permit() ||
                           !backend.owns()) {
                    stop_bench(now >= bench_until ? "duration_completed"
                                                  : "ground_precondition_lost");
                } else {
                    FollowCommand command{};
                    command.yaw_rate_deg_s = bench_rate;
                    update_context(now);
                    bench_sdk_accepted = vehicle.send_until(
                        command, std::min(bench_until, now + std::chrono::milliseconds(200)));
                    if (!bench_sdk_accepted) {
                        stop_bench("sdk_send_failed");
                    }
                }
                tick.mission = "GROUND_YAW_BENCH";
                tick.reason = bench_reason;
                tick.command_sent = bench_sdk_accepted;
                tick.command.yaw_rate_deg_s = bench_rate;
            } else {
                update_context(now);
                tick = control.step(now);
                // V3: 선택 문맥이 바뀌어 버려진 명령은 최신 상태로 한 번만 다시 계산한다.
                // 버린 후보를 다시 보내지 않으며, 설정에서 켠 기존 방식 추종에서만 쓴다.
                tick = bounded_discard_recheck(tick, guidance.v3_discard_recheck(), [&] {
                    const auto retry_now = Clock::now();
                    update_context(retry_now);
                    return control.step(retry_now);
                });
                update_context(Clock::now());
            }
            if (!bench_active.load() && !scan.active() &&
                (tick.mission == "FAULT" || tick.mission == "MANUAL" ||
                 tick.mission == "HOLD_CONFIRMED" ||
                 ((tick.mission == "IDLE" || tick.mission == "SELECTED") &&
                  selector.snapshot(now).phase != TargetSelectionSnapshot::Phase::Following))) {
                vehicle.revoke_commands();
                backend.release();
            }
            transition_lock.unlock();
            latest_runtime_status = tick;
            const auto control_done = Clock::now();
            const auto selection = selector.snapshot(now);
            const auto selector_done = Clock::now();
            const auto tracks_stats = tracks.stats();
            if ((config_.guidance.observed_path_rear_follow ||
                 config_.guidance.body_standoff_follow) &&
                !config_.diagnostics_csv.empty() && now >= next_guidance_diagnostic) {
                const auto path = std::filesystem::path(config_.diagnostics_csv).parent_path() /
                                  "gui-guidance.json";
                (void)write_guidance_diagnostic(path, tick, now, config_.guidance.range_deadband_m);
                next_guidance_diagnostic = now + std::chrono::milliseconds(100);
            }
            if (tracks_stats.last_error != last_tracks_error) {
                std::cerr << "tracks input error: " << tracks_stats.last_error << '\n';
                last_tracks_error = tracks_stats.last_error;
            }
            const auto vehicle_state = flight_state.read();
            auto status = make_status(tick, selection, vehicle_state, now);
            status.schema = kSystemStatusV2Schema;
            status.fc_uid = backend.verified_uid();
            status.identity_verified = !status.fc_uid.empty();
            auto backend_status = projected_status(selection, vehicle_state, now);
            backend_status.update(native_tracking_status(scan, selection.target_phase));
            backend_status["mission"] = tick.mission;
            backend_status["reason"] = tick.reason;
            backend_status["command_epoch"] = selection.command_epoch;
            backend_status["controller_session"] = backend.controller_session();
            backend_status["command_sent"] = tick.command_sent;
            backend_status["active"] = control.following_active_flag().load();
            backend_status["v2_1"] = {
                {"enabled", v21_enabled("ARGUS_V21_ENABLED")},
                {"box_motion_guard", v21_enabled("ARGUS_V21_BOX_MOTION_GUARD")},
                {"cv_gap", v21_enabled("ARGUS_V21_CV_GAP")},
                {"cv_visibility", v21_enabled("ARGUS_V21_CV_VISIBILITY")},
                {"vertical_center", v21_enabled("ARGUS_V21_VERTICAL_CENTER")},
                {"cv_gap_active", selection.cv_gap_active},
                {"box_motion_suspect", selection.box_motion_suspect},
                {"cv_origin_sample_ns", selection.cv_origin_sample_ns}};
            backend_status["scan_progress_deg"] = scan.progress_deg();
            backend_status["heading_stop_pending"] = heading_stop_pending;
            backend_status["selected"] = nullptr;
            if (selection.target) {
                backend_status["selected"] = {{"generation", selection.target->generation},
                                              {"track_id", selection.target->track_id}};
                if (selection.selected_track) {
                    backend_status["selected"]["class_id"] = selection.selected_track->class_id;
                    backend_status["selected"]["label"] = selection.selected_track->label;
                }
                if (selection.latest_frame) {
                    backend_status["selected"]["sensor"] = selection.latest_frame->source;
                }
            }
            try {
                backend.publish_status(backend_status);
                last_backend_status_error.clear();
            } catch (const std::exception& error) {
                loop_ok = false;
                if (last_backend_status_error != error.what()) {
                    std::cerr << "backend status publish failed: " << error.what() << '\n';
                    last_backend_status_error = error.what();
                }
            }
            drain_hold_diagnostics(now);
            if (!config_.diagnostics_csv.empty() &&
                (tick.predictive_input || tick.v21_motion ||
                 tick.predictive_diagnostic_capture_failed ||
                 tick.mission != last_decision_mission || tick.reason != last_decision_reason)) {
                const bool decision_transition =
                    tick.mission != last_decision_mission || tick.reason != last_decision_reason;
                try {
                    // Mode changes replace guidance; config_ alone may still describe the initial
                    // mode.
                    const auto& active_guidance =
                        guidance.yaw_only() ? yaw_profile_.guidance : moving_profile_.guidance;
                    // Capture immutable values after actuation; JSON construction runs on the
                    // worker.
                    std::function<NativeJson()> make_record =
                        [tick, now, decision_session_monotonic_ns, decision_session_unix_ns,
                         ground_down_m = config_.runtime.ground_down_m,
                         range_m = active_guidance.desired_range_m,
                         side_offset_m = active_guidance.follow_side_offset_m,
                         min_height_m = active_guidance.predictive_min_height_m,
                         max_height_m = active_guidance.predictive_max_height_m,
                         max_vertical_m_s = active_guidance.max_vertical_m_s,
                         height_anchor_m = active_guidance.predictive_height_anchor_m,
                         height_anchor_weight = active_guidance.predictive_height_anchor_weight,
                         height_anchor_deadband_m =
                             active_guidance.predictive_height_anchor_deadband_m,
                         vertical_deadband_m_s = active_guidance.predictive_vertical_deadband_m_s,
                         plant_horizontal = active_guidance.pampc_plant_horizontal_response,
                         plant_vertical = active_guidance.pampc_plant_vertical_response,
                         use_producer_ground = active_guidance.pampc_use_producer_ground,
                         model_log = active_guidance.pampc_model_log,
                         camera = active_guidance.pid_camera]() -> NativeJson {
                        NativeJson record = {
                            {"schema", "argus.follow.decision.v1"},
                            {"session_start_monotonic_ns", decision_session_monotonic_ns},
                            {"session_start_unix_ns_unverified", decision_session_unix_ns},
                            {"clock_domain", "host_monotonic"},
                            {"tick_monotonic_ns",
                             std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 now.time_since_epoch())
                                 .count()},
                            {"command_generated_ns", tick.command_generated_ns},
                            {"command_sample_ns", tick.command_sample_ns},
                            {"mission", tick.mission},
                            {"reason", tick.reason},
                            {"stop_trigger_reason", tick.stop_trigger_reason},
                            {"safety_event_code", static_cast<std::uint8_t>(tick.safety_event)},
                            {"command_sent", tick.command_sent},
                            {"predictive_input_capture_failed",
                             tick.predictive_diagnostic_capture_failed},
                            {"selection_epoch", tick.selection_command_epoch},
                            {"configuration",
                             {{"ground_down_m", ground_down_m},
                              {"range_m", range_m},
                              {"side_offset_m", side_offset_m},
                              {"predictive_min_height_m", min_height_m},
                              {"predictive_max_height_m", max_height_m},
                              {"max_vertical_m_s", max_vertical_m_s},
                              {"predictive_height_anchor_m", height_anchor_m},
                              {"predictive_height_anchor_weight", height_anchor_weight},
                              {"predictive_height_anchor_deadband_m", height_anchor_deadband_m},
                              {"predictive_vertical_deadband_m_s", vertical_deadband_m_s}}}};
                        // 2026-11-19 PAMPC 설정 값은 pampc_model_log=true일 때만 덧붙인다.
                        if (model_log) {
                            auto& configuration = record["configuration"];
                            configuration["pampc_plant_horizontal_response"] = plant_horizontal;
                            configuration["pampc_plant_vertical_response"] = plant_vertical;
                            configuration["pampc_use_producer_ground"] = use_producer_ground;
                        }
                        if (tick.v21_motion) {
                            const auto& v = *tick.v21_motion;
                            record["v2_1_motion"] = {
                                {"cv_gap_active", v.cv_gap_active},
                                {"box_motion_suspect", v.box_motion_suspect},
                                {"origin_sample_ns", v.origin_sample_ns},
                                {"deadline_ns", v.deadline_ns},
                                {"target_velocity_ne_m_s",
                                 {v.velocity_north_m_s, v.velocity_east_m_s}},
                                {"visibility_scale", v.visibility_scale},
                                {"velocity_unconfirmed", v.velocity_unconfirmed},
                                {"velocity_fit_motion_sample_ns", v.velocity_fit_sample_ns},
                                {"history_frame_id", v.history_frame_id},
                                {"history_source_ns", v.history_source_ns},
                                {"lateral", v.lateral},
                                {"lateral_delta_m_s", v.lateral_delta_m_s}};
                            if (v.visibility_diagnostics) {
                                const auto& trace = *v.visibility_diagnostics;
                                NativeJson detail = {
                                    {"shape_age_s", trace.shape_age_s},
                                    {"horizon_s", trace.horizon_s},
                                    {"tilt_model_requested", trace.tilt_model_requested},
                                    {"tilt_model_used", trace.tilt_model_used},
                                    {"selected_index", trace.selected_index},
                                    {"candidates", NativeJson::array()}};
                                for (std::size_t i = 0; i < trace.candidate_count; ++i) {
                                    const auto& candidate = trace.candidates[i];
                                    NativeJson entry = {
                                        {"scale", candidate.scale},
                                        {"forward", candidate.forward_m_s},
                                        {"right", candidate.right_m_s},
                                        {"score", candidate.score},
                                        {"valid", candidate.valid},
                                        {"lateral", candidate.lateral},
                                        {"lateral_delta_m_s", candidate.lateral_delta_m_s},
                                        {"worst_edge", candidate.worst_edge},
                                        {"max_range_error_m", candidate.max_range_error_m},
                                        {"predicted_boxes", NativeJson::array()},
                                        {"predicted_roll_pitch_deg", NativeJson::array()}};
                                    for (std::size_t j = 0; j < candidate.step_count; ++j) {
                                        const auto& step = candidate.steps[j];
                                        entry["predicted_boxes"].push_back({step.t_s, step.left,
                                                                            step.right, step.top,
                                                                            step.bottom});
                                        entry["predicted_roll_pitch_deg"].push_back(
                                            {step.t_s, step.roll_deg, step.pitch_deg});
                                    }
                                    detail["candidates"].push_back(std::move(entry));
                                }
                                record["v2_3_cv_visibility"] = std::move(detail);
                            }
                        }
                        record["camera_calibration"] = {
                            {"image_width", camera.image_width},
                            {"image_height", camera.image_height},
                            {"fx", camera.fx},
                            {"fy", camera.fy},
                            {"cx", camera.cx},
                            {"cy", camera.cy},
                            {"distortion", nullptr},
                            {"distortion_model", "not_supported_by_legacy_geometry"},
                            {"camera_to_body", camera.camera_to_body},
                            {"camera_offset_body_m", camera.camera_offset_body_m}};
                        if (tick.command_sent) {
                            record["sent_command"] = {
                                {"frame", "body_frd"},
                                {"vx_forward_m_s", tick.command.vx_m_s},
                                {"vy_right_m_s", tick.command.vy_m_s},
                                {"vz_down_m_s", tick.command.vz_m_s},
                                {"yaw_rate_deg_s", tick.command.yaw_rate_deg_s}};
                        }
                        if (tick.predictive_input && tick.predictive_vertical) {
                            const auto& in = *tick.predictive_input;
                            const auto& tr = in.track;
                            const auto& v = tick.predictive_vertical.value();
                            record["frame"] = {
                                {"source", in.source},
                                {"timestamp_source", in.timestamp_source},
                                {"frame_id", in.frame_id},
                                {"generation", in.generation},
                                {"sample_monotonic_ns", in.frame_sample_ns},
                                {"observation_pose_origin", in.observation_pose_origin},
                                {"width", in.width},
                                {"height", in.height}};
                            if (in.exposure_ns) {
                                record["frame"]["exposure_monotonic_ns"] = *in.exposure_ns;
                            }
                            record["target"] = {{"track_id", tr.track_id},
                                                {"mission_target_id", in.mission_target_id},
                                                {"command_epoch", in.command_epoch},
                                                {"class_id", tr.class_id},
                                                {"confidence", tr.confidence},
                                                {"state", tr.state},
                                                {"real_detection", tr.real_detection},
                                                {"last_real_detection_monotonic_ns",
                                                 tr.last_real_detection_monotonic_ns},
                                                {"bbox_xyxy_px", {tr.x1, tr.y1, tr.x2, tr.y2}},
                                                {"ground_motion_valid", in.ground_motion_valid},
                                                {"short_motion_valid", in.short_motion_valid},
                                                {"ground_motion_velocity_ne_m_s",
                                                 {in.target_vn_m_s, in.target_ve_m_s}},
                                                {"short_motion_velocity_ne_m_s",
                                                 {in.short_vn_m_s, in.short_ve_m_s}},
                                                {"short_motion_rmse_m", in.short_rmse_m},
                                                {"estimated_range_valid", in.range_valid},
                                                {"estimated_range_m", in.range_m},
                                                {"range_quality", in.range_quality},
                                                {"range_reason", in.range_reason},
                                                {"bearing_valid", in.bearing_valid},
                                                {"bearing_error_deg", in.bearing_error_deg}};
                            if (tr.ground) {
                                record["target"]["ground_anchor_ne_cov_alignment"] = *tr.ground;
                            }
                            if (tr.ground_anchor) {
                                record["target"]["ground_anchor_spec"] = *tr.ground_anchor;
                            }
                            const auto pose_json = [](const Pose& p) {
                                return NativeJson{
                                    {"ned_m", {p.north_m, p.east_m, p.down_m}},
                                    {"rpy_deg", {p.roll_deg, p.pitch_deg, p.yaw_deg}},
                                    {"velocity_ned_m_s",
                                     {p.velocity_north_m_s, p.velocity_east_m_s,
                                      p.velocity_down_m_s}},
                                    {"have_position", p.have_position},
                                    {"have_attitude", p.have_attitude},
                                    {"have_velocity", p.have_velocity},
                                    {"position_monotonic_ns", p.position_monotonic_ns},
                                    {"attitude_monotonic_ns", p.attitude_monotonic_ns}};
                            };
                            record["vehicle_pose"] = pose_json(in.vehicle_pose);
                            if (in.observation_pose) {
                                record["observation_pose"] = pose_json(*in.observation_pose);
                            }
                            record["vertical_decision"] = {
                                {"proposed_down_m_s", v.proposed_down_m_s},
                                {"limited_down_m_s", v.limited_down_m_s},
                                {"final_down_m_s", v.final_down_m_s},
                                {"previous_down_m_s", v.previous_down_m_s},
                                {"slew_m_s2", v.vertical_slew_m_s2},
                                {"height_reference_m", v.height_reference_m},
                                {"winning_plan_cost_before_slew", v.best_score},
                                {"cost_height", v.height_cost},
                                {"cost_visibility", v.visibility_cost},
                                {"cost_boundary", v.boundary_cost},
                                {"cost_other", v.other_cost},
                                {"cost_vertical_center", v.vertical_center_cost},
                                {"candidate_score_basis",
                                 "minimum evaluated coarse-grid score for each initial vertical "
                                 "seed; before refinement and not global optimum"},
                                {"safety_flags", v.safety_flags},
                                {"hold_down_m", in.hold_down_m}};
                            if (v.up_candidate_score) {
                                record["vertical_decision"]["candidate_up_score"] =
                                    *v.up_candidate_score;
                            }
                            if (v.zero_candidate_score) {
                                record["vertical_decision"]["candidate_zero_score"] =
                                    *v.zero_candidate_score;
                            }
                            if (v.down_candidate_score) {
                                record["vertical_decision"]["candidate_down_score"] =
                                    *v.down_candidate_score;
                            }
                            // 2026-11-19 PAMPC #3, #5 진단 (추가 필드, 기존 필드 의미는 그대로).
                            // pampc_model_log=false(기본)면 만들지 않는다 (기존 기록과 같다).
                            if (model_log) {
                                const auto& m = v.model;
                                const auto source_name = [](PampcGroundSource s) {
                                    return s == PampcGroundSource::Producer ? "producer"
                                           : s == PampcGroundSource::Held   ? "held"
                                                                            : "legacy";
                                };
                                static constexpr std::array<const char*, 6> kFallback{
                                    "none",      "absent",   "nonfinite",
                                    "deviation", "attitude", "stale"};
                                const auto fallback_index = static_cast<std::size_t>(m.fallback);
                                NativeJson model = {
                                    {"datum_source", source_name(m.datum_source)},
                                    {"attitude_source", source_name(m.attitude_source)},
                                    {"fallback", fallback_index < kFallback.size()
                                                     ? kFallback[fallback_index]
                                                     : "unknown"},
                                    {"ground_datum_down_m", m.ground_datum_down_m},
                                    {"target_datum_down_m", m.target_datum_down_m},
                                    {"height_ref_clamped", m.height_ref_clamped},
                                    {"height_ref_limit_m", m.height_ref_limit_m},
                                    {"held_ticks", m.held_ticks},
                                    {"height_ref_clamped_ticks", m.height_ref_clamped_ticks},
                                    {"projection_attitude_deg", m.projection_attitude_deg},
                                    {"plant_horizontal_response", m.plant_horizontal_response},
                                    {"plant_vertical_response", m.plant_vertical_response},
                                    {"producer_ticks", m.producer_ticks},
                                    {"fallback_ticks", m.fallback_ticks},
                                    {"plan_blocks_ned_m_s_yaw_deg_s", m.plan_blocks}};
                                if (m.producer_ground_down_m) {
                                    model["producer_ground_plane_down_m"] =
                                        std::isfinite(*m.producer_ground_down_m)
                                            ? NativeJson(*m.producer_ground_down_m)
                                            : NativeJson(nullptr);
                                }
                                if (m.producer_attitude_deg) {
                                    const auto& a = *m.producer_attitude_deg;
                                    model["producer_attitude_deg"] =
                                        std::isfinite(a[0] + a[1] + a[2]) ? NativeJson(a)
                                                                          : NativeJson(nullptr);
                                }
                                if (m.image_forecast_valid) {
                                    model["image_forecast_steps"] = {1, 3, 5, 10, 20};
                                    model["image_forecast_xy"] = m.image_forecast;
                                }
                                record["pampc_model"] = std::move(model);
                            }
                            if (tick.rear_goal) {
                                record["follow_goal"] = {{"north_m", tick.rear_goal->north_m},
                                                         {"east_m", tick.rear_goal->east_m},
                                                         {"admitted_target_velocity_ne_m_s",
                                                          {tick.rear_goal->velocity_north_m_s,
                                                           tick.rear_goal->velocity_east_m_s}}};
                            }
                            if (tick.altitude_target_down_m) {
                                record["vertical_decision"]["altitude_target_down_m"] =
                                    *tick.altitude_target_down_m;
                            }
                        }
                        return record;
                    };
                    bool queued;
                    if (decision_transition && tick.mission == "FAULT") {
                        queued = decision_log.try_record(make_record, true, false);
                        if (!queued && pending_hold_diagnostics.size() < 128) {
                            pending_hold_diagnostics.push_back(std::move(make_record));
                            queued = true;  // retained for nonblocking retry on the next tick
                        } else if (!queued) {
                            ++pending_hold_dropped;
                        }
                    } else {
                        queued = decision_log.try_record(std::move(make_record),
                                                         decision_transition ||
                                                             tick.mission == "FAULT" ||
                                                             tick.mission == "MANUAL");
                    }
                    if (queued) {
                        last_decision_mission = tick.mission;
                        last_decision_reason = tick.reason;
                    }
                } catch (...) {
                    decision_log.note_dropped();
                    std::cerr << "decision diagnostics: snapshot could not be queued\n";
                    // Diagnostics are never allowed to alter flight state or command ownership.
                }
            }
            if (diagnostics.is_open()) {
                const auto current_pose = pose.read();
                diagnostics
                    << std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch())
                           .count()
                    << ','
                    << (selection.latest_frame ? selection.latest_frame->sample_monotonic_ns : 0)
                    << ',' << tick.mission << ',' << tick.reason << ','
                    << selection.mission_target_id << ','
                    << (selection.target ? selection.target->track_id : 0) << ','
                    << target_phase_name(selection.target_phase) << ',' << tick.command_sent << ','
                    << current_pose.north_m << ',' << current_pose.east_m << ','
                    << current_pose.down_m << ',' << current_pose.yaw_deg << ','
                    << tick.command.vx_m_s << ',' << tick.command.vy_m_s << ','
                    << tick.command.vz_m_s << ',' << tick.command.yaw_rate_deg_s << ','
                    << std::chrono::duration<double, std::milli>(control_done - now).count() << ','
                    << std::chrono::duration<double, std::milli>(selector_done - control_done)
                           .count()
                    << ',';
                if (tick.guidance_ms) {
                    diagnostics << *tick.guidance_ms;
                }
                diagnostics << ',' << tick.command_sample_ns << ',' << tick.selection_command_epoch
                            << ',' << tick.manager_advance_ms << ',' << tick.command_generated_ns
                            << '\n';
                diagnostics.flush();
            }
            std::string publish_error;
            if (!status_publisher.publish(status, publish_error)) {
                loop_ok = false;
                std::cerr << "system/status publish failed: " << publish_error << '\n';
            }
            const std::string summary = tick.mission + ":" + tick.reason;
            if (summary != last_report) {
                std::cout << "mission=" << tick.mission << " reason=" << tick.reason
                          << " connected=" << vehicle_state.connected
                          << " armed=" << vehicle_state.armed << " in_air=" << vehicle_state.in_air
                          << " offboard=" << vehicle_state.offboard
                          << " target_state=" << status.target_state
                          << " mission_target_id=" << selection.mission_target_id
                          << " target_phase=" << target_phase_name(selection.target_phase)
                          << " target_revision=" << selection.target_revision
                          << " command_sent=" << tick.command_sent << '\n';
                last_report = summary;
            }
            if (event_driven) {
                // 새 추적 결과가 오면 바로(이전 tick 시작 뒤 20ms 이상), 없으면 기존 주기에 깬다.
                tick_wakeup.wait(now + config_.control_period, now + kMinEventTickInterval);
                next_tick = Clock::now();
                continue;
            }
            next_tick += config_.control_period;
            const auto after = Clock::now();
            if (next_tick < after) {
                next_tick = after + config_.control_period;
            }
            std::this_thread::sleep_until(next_tick);
        }
    } catch (...) {
        failure = std::current_exception();
    }

    // gateway join 완료 후 외부 Select/Start가 종료 제어에 들어오지 않는다.
    commands.stop();
    if (bench_active.load()) {
        stop_bench("shutdown");
        close_bench();
    }
    if (scan.active() || heading_stop_pending) {
        scan.stop(pose.read().yaw_deg);
        heading_stop_pending = !(backend.acquire() && send_heading(scan.heading_deg(), true));
        if (heading_stop_pending) {
            scan.fail("shutdown_heading_not_acknowledged");
            std::cerr << "shutdown heading stop not acknowledged; FC heading remains unconfirmed\n";
            loop_ok = false;
        }
        vehicle.revoke_commands();
        backend.release();
    }
    FlightControlRuntimeStatus last_tick;
    std::string last_shutdown_mission, last_shutdown_reason;
    try {
        // 종료도 Stop 사건으로 처리한다. 무장 해제·착륙은 수행하지 않고 Hold 결과만 한 번 확인한다.
        const auto stop_now = Clock::now();
        selector.stop_for_shutdown(stop_now);
        update_context(stop_now);
        const auto shutdown_deadline =
            stop_now + config_.runtime.hold_confirm_timeout + config_.runtime.prestream_interval;
        do {
            const auto now = Clock::now();
            update_context(now);
            last_tick = control.step(now);
            update_context(Clock::now());
            latest_runtime_status = last_tick;
            // Shutdown still produces real HOLD/FAULT outcomes. Preserve their original
            // snapshot and drain transport diagnostics without synchronous file I/O.
            drain_hold_diagnostics(now);
            if (!config_.diagnostics_csv.empty() &&
                (last_tick.mission != last_shutdown_mission ||
                 last_tick.reason != last_shutdown_reason || last_tick.offboard_stop_requested)) {
                try {
                    const auto snapshot = last_tick;
                    auto record = [snapshot, now, decision_session_monotonic_ns,
                                   decision_session_unix_ns] {
                        NativeJson value = {
                            {"schema", "argus.follow.decision.v1"},
                            {"clock_domain", "host_monotonic"},
                            {"session_start_monotonic_ns", decision_session_monotonic_ns},
                            {"session_start_unix_ns_unverified", decision_session_unix_ns},
                            {"tick_monotonic_ns",
                             std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 now.time_since_epoch())
                                 .count()},
                            {"mission", snapshot.mission},
                            {"reason", snapshot.reason},
                            {"command_sent", snapshot.command_sent},
                            {"selection_epoch", snapshot.selection_command_epoch},
                            {"safety_event_code", static_cast<int>(snapshot.safety_event)},
                            {"phase", "shutdown"}};
                        if (!snapshot.stop_trigger_reason.empty()) {
                            value["stop_trigger_reason"] = snapshot.stop_trigger_reason;
                        }
                        return value;
                    };
                    if (!decision_log.try_record(record, true, false)) {
                        if (pending_hold_diagnostics.size() < 128) {
                            pending_hold_diagnostics.push_back(std::move(record));
                        } else {
                            ++pending_hold_dropped;
                        }
                    }
                    last_shutdown_mission = last_tick.mission;
                    last_shutdown_reason = last_tick.reason;
                } catch (...) {
                    decision_log.note_critical_dropped(1);
                }
            }
            const auto final_selection = selector.snapshot(now);
            const auto final_flight = flight_state.read();
            auto final_backend_status = projected_status(final_selection, final_flight, now);
            final_backend_status.update(native_tracking_status(scan, final_selection.target_phase));
            final_backend_status["mission"] = last_tick.mission;
            final_backend_status["reason"] = last_tick.reason;
            final_backend_status["controller_session"] = backend.controller_session();
            final_backend_status["command_epoch"] = final_selection.command_epoch;
            final_backend_status["command_sent"] = last_tick.command_sent;
            final_backend_status["active"] = control.following_active_flag().load();
            final_backend_status["selected"] = nullptr;
            try {
                backend.publish_status(final_backend_status);
            } catch (const std::exception& error) {
                loop_ok = false;
                std::cerr << "backend shutdown status publish failed: " << error.what() << '\n';
            }
            auto status = make_status(last_tick, final_selection, final_flight, now);
            std::string ignored;
            if (!status_publisher.publish(status, ignored)) {
                loop_ok = false;
            }
            if (last_tick.mission != "HOLD_WAIT") {
                break;
            }
            std::this_thread::sleep_for(config_.runtime.prestream_interval);
        } while (Clock::now() < shutdown_deadline);
    } catch (...) {
        if (!failure) {
            failure = std::current_exception();
        }
    }

    watchdog.stop();
    tracks.stop();
    vehicle.revoke_commands();
    backend.release();
    drain_hold_diagnostics(Clock::now());
    const auto unsaved_critical = pending_hold_dropped + pending_hold_diagnostics.size();
    if (unsaved_critical) {
        // No wait on the command thread: report records that could not enter the logger.
        decision_log.note_critical_dropped(unsaved_critical);
    }
    if (failure) {
        std::rethrow_exception(failure);
    }
    return loop_ok && last_tick.mission != "FAULT" ? 0 : 1;
}
}  // namespace follow
