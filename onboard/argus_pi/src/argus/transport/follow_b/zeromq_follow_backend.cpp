#include "argus/transport/follow_b/include/zeromq_follow_backend.hpp"

#include <zmq.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>

#include "argus/bus/typed.hpp"
#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/control/contracts/include/mode_request_identity.hpp"
#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/transport/team_wire/include/pi_codec_table.hpp"
#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"
#include "argus_msg/control_setpoint.pb.h"
#include "argus_msg/pi_follow_status.pb.h"

namespace follow {
namespace {
std::int64_t ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
        .count();
}
Clock::time_point point(std::int64_t value) {
    return Clock::time_point(std::chrono::nanoseconds(value));
}
bool loopback(const std::string& value) { return value.rfind("tcp://127.0.0.1:", 0) == 0; }
std::string nonce() {
    std::random_device random;
    std::string result;
    constexpr char digits[] = "0123456789abcdef";
    for (int i = 0; i < 32; ++i) {
        result += digits[random() & 15];
    }
    return result;
}
void close_socket(void* socket) {
    if (socket) {
        zmq_close(socket);
    }
}
bool valid_stamp(std::int64_t stamp, std::int64_t now, int milliseconds) {
    return stamp > 0 && stamp <= now && now - stamp <= std::int64_t(milliseconds) * 1000000;
}
// 2026-11-25 batch 6a: 6370 은 팀 [이름][protobuf]. 받는 이름 (정확히 같아야 한다)
const std::vector<std::string_view>& fc_topics() {
    static const std::vector<std::string_view> kTopics{
        argus::team_wire::kFcAttitude, argus::team_wire::kFcState, argus::team_wire::kFcModeEvent};
    return kTopics;
}
constexpr int kTelemetryReceiveHwm = 128;  // 예전 RCVHWM 그대로

// 2026-11-25 batch 6b: 6372 (control/setpoint, follow/status) 는 팀 [이름][protobuf]. 요청 처리
// 스레드 (worker, run) 하나가 팀 ZmqBus 와 두 발행자를 만들고, 쓰고, 없앤다 (팀 ZmqBus 단일 스레드
// 규칙. 발행자만 있어 poller 에 등록되는 소켓이 없고 process_events 는 부르지 않는다). 두 토픽은
// 같은 엔드포인트라 PUB 소켓 하나를 함께 쓰므로 토픽 사이 순서는 보낸 순서 그대로 나간다. 큐 길이는
// 예전 SNDHWM 2 (팀 기본 4 가 아님). 팀 seq 는 토픽별 발행 번호, Pi 공용 카운터 seq 는 pi_seq.
// 변환 실패는 보내지 않고 false (pi_codec_table 이 세고 1, 2, 4 ... 번째를 남긴다. 팀 ZmqBus 도
// 실패마다 한 줄 남긴다)
using SetpointValue = argus::team_wire::PbJson<argus::msg::ControlSetpoint>;
using StatusValue = argus::team_wire::PbJson<argus::msg::PiFollowStatus>;
constexpr int kControlQueueDepth = 2;

class ControlBus {
public:
    explicit ControlBus(const std::string& endpoint)
        : bus_(context_, poller_, clock_,
               argus::bus_zmq::ZmqEndpointTable{
                   {std::string(argus::team_wire::kControlSetpoint), endpoint},
                   {std::string(argus::team_wire::kFollowStatus), endpoint}},
               argus::team_wire::pi_codec_table(stats_)),
          setpoint_(argus::bus::make_publisher<SetpointValue>(
              bus_, argus::team_wire::kControlSetpoint,
              argus::team_wire::pb_json_type<argus::msg::ControlSetpoint>(), kControlQueueDepth)),
          status_(argus::bus::make_publisher<StatusValue>(
              bus_, argus::team_wire::kFollowStatus,
              argus::team_wire::pb_json_type<argus::msg::PiFollowStatus>(), kControlQueueDepth)) {}

    // 두 발행자를 만들었는지 (bind 실패이면 false)
    bool ready() const { return setpoint_ != nullptr && status_ != nullptr; }
    bool publish_setpoint(NativeJson value) {
        return setpoint_->publish(SetpointValue{std::move(value), {}});
    }
    bool publish_status(NativeJson value) {
        return status_->publish(StatusValue{std::move(value), {}});
    }

private:
    zmq::context_t context_;
    argus::core::SteadyClock clock_;
    argus::bus_zmq::ZmqPoller poller_{clock_};
    std::shared_ptr<argus::team_wire::CodecStats> stats_ =
        std::make_shared<argus::team_wire::CodecStats>();
    argus::bus_zmq::ZmqBus bus_;
    std::unique_ptr<argus::bus::Publisher<SetpointValue>> setpoint_;
    std::unique_ptr<argus::bus::Publisher<StatusValue>> status_;
};
}  // namespace

struct ZeroMqFollowBackend::Impl {
    struct PosePort final : IVehicleStateSource {
        Impl& owner;
        explicit PosePort(Impl& value) : owner(value) {}
        Pose read() const override {
            std::lock_guard<std::mutex> lock(owner.mutex);
            return owner.pose_locked();
        }
        std::optional<Pose> read_aligned(std::int64_t stamp, const std::string& domain,
                                         std::chrono::nanoseconds limit) const override {
            if (domain != "host_monotonic" || stamp <= 0 || stamp > ns() || limit.count() < 0) {
                return std::nullopt;
            }
            std::lock_guard<std::mutex> lock(owner.mutex);
            if (!owner.fresh_locked()) {
                return std::nullopt;
            }
            auto output = owner.pose_locked();
            std::int64_t best_position = INT64_MAX, best_attitude = INT64_MAX;
            for (const auto& sample : owner.history["LOCAL_POSITION_NED"]) {
                const auto at = sample.at("received_monotonic_ns").get<std::int64_t>();
                auto delta = std::abs(at - stamp);
                if (valid_stamp(at, ns(), owner.config.telemetry_ttl_ms) && delta < best_position) {
                    best_position = delta;
                    const auto& data = sample.at("data");
                    output.north_m = data.at("x");
                    output.east_m = data.at("y");
                    output.down_m = data.at("z");
                    output.velocity_north_m_s = data.at("vx");
                    output.velocity_east_m_s = data.at("vy");
                    output.velocity_down_m_s = data.at("vz");
                    output.position_at = point(at);
                    output.position_monotonic_ns = at;
                }
            }
            for (const auto& sample : owner.history["ATTITUDE"]) {
                const auto at = sample.at("received_monotonic_ns").get<std::int64_t>();
                auto delta = std::abs(at - stamp);
                if (valid_stamp(at, ns(), owner.config.telemetry_ttl_ms) && delta < best_attitude) {
                    best_attitude = delta;
                    const auto& data = sample.at("data");
                    constexpr double degrees = 180.0 / 3.14159265358979323846;
                    output.roll_deg = data.at("roll").get<double>() * degrees;
                    output.pitch_deg = data.at("pitch").get<double>() * degrees;
                    output.yaw_deg = data.at("yaw").get<double>() * degrees;
                    output.attitude_at = point(at);
                    output.attitude_monotonic_ns = at;
                }
            }
            if (best_position > limit.count() || best_attitude > limit.count()) {
                return std::nullopt;
            }
            return output;
        }
    } pose_port{*this};
    struct StatePort final : IVehicleFlightStatusSource {
        Impl& owner;
        explicit StatePort(Impl& value) : owner(value) {}
        VehicleFlightStatus read() const override {
            std::lock_guard<std::mutex> lock(owner.mutex);
            return owner.fresh_locked() ? owner.flight : VehicleFlightStatus{};
        }
    } state_port{*this};
    struct VehiclePort final : IVehiclePort {
        Impl& owner;
        explicit VehiclePort(Impl& value) : owner(value) {}
        bool send(const FollowCommand&) override { return false; }
        bool send_until(const FollowCommand& value, Clock::time_point until) override {
            constexpr double radians = 3.14159265358979323846 / 180.0;
            if (!std::isfinite(value.vx_m_s) || !std::isfinite(value.vy_m_s) ||
                !std::isfinite(value.vz_m_s) || !std::isfinite(value.yaw_rate_deg_s)) {
                return false;
            }
            return owner.submit("velocity",
                                {{"vx_m_s", value.vx_m_s},
                                 {"vy_m_s", value.vy_m_s},
                                 {"vz_m_s", value.vz_m_s},
                                 {"yaw_rate_rad_s", value.yaw_rate_deg_s * radians},
                                 {"coordinate_frame", "body_frd"},
                                 {"cv_gap_active", value.mode == "GAP_CV"}},
                                until, false);
        }
        void revoke_commands() override { owner.revoke(); }
    } vehicle_port{*this};
    struct ModePort final : IFlightModePort {
        Impl& owner;
        explicit ModePort(Impl& value) : owner(value) {}
        bool start_offboard() override { return owner.mode_request("start_offboard"); }
        bool stop_offboard() override { return owner.mode_request("hold"); }
        std::optional<bool> start_result() const override {
            std::lock_guard<std::mutex> lock(owner.mutex);
            return owner.start_ack;
        }
        std::optional<bool> hold_result() const override {
            std::lock_guard<std::mutex> lock(owner.mutex);
            return owner.hold_ack;
        }
    } mode_port{*this};
    struct Job {
        std::string action;
        NativeJson request;
        Clock::time_point deadline;
        std::shared_ptr<std::promise<bool>> completion;
    };
    ZeroMqFollowBackendConfig config;
    mutable std::mutex mutex;
    std::mutex guard_mutex;
    std::deque<Job> jobs;
    // Bounded diagnostics only; admission and transport decisions never depend on it.
    std::deque<NativeJson> hold_diagnostics;
    std::uint64_t hold_diagnostics_dropped{0};
    NativeJson state, attitude;
    NativeJson history{{"ATTITUDE", NativeJson::array()},
                       {"LOCAL_POSITION_NED", NativeJson::array()},
                       {"DISTANCE_SENSOR", NativeJson::array()}};
    VehicleFlightStatus flight;
    FollowCommandContext command;
    std::function<bool()> guard;
    NativeJson latest_status, last_status;
    Clock::time_point last_status_at{};
    bool status_dirty{false};
    std::string client_session{nonce()}, onboard_session, token;
    std::uint64_t epoch{0}, sequence{0}, revision{0}, wire_command_epoch{0};
    bool authority{false};
    bool writer_requested{false};
    std::optional<bool> start_ack{false}, hold_ack{false};
    std::string pending_start, pending_hold;
    NativeJson pending_start_context, pending_hold_context;
    Clock::time_point pending_start_until{}, pending_hold_until{};
    std::uint64_t mode_event_sequence{0};
    // 보낸 뒤 응답을 아직 처리하지 않은 모드 요청 ID와, 그 사이 먼저 온 이벤트 (요청 하나씩만
    // 보낸다)
    std::string inflight_start, inflight_hold;
    std::map<std::string, NativeJson> early_mode_events;
    std::atomic<bool> quitting{false};
    // 2026-11-24 요청 처리(worker, run)와 FC 텔레메트리 수신(telemetry_worker, receive_loop)을
    // 나눈다. 예전에는 한 스레드가 둘을 다 해서, 요청이 응답을 기다리는 동안(REQ 수신 제한
    // rpc_timeout, 최대 250 ms) FC 자세와 상태를 받지 못했다. 공유 상태는 예전처럼 mutex
    // 안에서만 바꾼다.
    std::thread worker;
    std::thread telemetry_worker;
    // 작업이 들어오면 worker를 깨운다 (예전 2 ms poll 주기는 그대로)
    std::condition_variable jobs_ready;
    // context는 두 스레드가 함께 쓰고(zmq context는 스레드 안전) 두 스레드가 끝난 뒤 닫는다.
    // subscriber는 telemetry_worker에서만, control_bus와 요청마다 새로 만드는 REQ는 worker에서만
    // 만들고 쓰고 닫는다
    void* context{nullptr};
    void* subscriber{nullptr};
    // 6370 본문 해석 실패 수 (본문 상한 초과 포함). receive_loop 스레드가 올리고 아무 스레드나
    // 읽는다
    std::atomic<std::uint64_t> decode_failures{0};
    // 2026-11-25 batch 6b: 6372 발행 (worker 스레드만. run 이 만들고 끝날 때 없앤다)
    std::unique_ptr<ControlBus> control_bus;
    // 2026-11-25 batch 7: 6371 요청-응답의 팀 [control/mode_request][protobuf] 변환 (worker
    // 스레드만. rpc 는 process 와 poll_pending 에서만, 둘 다 run 안이다). 응답을 받지 못했거나
    // 해석하지 못한 수는 아무 스레드나 읽는다
    argus::team_wire::TeamWire rpc_wire;
    std::atomic<std::uint64_t> rpc_reply_failures{0};
    // 요청을 변환하지 못했거나 보내지 못한 수 (batch 7 리뷰)
    std::atomic<std::uint64_t> rpc_send_failures{0};

    explicit Impl(ZeroMqFollowBackendConfig value) : config(std::move(value)) {
        if (!loopback(config.telemetry_endpoint) || !loopback(config.rpc_endpoint) ||
            !loopback(config.control_endpoint) || config.expected_uid.size() != 36 ||
            !std::all_of(config.expected_uid.begin(), config.expected_uid.end(),
                         [](char value) {
                             return (value >= '0' && value <= '9') ||
                                    (value >= 'a' && value <= 'f');
                         }) ||
            config.rpc_timeout_ms < 1 || config.rpc_timeout_ms > 250 || config.command_ttl_ms < 1 ||
            config.command_ttl_ms > 250 || config.telemetry_ttl_ms < 1 ||
            config.telemetry_ttl_ms > 250 || config.source_max_age_ms < 1 ||
            config.source_max_age_ms > 250 || config.mode_result_timeout_ms < 1 ||
            config.mode_result_timeout_ms > 3000) {
            throw std::invalid_argument("invalid B backend loopback/safety config");
        }
        context = zmq_ctx_new();
        if (context == nullptr) {
            throw std::runtime_error("B backend sockets unavailable");
        }
        auto telemetry_started = std::make_shared<std::promise<bool>>();
        auto telemetry_ready = telemetry_started->get_future();
        // 스레드를 만들지 못하면(std::system_error) 만든 것만 정리하고 다시 던진다
        try {
            telemetry_worker =
                std::thread([this, telemetry_started] { receive_loop(telemetry_started); });
        } catch (...) {
            zmq_ctx_term(context);
            throw;
        }
        if (!telemetry_ready.get()) {
            quitting = true;
            telemetry_worker.join();
            zmq_ctx_term(context);
            throw std::runtime_error("B backend sockets unavailable");
        }
        // 읽기 전용(추적기의 FC 읽기)은 요청을 보내지 않으므로 수신 스레드만 돈다
        if (config.read_only) {
            return;
        }
        auto started = std::make_shared<std::promise<bool>>();
        auto ready = started->get_future();
        try {
            worker = std::thread([this, started] { run(started); });
        } catch (...) {
            // joinable한 telemetry_worker가 남아 소멸 때 std::terminate가 되지 않게 먼저 끝낸다
            quitting = true;
            telemetry_worker.join();
            zmq_ctx_term(context);
            throw;
        }
        if (!ready.get()) {
            quitting = true;
            jobs_ready.notify_all();
            worker.join();
            telemetry_worker.join();
            zmq_ctx_term(context);
            throw std::runtime_error("B backend sockets unavailable");
        }
    }
    ~Impl() {
        {
            std::lock_guard<std::mutex> lock(guard_mutex);
            guard = {};
        }
        quitting = true;
        jobs_ready.notify_all();
        if (worker.joinable()) {
            worker.join();
        }
        if (telemetry_worker.joinable()) {
            telemetry_worker.join();
        }
        zmq_ctx_term(context);
    }
    bool fresh_locked() const {
        auto now = ns();
        return !state.empty() && !attitude.empty() && flight.connected &&
               valid_stamp(state.value("received_monotonic_ns", std::int64_t(0)), now,
                           config.telemetry_ttl_ms) &&
               valid_stamp(state.value("position_monotonic_ns", std::int64_t(0)), now,
                           config.telemetry_ttl_ms) &&
               valid_stamp(state.value("distance_monotonic_ns", std::int64_t(0)), now,
                           config.telemetry_ttl_ms) &&
               valid_stamp(attitude.value("received_monotonic_ns", std::int64_t(0)), now,
                           config.telemetry_ttl_ms);
    }
    void hold_diagnostic_locked(const char* stage, const char* outcome,
                                const NativeJson& request = NativeJson{},
                                std::optional<bool> writer_ok = std::nullopt) noexcept {
        try {
            if (hold_diagnostics.size() >= 64) {
                ++hold_diagnostics_dropped;
                return;
            }
            const auto now = ns();
            const auto stamp = [](const NativeJson& object, const char* key) -> std::int64_t {
                if (!object.is_object() || !object.contains(key) ||
                    !object.at(key).is_number_integer()) {
                    return 0;
                }
                return object.at(key).get<std::int64_t>();
            };
            const auto stamp_state = stamp(state, "received_monotonic_ns");
            const auto stamp_position = stamp(state, "position_monotonic_ns");
            const auto stamp_distance = stamp(state, "distance_monotonic_ns");
            const auto stamp_attitude = stamp(attitude, "received_monotonic_ns");
            NativeJson item = {
                {"schema", "argus.follow.hold_diagnostic.v1"},
                {"clock_domain", "host_monotonic"},
                {"monotonic_ns", now},
                {"stage", stage},
                {"outcome", outcome},
                {"dropped_before", hold_diagnostics_dropped},
                {"read_only", config.read_only},
                {"queue_size", jobs.size()},
                {"queue_limit", 16},
                {"authority", authority},
                {"owner_token_present", !token.empty()},
                {"fc_fresh", fresh_locked()},
                {"fc_connected", flight.connected},
                {"fc_armed", flight.armed},
                {"fc_in_air", flight.in_air},
                {"fc_offboard", flight.offboard},
                {"fc_hold", flight.hold},
                {"fc_position", flight.position},
                {"fc_rc_override", flight.rc_override},
                {"fc_epoch", epoch},
                {"revision", revision},
                {"command_epoch", command.command_epoch},
                {"wire_command_epoch", wire_command_epoch},
                {"generation", command.generation},
                {"track_id", command.track_id},
                {"source_sample_monotonic_ns", command.source_sample_monotonic_ns},
                {"bench", command.bench},
                {"telemetry_ttl_ms", config.telemetry_ttl_ms},
                {"state_stamp_ns", stamp_state},
                {"position_stamp_ns", stamp_position},
                {"distance_stamp_ns", stamp_distance},
                {"attitude_stamp_ns", stamp_attitude},
                {"state_fresh", valid_stamp(stamp_state, now, config.telemetry_ttl_ms)},
                {"position_fresh", valid_stamp(stamp_position, now, config.telemetry_ttl_ms)},
                {"distance_fresh", valid_stamp(stamp_distance, now, config.telemetry_ttl_ms)},
                {"attitude_fresh", valid_stamp(stamp_attitude, now, config.telemetry_ttl_ms)}};
            if (writer_ok) {
                item["writer_permit"] = *writer_ok;
            }
            if (request.is_object() && !request.empty()) {
                item["request_id"] = request.value("request_id", std::string{});
                if (request.contains("original_request_id") &&
                    request.at("original_request_id").is_string()) {
                    item["original_request_id"] = request.at("original_request_id");
                }
                item["request_command_epoch"] = request.value("command_epoch", std::uint64_t{});
                item["request_generation"] = request.value("generation", std::uint64_t{});
                item["request_track_id"] = request.value("track_id", std::int64_t{});
                item["request_revision"] = request.value("revision", std::uint64_t{});
                item["request_fc_epoch"] = request.value("fc_epoch", std::uint64_t{});
                item["request_revision_matches"] =
                    request.value("revision", std::uint64_t{}) == revision;
                item["request_fc_epoch_matches"] =
                    request.value("fc_epoch", std::uint64_t{}) == epoch;
                item["request_command_epoch_nonzero"] =
                    request.value("command_epoch", std::uint64_t{}) != 0;
                item["request_owner_matches"] =
                    request.value("owner_token", std::string{}) == token && !token.empty();
                item["request_session_matches"] =
                    request.value("onboard_session", std::string{}) == onboard_session;
                item["request_source_not_future"] =
                    request.value("source_sample_monotonic_ns", std::int64_t{}) <= now;
                item["request_valid_until_ns"] =
                    request.value("valid_until_monotonic_ns", std::int64_t{});
                item["request_issued_ns"] = request.value("issued_monotonic_ns", std::int64_t{});
                if (request.value("cv_gap_active", false)) {
                    item["cv_gap_active"] = true;
                    item["cv_origin_sample_ns"] =
                        request.value("cv_origin_sample_ns", std::int64_t{});
                    item["cv_gap_deadline_monotonic_ns"] =
                        request.value("cv_gap_deadline_monotonic_ns", std::int64_t{});
                }
                item["request_not_expired"] =
                    request.value("valid_until_monotonic_ns", std::int64_t{}) > now;
                item["request_ttl_within_limit"] =
                    request.value("valid_until_monotonic_ns", std::int64_t{}) -
                        request.value("issued_monotonic_ns", std::int64_t{}) <=
                    std::int64_t(config.command_ttl_ms) * 1000000;
                item["fc_mode_allows_hold"] = flight.offboard || flight.hold || flight.position;
                item["request_bench"] = request.value("bench", false);
            }
            hold_diagnostics.push_back(std::move(item));
            hold_diagnostics_dropped = 0;
        } catch (...) {
            ++hold_diagnostics_dropped;
        }
    }
    void cancel_locked() {
        for (const auto& job : jobs) {
            if (job.action == "hold" ||
                (job.action == "velocity" && job.request.value("cv_gap_active", false))) {
                hold_diagnostic_locked("queue_cancelled", "context_or_authority_changed",
                                       job.request);
            }
        }
        if (!pending_hold.empty()) {
            hold_diagnostic_locked("pending_cancelled", "context_or_authority_changed",
                                   pending_hold_context);
        }
        ++revision;
        // Revocation/release are safety transactions, not stale motion candidates.
        jobs.erase(std::remove_if(jobs.begin(), jobs.end(),
                                  [](const Job& job) {
                                      return job.action != "revoke" && job.action != "release";
                                  }),
                   jobs.end());
        pending_start.clear();
        pending_hold.clear();
        start_ack = false;
        hold_ack = false;
    }
    void invalidate_locked() {
        authority = false;
        token.clear();
        cancel_locked();
    }
    Pose pose_locked() const {
        Pose output;
        if (!fresh_locked()) {
            return output;
        }
        constexpr double degrees = 180.0 / 3.14159265358979323846;
        output.north_m = state.at("north_m");
        output.east_m = state.at("east_m");
        output.down_m = state.at("down_m");
        output.velocity_north_m_s = state.at("vn_m_s");
        output.velocity_east_m_s = state.at("ve_m_s");
        output.velocity_down_m_s = state.at("vd_m_s");
        output.have_velocity = true;
        output.roll_deg = attitude.at("roll_rad").get<double>() * degrees;
        output.pitch_deg = attitude.at("pitch_rad").get<double>() * degrees;
        output.yaw_deg = attitude.at("yaw_rad").get<double>() * degrees;
        output.position_monotonic_ns = state.at("position_monotonic_ns");
        output.attitude_monotonic_ns = attitude.at("received_monotonic_ns");
        output.position_at = point(output.position_monotonic_ns);
        output.attitude_at = point(output.attitude_monotonic_ns);
        output.have_position = true;
        output.have_attitude = true;
        output.snapshot_at = Clock::now();
        return output;
    }
    void append(const char* kind, std::int64_t stamp, NativeJson data) {
        auto& values = history[kind];
        const auto position = std::lower_bound(
            values.begin(), values.end(), stamp, [](const NativeJson& row, std::int64_t time) {
                return row.at("received_monotonic_ns").get<std::int64_t>() < time;
            });
        if (position != values.end() && position->at("received_monotonic_ns") == stamp) {
            return;
        }
        NativeJson record = {{"received_monotonic_ns", stamp}, {"data", std::move(data)}};
        values.insert(position, std::move(record));
        if (values.size() > 64) {
            values.erase(values.begin());
        }
    }
    // 대기 중인 모드 요청과 맞는 이벤트면 결과를 정하고 true (조건은 예전 그대로)
    bool match_mode_event_locked(const NativeJson& input, const std::string& event) {
        for (bool start : {true, false}) {
            auto& id = start ? pending_start : pending_hold;
            auto& context = start ? pending_start_context : pending_hold_context;
            auto& ack = start ? start_ack : hold_ack;
            const auto until = start ? pending_start_until : pending_hold_until;
            if (id.empty() || input.at("command_id") != id || context.empty() ||
                input.at("revision") != context.at("revision") ||
                input.at("command_epoch") != context.at("command_epoch") ||
                context.at("revision") != revision || !authority || !fresh_locked() ||
                (input.contains("owner_token") && input.at("owner_token") != token) ||
                Clock::now() >= until ||
                input.at("received_monotonic_ns").get<std::int64_t>() <
                    context.at("issued_monotonic_ns").get<std::int64_t>()) {
                continue;
            }
            if (event == "MODE_COMPLETED" &&
                (input.at("actual_mode") == (start ? "OFFBOARD" : "HOLD") ||
                 (!start && input.at("actual_mode") == "OFFBOARD" &&
                  input.value("reason", "") == "offboard_hover_confirmed"))) {
                if (!start) {
                    hold_diagnostic_locked("mode_event", "completed", context);
                }
                ack = true;
                id.clear();
            } else if (event == "MODE_FAILED") {
                if (!start) {
                    const auto diagnostic_count = hold_diagnostics.size();
                    hold_diagnostic_locked("mode_event", "failed", context);
                    if (hold_diagnostics.size() > diagnostic_count) {
                        try {
                            if (input.contains("reason") && input.at("reason").is_string()) {
                                hold_diagnostics.back()["remote_reason"] = input.at("reason");
                            }
                            if (input.contains("mav_result") &&
                                input.at("mav_result").is_number_integer()) {
                                hold_diagnostics.back()["mav_result"] = input.at("mav_result");
                            }
                            if (input.contains("mav_result_param2") &&
                                input.at("mav_result_param2").is_number_integer()) {
                                hold_diagnostics.back()["mav_result_param2"] =
                                    input.at("mav_result_param2");
                            }
                        } catch (...) {
                        }
                    }
                }
                ack = false;
                id.clear();
            }
            return true;
        }
        return false;
    }
    void telemetry(const std::string& topic, const NativeJson& wire) {
        if (topic != "fc/attitude" && topic != "fc/state" && topic != "fc/mode_event") {
            return;
        }
        if (topic == "fc/mode_event" && config.read_only) {
            return;
        }
        auto input = wire;
        if (!input.contains("onboard_session") && input.contains("session_id")) {
            input["onboard_session"] = input.at("session_id");
        }
        if (topic == "fc/state") {
            for (const auto& pair : {std::make_pair("distance_m", "distance_cm"),
                                     std::make_pair("min_distance_m", "min_distance_cm"),
                                     std::make_pair("max_distance_m", "max_distance_cm")}) {
                if (!input.contains(pair.first)) {
                    continue;
                }
                if (!input.at(pair.first).is_number()) {
                    throw std::runtime_error("invalid SI distance");
                }
                const auto cm = input.at(pair.first).get<double>() * 100.0;
                if (!std::isfinite(cm) || (input.contains(pair.second) &&
                                           (!input.at(pair.second).is_number() ||
                                            !std::isfinite(input.at(pair.second).get<double>()) ||
                                            std::abs(input.at(pair.second).get<double>() - cm) >
                                                1e-6 * std::max(1.0, std::abs(cm))))) {
                    throw std::runtime_error("distance unit aliases conflict");
                }
                input[pair.second] = cm;
            }
        }
        auto now = ns();
        if (topic == "fc/mode_event") {
            // 지연된 이전 세션 이벤트가 현재 FC 문맥을 되돌리거나 새 모드를 승인하면 안 된다.
            try {
                std::lock_guard<std::mutex> lock(mutex);
                if (input.at("uid") != config.expected_uid ||
                    input.at("clock_domain") != "host_monotonic" ||
                    input.at("onboard_session") != onboard_session ||
                    input.at("fc_epoch") != epoch || !input.at("fc_epoch").is_number_unsigned() ||
                    !valid_stamp(input.at("received_monotonic_ns"), now,
                                 config.mode_result_timeout_ms)) {
                    return;
                }
            } catch (...) {
                return;
            }
        }
        if (input.at("uid") != config.expected_uid ||
            input.at("clock_domain") != "host_monotonic" ||
            !input.at("fc_epoch").is_number_unsigned() ||
            !input.at("onboard_session").is_string() ||
            input.at("onboard_session").get<std::string>().empty() ||
            !valid_stamp(input.at("received_monotonic_ns"), now,
                         topic == "fc/mode_event" ? config.mode_result_timeout_ms
                                                  : config.telemetry_ttl_ms)) {
            throw std::runtime_error("invalid FC identity/time");
        }
        auto incoming_session = input.at("onboard_session").get<std::string>();
        auto incoming_epoch = input.at("fc_epoch").get<std::uint64_t>();
        std::lock_guard<std::mutex> lock(mutex);
        if (incoming_session != onboard_session || incoming_epoch != epoch) {
            if (authority) {
                hold_diagnostic_locked("token_invalidation", "fc_session_or_epoch_changed");
            }
            invalidate_locked();
            state = NativeJson{};
            attitude = NativeJson{};
            for (auto& values : history.items()) {
                values.value() = NativeJson::array();
            }
            onboard_session = incoming_session;
            epoch = incoming_epoch;
            mode_event_sequence = 0;
        }
        if (input.contains("session_id") && input.at("session_id") != incoming_session) {
            throw std::runtime_error("FC session alias mismatch");
        }
        if (topic == "fc/mode_event") {
            if (!config.mode_events || input.at("schema") != "argus.fc.mode_event.v1" ||
                input.at("requester_session_id") != client_session ||
                !input.at("seq").is_number_unsigned() ||
                input.at("seq").get<std::uint64_t>() <= mode_event_sequence) {
                return;
            }
            mode_event_sequence = input.at("seq").get<std::uint64_t>();
            const auto event = input.at("event").get<std::string>();
            if (event == "MODE_EXIT") {
                if (authority && input.value("owner_token", "") == token) {
                    const auto reason = input.value("reason", "");
                    const bool pilot = reason.find("pilot") != std::string::npos ||
                                       reason == "rc_override" ||
                                       reason == "offboard_departure_requires_explicit_handoff";
                    const auto before = hold_diagnostics.size();
                    hold_diagnostic_locked("token_invalidation", "owner_mode_exit");
                    if (hold_diagnostics.size() > before) {
                        try {
                            hold_diagnostics.back()["remote_reason"] = reason;
                        } catch (...) {
                        }
                    }
                    invalidate_locked();
                    if (pilot) {
                        flight.rc_override = true;
                    }
                }
                return;
            }
            if (!match_mode_event_locked(input, event)) {
                // 2026-11-24 응답보다 먼저 온 이벤트 (요청 처리와 수신 스레드가 나뉘어 생길 수
                // 있다. 게이트웨이는 이미 그 모드면 응답 직후 바로 완료를 보낸다). 그 요청의 응답을
                // 처리할 때 같은 조건으로 맞춘다
                const auto command_id = input.value("command_id", std::string{});
                if (!command_id.empty() &&
                    (command_id == inflight_start || command_id == inflight_hold)) {
                    early_mode_events[command_id] = input;
                }
            }
            return;
        }
        auto finite = [&input](const char* field) {
            if (!input.at(field).is_number() || !std::isfinite(input.at(field).get<double>())) {
                throw std::runtime_error("nonfinite FC sample");
            }
        };
        if (topic == "fc/attitude" && input.at("schema") == "argus.fc.attitude.v1") {
            for (auto field : {"roll_rad", "pitch_rad", "yaw_rad"}) {
                finite(field);
            }
            auto stamp = input.at("received_monotonic_ns").get<std::int64_t>();
            if (!attitude.empty() &&
                stamp <= attitude.at("received_monotonic_ns").get<std::int64_t>()) {
                return;
            }
            attitude = input;
            NativeJson attitude_sample{
                {"roll", input.at("roll_rad")},
                {"pitch", input.at("pitch_rad")},
                {"yaw", input.at("yaw_rad")},
                {"time_boot_ms", input.value("fc_time_boot_ms", NativeJson{})}};
            // 2026-11-16: 외삽용 몸체 각속도 (게이트웨이가 이미 보냄, 유한한 수만)
            for (const auto& [from, to] :
                 {std::pair<const char*, const char*>{"roll_rad_s", "rollspeed"},
                  {"pitch_rad_s", "pitchspeed"},
                  {"yaw_rad_s", "yawspeed"}}) {
                if (input.contains(from) && input.at(from).is_number() &&
                    std::isfinite(input.at(from).get<double>())) {
                    attitude_sample[to] = input.at(from);
                }
            }
            append("ATTITUDE", stamp, std::move(attitude_sample));
        } else if (topic == "fc/state" && input.at("schema") == "argus.fc.state.v1") {
            for (auto field :
                 {"north_m", "east_m", "down_m", "vn_m_s", "ve_m_s", "vd_m_s", "distance_cm",
                  "min_distance_cm", "max_distance_cm", "flow_quality"}) {
                finite(field);
            }
            const auto pos = input.at("position_monotonic_ns").get<std::int64_t>();
            const auto distance = input.at("distance_monotonic_ns").get<std::int64_t>();
            if (!valid_stamp(pos, now, config.telemetry_ttl_ms) ||
                !valid_stamp(distance, now, config.telemetry_ttl_ms)) {
                throw std::runtime_error("expired FC samples");
            }
            if (!state.empty() && input.at("received_monotonic_ns").get<std::int64_t>() <=
                                      state.at("received_monotonic_ns").get<std::int64_t>()) {
                return;
            }
            state = input;
            if ((input.value("flight_environment", "") == "indoor" ||
                 input.value("flight_environment", "") == "outdoor") &&
                input.contains("cmc_pose_history") && input["cmc_pose_history"].is_object()) {
                for (const auto* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
                    const auto& batch = input["cmc_pose_history"].value(kind, NativeJson::array());
                    if (!batch.is_array() || batch.size() > 16) {
                        continue;
                    }
                    for (const auto& record : batch) {
                        if (!record.is_object() || !record.contains("data") ||
                            !record["data"].is_object() ||
                            !record.contains("received_monotonic_ns") ||
                            !valid_stamp(record["received_monotonic_ns"], now,
                                         config.telemetry_ttl_ms)) {
                            continue;
                        }
                        const auto& raw = record["data"];
                        std::vector<std::string> keys;
                        if (std::string(kind) == "ATTITUDE") {
                            keys = {"roll", "pitch", "yaw"};
                        } else if (std::string(kind) == "LOCAL_POSITION_NED") {
                            keys = {"x", "y", "z", "vx", "vy", "vz"};
                        } else {
                            keys = {"orientation", "current_distance", "min_distance",
                                    "max_distance"};
                        }
                        NativeJson sample = NativeJson::object();
                        bool valid = true;
                        for (const auto& key : keys) {
                            if (!raw.contains(key) || !raw[key].is_number() ||
                                !std::isfinite(raw[key].get<double>())) {
                                valid = false;
                                break;
                            }
                            sample[key] = raw[key];
                        }
                        if (!valid) {
                            continue;
                        }
                        // 2026-11-16: 외삽용 몸체 각속도 (있으면, 유한한 수만)
                        if (std::string(kind) == "ATTITUDE") {
                            for (const char* rate : {"rollspeed", "pitchspeed", "yawspeed"}) {
                                if (raw.contains(rate) && raw[rate].is_number() &&
                                    std::isfinite(raw[rate].get<double>())) {
                                    sample[rate] = raw[rate];
                                }
                            }
                        }
                        if (raw.contains("time_boot_ms") &&
                            raw["time_boot_ms"].is_number_integer() &&
                            raw["time_boot_ms"].get<std::int64_t>() >= 0 &&
                            raw["time_boot_ms"].get<std::uint64_t>() <= UINT32_MAX) {
                            sample["time_boot_ms"] = raw["time_boot_ms"];
                        }
                        append(kind, record["received_monotonic_ns"].get<std::int64_t>(),
                               std::move(sample));
                    }
                }
            }
            flight = {input.at("connected"), input.at("armed"),
                      input.at("in_air"),    input.at("offboard"),
                      input.at("hold"),      input.at("rc_override"),
                      input.at("position"),  input.at("ground_known"),
                      input.at("landed"),    input.value("offboard_hover", false)};
            if (!flight.connected || flight.rc_override ||
                (flight.ground_known && flight.landed && !command.bench)) {
                if (authority) {
                    hold_diagnostic_locked("token_invalidation", "fc_state_invalid_or_override");
                }
                invalidate_locked();
            }
            append("LOCAL_POSITION_NED", pos,
                   {{"x", input.at("north_m")},
                    {"y", input.at("east_m")},
                    {"z", input.at("down_m")},
                    {"vx", input.at("vn_m_s")},
                    {"vy", input.at("ve_m_s")},
                    {"vz", input.at("vd_m_s")},
                    {"time_boot_ms", input.value("position_time_boot_ms", NativeJson{})}});
            append("DISTANCE_SENSOR", distance,
                   {{"orientation", input.at("distance_orientation")},
                    {"current_distance", input.at("distance_cm")},
                    {"min_distance", input.at("min_distance_cm")},
                    {"max_distance", input.at("max_distance_cm")},
                    {"time_boot_ms", input.value("distance_time_boot_ms", NativeJson{})}});
        } else {
            throw std::runtime_error("unexpected telemetry schema");
        }
    }
    NativeJson envelope_locked(const std::string& action) {
        return {{"schema", "argus.follow.rpc.v1"},
                {"source", "argus-follow-controller"},
                {"request_id", client_session + "-" + std::to_string(++sequence)},
                {"client_session", client_session},
                {"session_id", client_session},
                {"command_id", client_session + "-" + std::to_string(sequence)},
                {"action", action},
                {"uid", config.expected_uid},
                {"clock_domain", "host_monotonic"},
                {"onboard_session", onboard_session},
                {"fc_epoch", epoch},
                {"owner_token", token},
                {"command_epoch", command.command_epoch ? wire_command_epoch : 0},
                {"generation", command.generation},
                {"track_id", command.track_id},
                {"source_sample_monotonic_ns", command.source_sample_monotonic_ns},
                {"bench", command.bench},
                {"issued_monotonic_ns", ns()},
                {"seq", sequence},
                {"revision", revision}};
    }
    bool writer_permit() {
        // Callback may call identity_permit()/owns()/pose(): never hold state mutex.
        // Clearing the callback waits for an in-flight call before runtime teardown.
        std::lock_guard<std::mutex> lock(guard_mutex);
        try {
            return guard && guard();
        } catch (...) {
            return false;
        }
    }
    bool command_permit_locked(const NativeJson& request) const {
        auto now = ns();
        const bool stopping_heading =
            request.at("action") == "heading" && request.value("stopping", false) && flight.hold;
        const bool safety_hold = request.at("action") == "hold";
        const bool zero = request.at("action") == "velocity" && request.at("vx_m_s") == 0 &&
                          request.at("vy_m_s") == 0 && request.at("vz_m_s") == 0 &&
                          request.at("yaw_rate_rad_s") == 0;
        const bool actual_mode =
            request.at("action") == "velocity"
                ? (flight.offboard || (zero && (flight.hold || flight.position)))
                : (request.at("action") == "heading"
                       ? flight.hold
                       : (flight.offboard || flight.hold || flight.position));
        const bool cv = request.value("cv_gap_active", false);
        const auto source = request.at("source_sample_monotonic_ns").get<std::int64_t>();
        bool cv_valid = false;
        if (cv && v21_enabled("ARGUS_V21_CV_GAP") && request.at("action") == "velocity" &&
            command.cv_gap_active && request.contains("cv_origin_sample_ns") &&
            request.contains("cv_gap_deadline_monotonic_ns")) {
            const auto origin = request.at("cv_origin_sample_ns").get<std::int64_t>();
            const auto deadline = request.at("cv_gap_deadline_monotonic_ns").get<std::int64_t>();
            cv_valid = origin > 0 && origin <= now && deadline > origin &&
                       deadline - origin == INT64_C(3000000000) && now < deadline &&
                       request.at("valid_until_monotonic_ns").get<std::int64_t>() <= deadline &&
                       source == origin && command.cv_origin_sample_ns == origin &&
                       command.cv_gap_deadline_monotonic_ns == deadline;
        }
        return authority && fresh_locked() && !flight.rc_override &&
               request.at("owner_token") == token && actual_mode && source <= now &&
               (!cv || cv_valid) && request.at("revision") == revision &&
               request.at("fc_epoch") == epoch &&
               request.at("onboard_session") == onboard_session &&
               (safety_hold || stopping_heading || zero || cv_valid ||
                valid_stamp(source, now, config.source_max_age_ms)) &&
               request.at("command_epoch").get<std::uint64_t>() != 0 &&
               !request.at("bench").get<bool>() && (flight.armed && flight.in_air) &&
               request.at("valid_until_monotonic_ns").get<std::int64_t>() > now &&
               request.at("valid_until_monotonic_ns").get<std::int64_t>() -
                       request.at("issued_monotonic_ns").get<std::int64_t>() <=
                   std::int64_t(config.command_ttl_ms) * 1000000;
    }
    bool enqueue(Job job, bool wait) {
        if (config.read_only) {
            if (job.action == "hold") {
                std::lock_guard<std::mutex> lock(mutex);
                hold_diagnostic_locked("queue", "read_only", job.request);
            }
            return false;
        }
        auto result = job.completion->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (jobs.size() >= 16) {
                if (job.action == "hold") {
                    hold_diagnostic_locked("queue", "queue_full", job.request);
                }
                return false;
            }
            if (job.action == "hold") {
                hold_diagnostic_locked("queue", "accepted", job.request);
            }
            jobs.push_back(std::move(job));
        }
        jobs_ready.notify_one();
        if (!wait) {
            return true;
        }
        try {
            return result.wait_for(std::chrono::milliseconds(config.rpc_timeout_ms + 30)) ==
                       std::future_status::ready &&
                   result.get();
        } catch (...) {
            return false;
        }
    }
    bool submit(const std::string& action, NativeJson fields, Clock::time_point until,
                bool stopping) {
        if (!writer_permit()) {
            revoke();
            return false;
        }
        NativeJson request;
        {
            std::lock_guard<std::mutex> lock(mutex);
            request = envelope_locked(action);
            // Runtime priming may request 3 intervals (300 ms). Shorten at this
            // transport boundary; never extend a caller deadline or exceed B TTL.
            const auto transport_until =
                Clock::time_point(std::chrono::nanoseconds(
                    request.at("issued_monotonic_ns").get<std::int64_t>())) +
                std::chrono::milliseconds(config.command_ttl_ms);
            until = std::min(until, transport_until);
            request["valid_until_monotonic_ns"] =
                std::chrono::duration_cast<std::chrono::nanoseconds>(until.time_since_epoch())
                    .count();
            request["stopping"] = stopping;
            request.update(fields);
            if (request.value("cv_gap_active", false)) {
                if (!command.cv_gap_active) {
                    return false;
                }
                request["cv_origin_sample_ns"] = command.cv_origin_sample_ns;
                request["cv_gap_deadline_monotonic_ns"] = command.cv_gap_deadline_monotonic_ns;
            }
            if (!command_permit_locked(request)) {
                return false;
            }
        }
        // 속도 경로는 RPC 응답을 기다리지 않는다. true는 큐 수락이며,
        // 실제 송신 직전에 actor가 제어권·문맥·기한을 다시 검사한다.
        return enqueue({action, request, until, std::make_shared<std::promise<bool>>()},
                       action != "velocity");
    }
    bool mode_request(const std::string& action) {
        const bool writer_ok = writer_permit();
        if (!writer_ok) {
            if (action == "hold") {
                std::lock_guard<std::mutex> lock(mutex);
                hold_diagnostic_locked("precheck", "writer_permit_denied", NativeJson{}, writer_ok);
            }
            return false;
        }
        NativeJson request;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!authority || !fresh_locked()) {
                if (action == "hold") {
                    hold_diagnostic_locked("precheck", "authority_or_freshness_denied",
                                           NativeJson{}, writer_ok);
                }
                return false;
            }
            request = envelope_locked(action);
            request["valid_until_monotonic_ns"] =
                request.at("issued_monotonic_ns").get<std::int64_t>() +
                std::int64_t(config.command_ttl_ms) * 1000000;
            if (!command_permit_locked(request)) {
                if (action == "hold") {
                    hold_diagnostic_locked("precheck", "command_permit_denied", request, writer_ok);
                }
                return false;
            }
            if (action == "hold") {
                hold_diagnostic_locked("precheck", "accepted", request, writer_ok);
            }
            if (action == "start_offboard") {
                start_ack = std::nullopt;
            } else {
                hold_ack = std::nullopt;
            }
        }
        bool queued = enqueue(
            {action, request, Clock::now() + std::chrono::milliseconds(config.command_ttl_ms),
             std::make_shared<std::promise<bool>>()},
            false);
        if (!queued) {
            std::lock_guard<std::mutex> lock(mutex);
            (action == "start_offboard" ? start_ack : hold_ack) = false;
        }
        return queued;
    }
    void revoke(bool relinquish = false) {
        NativeJson request;
        {
            std::lock_guard<std::mutex> lock(mutex);
            // Before the first acquisition attempt there is no remote command to cancel.
            // Keep local invalidation; once acquisition is attempted preserve all revocations.
            if (!writer_requested) {
                cancel_locked();
                return;
            }
            request = envelope_locked("revoke");
            request["cancel_request_ids"] = NativeJson::array();
            for (const auto& id : {pending_start, pending_hold}) {
                if (!id.empty()) {
                    request["cancel_request_ids"].push_back(id);
                }
            }
            cancel_locked();
            request["next_revision"] = revision;
            // Cancel cached commands without releasing the independent owner lease.
            // On teardown retain the token so release() can return remote ownership.
            if (relinquish) {
                authority = false;
            }
        }
        enqueue({"revoke", request, Clock::now() + std::chrono::milliseconds(config.rpc_timeout_ms),
                 std::make_shared<std::promise<bool>>()},
                false);
    }
    // 6371 요청 하나 (worker 스레드). 2026-11-25 batch 7: 팀 규약 [control/mode_request]
    // [ControlModeRequest] 하나로 보내고 [control/mode_request][ControlModeReply] 를 받는다 (모드
    // 요청과 제어권 RPC 모두. 예전 한 프레임 msgpack RPC 와 control/mode_result 이름은 없앴다).
    // 요청마다 REQ 를 새로 만든다 (예전 그대로): 기한 (rpc_timeout) 이 지나 닫은 소켓의 늦은 응답은
    // 다음 요청에 닿지 않는다. 팀 ZmqRequester 를 쓰지 않는 까닭: 응답을 process_events 의 handler
    // 로 돌려주므로 process 안에서 기다리려면 이 스레드에서 poller 를 따로 돌려야 하고, REQ 하나를
    // 계속 쓰는 (REQ_RELAXED, REQ_CORRELATE) 동작이 예전의 늦은 응답 폐기와 다르다. 변환은
    // team_wire (encode_body, receive_raw 의 이름 정확 일치와 프레임 수, 요청-응답 본문 상한 256
    // KiB. 예전 응답 상한 64 KiB 보다 넓다). 빈 본문 (팀 replier 와 게이트웨이의 처리 실패 응답) 과
    // 해석 실패는 응답 없음과 같다. 그 뒤의 신원, 시각 검사는 예전 그대로다
    NativeJson rpc(const NativeJson& request) {
        void* socket = zmq_socket(context, ZMQ_REQ);
        if (!socket) {
            return NativeJson::object();
        }
        int zero = 0, timeout = config.rpc_timeout_ms;
        zmq_setsockopt(socket, ZMQ_LINGER, &zero, sizeof(zero));
        zmq_setsockopt(socket, ZMQ_SNDTIMEO, &timeout, sizeof(timeout));
        zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
        NativeJson response = NativeJson::object();
        if (zmq_connect(socket, config.rpc_endpoint.c_str()) == 0) {
            const auto action = request.at("action").get<std::string>();
            const bool mode_wire = config.mode_events && (action == "start_offboard" ||
                                                          action == "hold" || action == "heading");
            auto wire = request;
            if (mode_wire) {
                wire["schema"] = "argus.control.mode_request.v1";
                wire["mode"] = action == "start_offboard" ? "OFFBOARD"
                               : action == "hold"         ? "HOLD"
                                                          : "HOLD_HEADING";
            }
            const zmq::socket_ref ref(zmq::from_handle, socket);
            if (rpc_wire.send_request(ref, argus::team_wire::kControlModeRequest, wire)) {
                NativeJson reply;
                const auto status = rpc_wire.receive_reply(
                    ref, argus::team_wire::kControlModeRequest, reply, zmq::recv_flags::none);
                if (status != argus::team_wire::ReceiveStatus::Ok) {
                    // 기한 안에 응답이 없으면 NoMessage (예전처럼 따로 남기지 않는다)
                    if (status != argus::team_wire::ReceiveStatus::NoMessage) {
                        const std::uint64_t count = ++rpc_reply_failures;
                        if ((count & (count - 1)) == 0) {
                            std::cerr << "follow 6371 응답 거부 (누적 " << count
                                      << "): " << argus::team_wire::to_string(status) << ' '
                                      << rpc_wire.last_error() << '\n';
                        }
                    }
                } else {
                    try {
                        response = std::move(reply);
                        if (mode_wire) {
                            const ModeRequestIdentity expected{
                                request.at("request_id").get<std::string>(), client_session,
                                request.at("onboard_session").get<std::string>(),
                                config.expected_uid, request.at("fc_epoch").get<std::uint64_t>()};
                            const ModeRequestIdentity received{
                                response.at("command_id").get<std::string>(),
                                response.at("requester_session_id").get<std::string>(),
                                response.at("session_id").get<std::string>(),
                                response.at("uid").get<std::string>(),
                                response.at("fc_epoch").get<std::uint64_t>()};
                            if (response.at("schema") != "argus.control.mode_result.v1" ||
                                !expected.matches(received) || !response.at("source").is_string() ||
                                response.at("source").get<std::string>().empty() ||
                                !response.at("seq").is_number_unsigned() ||
                                response.at("clock_domain") != "host_monotonic" ||
                                !valid_stamp(response.at("published_monotonic_ns"), ns(),
                                             config.command_ttl_ms) ||
                                response.at("published_monotonic_ns").get<std::int64_t>() <
                                    request.at("issued_monotonic_ns").get<std::int64_t>()) {
                                response = NativeJson::object();
                            } else {
                                response["schema"] = "argus.follow.rpc.v1";
                                response["status"] =
                                    response.at("result") == "ACCEPTED" ? "accepted" : "failed";
                            }
                        }
                    } catch (...) {
                        response = NativeJson::object();
                    }
                }
            } else {
                // batch 7 리뷰: 요청 변환 (to_proto 거부, 256 KiB 초과) 이나 송신이 실패해도 세고
                // 1, 2, 4 ... 번째에 남긴다. 결과는 예전처럼 응답 없음이다
                const std::uint64_t count = ++rpc_send_failures;
                if ((count & (count - 1)) == 0) {
                    std::cerr << "follow 6371 요청 보내기 실패 (누적 " << count
                              << "): " << rpc_wire.last_error() << '\n';
                }
            }
        }
        close_socket(socket);
        if (!response.is_object() || response.value("request_id", "") != request.at("request_id") ||
            response.value("client_session", "") != client_session ||
            response.value("schema", "") != "argus.follow.rpc.v1" ||
            !response.contains("fc_epoch") || response.at("fc_epoch") != request.at("fc_epoch") ||
            response.value("onboard_session", "") != request.at("onboard_session")) {
            return NativeJson::object();
        }
        return response;
    }
    void process(Job& job) {
        bool success = false;
        // 응답보다 먼저 온 이벤트가 결과를 정했으면 끝의 실패 처리가 덮어쓰지 않는다
        bool early_decided = false;
        if (Clock::now() < job.deadline) {
            bool permitted = true;
            if (job.action != "acquire" && job.action != "release" && job.action != "revoke") {
                permitted = writer_permit();
                std::lock_guard<std::mutex> lock(mutex);
                if (job.action == "hold" && !permitted) {
                    hold_diagnostic_locked("actor_precheck", "writer_permit_denied", job.request,
                                           false);
                }
                permitted = permitted && command_permit_locked(job.request);
                if (!permitted &&
                    (job.action == "hold" ||
                     (job.action == "velocity" && job.request.value("cv_gap_active", false)))) {
                    hold_diagnostic_locked("actor_precheck", "denied", job.request);
                }
            }
            if (permitted) {
                if (job.action == "velocity") {
                    // Serialize revocation with the final nonblocking wire exit.
                    std::lock_guard<std::mutex> lock(mutex);
                    if (!command_permit_locked(job.request)) {
                        if (job.request.value("cv_gap_active", false)) {
                            hold_diagnostic_locked("cv_wire_precheck",
                                                   "context_or_deadline_changed", job.request);
                        }
                        job.completion->set_value(false);
                        return;
                    }
                    auto request = job.request;
                    request["schema"] = "argus.control.setpoint.v1";
                    request["forward"] = request.at("vx_m_s");
                    request["right"] = request.at("vy_m_s");
                    request["down"] = request.at("vz_m_s");
                    request["yaw_rate"] = request.at("yaw_rate_rad_s");
                    // 2026-11-25 batch 6b: 팀 [control/setpoint][protobuf]. 예전처럼 상태 잠금
                    // 안에서 다시 검사한 뒤 보낸다 (취소와 마지막 송신을 직렬화한다). 변환 실패도
                    // 송신 실패다
                    success = control_bus->publish_setpoint(std::move(request));
                    if (!success && job.request.value("cv_gap_active", false)) {
                        hold_diagnostic_locked("cv_wire_send", "send_failed", job.request);
                    }
                } else {
                    const bool mode_action = job.action == "start_offboard" || job.action == "hold";
                    if (mode_action) {
                        const std::scoped_lock lock(mutex);
                        (job.action == "start_offboard" ? inflight_start : inflight_hold) =
                            job.request.at("request_id").get<std::string>();
                    }
                    auto response = rpc(job.request);
                    std::lock_guard<std::mutex> lock(mutex);
                    const bool same = job.request.at("revision") == revision && fresh_locked() &&
                                      job.request.at("fc_epoch") == epoch &&
                                      job.request.at("onboard_session") == onboard_session;
                    auto status = response.value("status", "failed");
                    success = same && Clock::now() < job.deadline && status == "accepted";
                    if (job.action == "hold") {
                        const auto result =
                            !same ? "context_or_freshness_changed"
                                  : (Clock::now() >= job.deadline
                                         ? "deadline_expired"
                                         : (response.empty()
                                                ? "no_valid_response"
                                                : (status == "accepted"
                                                       ? "accepted"
                                                       : (status == "pending" ? "pending"
                                                                              : "rejected"))));
                        const auto diagnostic_count = hold_diagnostics.size();
                        hold_diagnostic_locked("rpc_response", result, job.request);
                        if (hold_diagnostics.size() > diagnostic_count && response.is_object() &&
                            response.contains("reason") && response.at("reason").is_string()) {
                            try {
                                hold_diagnostics.back()["remote_reason"] =
                                    response.at("reason").get<std::string>();
                            } catch (...) {
                            }
                        }
                    }
                    if (job.action == "acquire" && success) {
                        auto capabilities =
                            response.value("capabilities", std::vector<std::string>{});
                        success = std::find(capabilities.begin(), capabilities.end(),
                                            "context_v1") != capabilities.end() &&
                                  std::find(capabilities.begin(), capabilities.end(),
                                            "absolute_heading_rad") != capabilities.end() &&
                                  !response.value("owner_token", "").empty() && !flight.rc_override;
                        if (success) {
                            token = response.at("owner_token");
                            authority = true;
                        }
                    }
                    if (job.action == "start_offboard" || job.action == "hold") {
                        auto& ack = job.action == "start_offboard" ? start_ack : hold_ack;
                        auto& pending =
                            job.action == "start_offboard" ? pending_start : pending_hold;
                        const bool await_event = config.mode_events && same &&
                                                 Clock::now() < job.deadline &&
                                                 (status == "accepted" || response.empty());
                        ack = same && (status == "pending" || await_event)
                                  ? std::optional<bool>{}
                                  : std::optional<bool>{success};
                        pending = same && (status == "pending" || await_event)
                                      ? job.request.at("request_id").get<std::string>()
                                      : "";
                        if (await_event) {
                            (job.action == "start_offboard" ? pending_start_context
                                                            : pending_hold_context) = job.request;
                            (job.action == "start_offboard" ? pending_start_until
                                                            : pending_hold_until) =
                                Clock::now() +
                                std::chrono::milliseconds(config.mode_result_timeout_ms);
                            const auto early = early_mode_events.find(pending);
                            if (early != early_mode_events.end()) {
                                const NativeJson stored = early->second;
                                try {
                                    (void)match_mode_event_locked(
                                        stored, stored.at("event").get<std::string>());
                                    // 응답이 비어(REQ 수신 제한 초과) success가 false여도
                                    // 이벤트가 정한 결과를 지킨다
                                    early_decided = pending.empty();
                                } catch (...) {
                                    // 형식이 틀린 FC 이벤트는 예전 수신 경로처럼 즉시 무효화한다
                                    if (authority) {
                                        hold_diagnostic_locked("token_invalidation",
                                                               "telemetry_exception");
                                    }
                                    invalidate_locked();
                                    state = {};
                                    attitude = {};
                                }
                            }
                        }
                    }
                    if (mode_action) {
                        // worker는 요청을 하나씩 보내므로 보관된 이벤트는 모두 이 요청의 것이다
                        (job.action == "start_offboard" ? inflight_start : inflight_hold).clear();
                        early_mode_events.clear();
                    }
                }
            }
        } else if (job.action == "hold" ||
                   (job.action == "velocity" && job.request.value("cv_gap_active", false))) {
            std::lock_guard<std::mutex> lock(mutex);
            hold_diagnostic_locked("actor_precheck", "deadline_expired", job.request);
        }
        if (!success && !early_decided &&
            (job.action == "start_offboard" || job.action == "hold")) {
            std::lock_guard<std::mutex> lock(mutex);
            if ((job.action == "start_offboard" ? pending_start : pending_hold).empty()) {
                (job.action == "start_offboard" ? start_ack : hold_ack) = false;
            }
        }
        job.completion->set_value(success);
    }
    void poll_pending(const std::string& action) {
        if (config.mode_events) {
            std::lock_guard<std::mutex> lock(mutex);
            auto& pending = action == "start_offboard" ? pending_start : pending_hold;
            const auto until =
                action == "start_offboard" ? pending_start_until : pending_hold_until;
            if (!pending.empty() && Clock::now() >= until) {
                if (action == "hold") {
                    hold_diagnostic_locked("mode_event", "timeout", pending_hold_context);
                }
                (action == "start_offboard" ? start_ack : hold_ack) = false;
                pending.clear();
            }
            return;
        }
        NativeJson request;
        std::string original;
        {
            std::lock_guard<std::mutex> lock(mutex);
            original = action == "start_offboard" ? pending_start : pending_hold;
            if (original.empty() || !authority || !fresh_locked()) {
                return;
            }
            request = envelope_locked("request_status");
            request["original_request_id"] = original;
        }
        auto response = rpc(request);
        auto status = response.value("status", "failed");
        std::lock_guard<std::mutex> lock(mutex);
        auto& pending = action == "start_offboard" ? pending_start : pending_hold;
        auto& ack = action == "start_offboard" ? start_ack : hold_ack;
        if (pending == original && request.at("revision") == revision) {
            if (status != "pending") {
                if (action == "hold") {
                    const auto diagnostic_count = hold_diagnostics.size();
                    hold_diagnostic_locked(
                        "legacy_status", status == "accepted" ? "accepted" : "rejected_or_invalid",
                        request);
                    if (hold_diagnostics.size() > diagnostic_count && response.is_object()) {
                        try {
                            if (response.contains("reason") && response.at("reason").is_string()) {
                                hold_diagnostics.back()["remote_reason"] = response.at("reason");
                            }
                            for (const auto* field : {"mav_result", "mav_result_param2"}) {
                                if (response.contains(field) &&
                                    response.at(field).is_number_integer()) {
                                    hold_diagnostics.back()[field] = response.at(field);
                                }
                            }
                        } catch (...) {
                        }
                    }
                }
                ack = status == "accepted";
                pending.clear();
            }
        }
    }
    // 대기 중인 FC 메시지 하나를 처리한다. 받을 메시지가 없으면 false.
    // 2026-11-25 batch 6a: 팀 6370 [이름][protobuf]. "fc/" 접두 SUB 하나로 받은 순서대로 처리하고
    // 이름은 정확히 비교한다 (team_wire receive_raw, 남은 프레임은 버린다). 다른 이름은 예전처럼
    // 건너뛴다. 해석하지 못한 본문은 예전 msgpack 해석 실패처럼 빈 값으로 telemetry 를 불러 같은
    // 경로로 간다. 2026-11-25 6a 리뷰: 구조 오류도 예전 처리와 같게 나눈다. 본문 프레임까지 받은
    // 뒤의 오류 (3 프레임 이상, 본문 상한 초과) 는 예전 코드가 빈 값으로 telemetry 를 부르던 경우라
    // 해석 실패와 같은 경로다 (fc/state, fc/attitude 는 바로 무효, fc/mode_event 는 버린다). 본문
    // 프레임이 없는 1 프레임과 소켓 오류는 예전 "bad FC multipart" 처럼 토픽과 관계없이 바로
    // 무효다. 3 프레임인데 본문이 0 바이트이면 1 프레임과 구별하지 못해 무효 쪽 (fail closed) 으로
    // 둔다
    bool receive_one() {
        argus::team_wire::RawMessage raw;
        const auto status = argus::team_wire::receive_raw(
            zmq::socket_ref(zmq::from_handle, subscriber), fc_topics(), raw);
        if (status == argus::team_wire::ReceiveStatus::NoMessage) {
            return false;
        }
        if (status == argus::team_wire::ReceiveStatus::NameMismatch) {
            return true;
        }
        try {
            const bool body_frame_error =
                status == argus::team_wire::ReceiveStatus::TooLarge ||
                (status == argus::team_wire::ReceiveStatus::BadFrameCount && !raw.body.empty());
            if (status != argus::team_wire::ReceiveStatus::Ok && !body_frame_error) {
                throw std::runtime_error(std::string("bad FC multipart: ") +
                                         argus::team_wire::to_string(status));
            }
            NativeJson data;
            std::uint64_t team_sequence = 0;
            std::string error = argus::team_wire::to_string(status);
            if (body_frame_error || !argus::team_wire::decode_body(raw.name, false, raw.body, data,
                                                                   team_sequence, error)) {
                data = NativeJson{};
                const std::uint64_t count = ++decode_failures;
                if ((count & (count - 1)) == 0) {
                    std::cerr << "follow FC 6370 해석 실패 (누적 " << count << "): " << error
                              << '\n';
                }
            }
            telemetry(raw.name, data);
        } catch (...) {
            const std::scoped_lock lock(mutex);
            if (authority) {
                hold_diagnostic_locked("token_invalidation", "telemetry_exception");
            }
            invalidate_locked();
            state = {};
            attitude = {};
        }
        return true;
    }
    // 경로 7~9 수신 전용. 요청이 응답을 기다려도 FC 자세와 상태는 계속 받는다
    void receive_loop(const std::shared_ptr<std::promise<bool>>& started) {
        subscriber = zmq_socket(context, ZMQ_SUB);
        bool ready = subscriber != nullptr;
        if (ready) {
            // 2026-11-25 batch 6a: linger 0, rcvhwm 128, "fc/" 접두 구독 하나 (토픽 사이 순서를
            // 지킨다). 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 는 두지 않는다. 본문 상한 (1 MiB) 은
            // receive_raw 가 보고 receive_one 이 해석 실패와 같은 경로로 센다 (연결은 그대로)
            try {
                argus::team_wire::configure_subscriber(
                    zmq::socket_ref(zmq::from_handle, subscriber), {"fc/"}, kTelemetryReceiveHwm);
                ready = zmq_connect(subscriber, config.telemetry_endpoint.c_str()) == 0;
            } catch (const zmq::error_t&) {
                ready = false;
            }
        }
        started->set_value(ready);
        // 한 번에 꺼내는 수의 상한. 수신 HWM(128)의 두 배라 쌓인 메시지는 모두 꺼내고, 끝없이
        // 들어와도 아래 권한 검사와 종료 확인은 계속 돈다
        constexpr int kMaxDrain = 256;
        while (ready && !quitting) {
            // 예전처럼 2 ms까지 기다린 뒤 쌓인 메시지를 모두 꺼낸다
            zmq_pollitem_t item{subscriber, 0, ZMQ_POLLIN, 0};
            zmq_poll(&item, 1, 2);
            int drained = 0;
            while (drained < kMaxDrain && receive_one()) {
                ++drained;
            }
            const std::scoped_lock lock(mutex);
            if (authority && (!fresh_locked() || flight.rc_override)) {
                hold_diagnostic_locked("token_invalidation", "telemetry_stale_or_rc_override");
                invalidate_locked();
            }
        }
        close_socket(subscriber);
    }
    void run(const std::shared_ptr<std::promise<bool>>& started) {
        // 2026-11-25 batch 6b: 팀 ZmqBus 발행자 둘 (linger 0, sndtimeo 0, SNDHWM 2, bind). 이
        // 스레드가 만들고 쓰고 없앤다
        bool ready = false;
        try {
            control_bus = std::make_unique<ControlBus>(config.control_endpoint);
            ready = control_bus->ready();
        } catch (const std::exception&) {
            ready = false;
        }
        if (!ready) {
            control_bus.reset();
        }
        started->set_value(ready);
        while (ready && !quitting) {
            // 작업이 들어오거나 2 ms가 지나면 깬다 (예전 poll 주기). 텔레메트리는 receive_loop가
            // 받는다
            Job job;
            bool have = false;
            {
                std::unique_lock<std::mutex> lock(mutex);
                jobs_ready.wait_for(lock, std::chrono::milliseconds(2),
                                    [this] { return !jobs.empty() || quitting.load(); });
                if (!jobs.empty()) {
                    job = std::move(jobs.front());
                    jobs.pop_front();
                    have = true;
                }
            }
            if (have) {
                try {
                    process(job);
                } catch (...) {
                    if (job.action == "hold") {
                        std::lock_guard<std::mutex> lock(mutex);
                        hold_diagnostic_locked("actor", "exception", job.request);
                    }
                    try {
                        job.completion->set_value(false);
                    } catch (...) {
                    }
                }
            }
            poll_pending("start_offboard");
            poll_pending("hold");
            if (control_bus) {
                NativeJson report;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (!latest_status.is_null() &&
                        (status_dirty || Clock::now() - last_status_at >= 1s)) {
                        report = {{"schema", "argus.follow.status.v1"},
                                  {"source", "argus-follow-controller"},
                                  {"clock_domain", "host_monotonic"},
                                  {"client_session", client_session},
                                  {"controller_session", client_session},
                                  {"session_id", client_session},
                                  {"uid", config.expected_uid},
                                  {"onboard_session", onboard_session},
                                  {"fc_epoch", epoch},
                                  {"seq", ++sequence},
                                  {"received_monotonic_ns", ns()},
                                  {"status", latest_status}};
                        status_dirty = false;
                        last_status_at = Clock::now();
                    }
                }
                if (!report.is_null()) {
                    // 2026-11-25 batch 6b: 팀 [follow/status][protobuf]. 예전처럼 결과를 보지
                    // 않는다. 변환 실패 (예: command_results 가 32 행 초과, 생산자 보관 한도는 32)
                    // 는 보내지 않는다. 게이트웨이는 3 s 뒤 follow_status_timeout 으로 멈춘다 (fail
                    // closed)
                    (void)control_bus->publish_status(std::move(report));
                }
            }
        }
        control_bus.reset();
    }
};

ZeroMqFollowBackend::ZeroMqFollowBackend(ZeroMqFollowBackendConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
ZeroMqFollowBackend::~ZeroMqFollowBackend() {
    bool owned;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        owned = !impl_->token.empty();
    }
    if (owned) {
        release();
    }
}
IVehicleStateSource& ZeroMqFollowBackend::pose() { return impl_->pose_port; }
IVehicleFlightStatusSource& ZeroMqFollowBackend::flight_state() { return impl_->state_port; }
IVehiclePort& ZeroMqFollowBackend::vehicle() { return impl_->vehicle_port; }
IFlightModePort& ZeroMqFollowBackend::mode() { return impl_->mode_port; }
bool ZeroMqFollowBackend::identity_permit() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->fresh_locked();
}
std::uint64_t ZeroMqFollowBackend::telemetry_decode_failures() const {
    return impl_->decode_failures.load();
}
std::string ZeroMqFollowBackend::verified_uid() const {
    return identity_permit() ? impl_->config.expected_uid : "";
}
bool ZeroMqFollowBackend::owns() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->authority && impl_->fresh_locked() && !impl_->flight.rc_override;
}
bool ZeroMqFollowBackend::acquire() {
    NativeJson request;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->fresh_locked() || impl_->flight.rc_override || impl_->command.bench) {
            return false;
        }
        // Reuse this controller's live ownership, including Scan terminal heading.
        // A revoked/stale owner must still perform an explicit fresh acquisition.
        if (impl_->authority && !impl_->token.empty()) {
            return true;
        }
        impl_->writer_requested = true;
        request = impl_->envelope_locked("acquire");
    }
    return impl_->enqueue(
        {"acquire", request, Clock::now() + std::chrono::milliseconds(impl_->config.rpc_timeout_ms),
         std::make_shared<std::promise<bool>>()},
        true);
}
void ZeroMqFollowBackend::release() {
    NativeJson request;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->token.empty()) {
            impl_->invalidate_locked();
            return;
        }
        request = impl_->envelope_locked("release");
        impl_->invalidate_locked();
    }
    impl_->enqueue(
        {"release", request, Clock::now() + std::chrono::milliseconds(impl_->config.rpc_timeout_ms),
         std::make_shared<std::promise<bool>>()},
        true);
}
void ZeroMqFollowBackend::set_writer_guard(std::function<bool()> permit) {
    bool clearing = !permit;
    {
        std::lock_guard<std::mutex> lock(impl_->guard_mutex);
        impl_->guard = std::move(permit);
    }
    if (clearing) {
        impl_->revoke(true);
    }
}
void ZeroMqFollowBackend::update_context(const FollowCommandContext& context) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->command.command_epoch != context.command_epoch ||
        impl_->command.generation != context.generation ||
        impl_->command.track_id != context.track_id) {
        // Keep a distinct wire context whenever the selected native ID changes.
        impl_->wire_command_epoch = std::max(impl_->wire_command_epoch + 1, context.command_epoch);
        impl_->cancel_locked();
    }
    impl_->command = context;
}
bool ZeroMqFollowBackend::send_heading(double yaw_rad, Clock::time_point until, bool stopping) {
    return std::isfinite(yaw_rad) &&
           impl_->submit("heading", {{"yaw_rad", yaw_rad}}, until, stopping);
}
NativeJson ZeroMqFollowBackend::observations() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    bool fresh = impl_->fresh_locked();
    auto state = impl_->state.is_object() ? impl_->state : NativeJson::object();
    return {{"schema", "argus.fc.observations.v1"},
            {"clock_domain", "host_monotonic"},
            {"epoch", impl_->epoch},
            {"onboard_session", impl_->onboard_session},
            {"state",
             {{"uid", fresh ? impl_->config.expected_uid : ""},
              {"identity_ok", fresh},
              {"fresh", fresh},
              {"local_ok", fresh && state.value("local_ok", false)},
              {"flight_environment", state.value("flight_environment", "")},
              {"flow_quality", state.value("flow_quality", NativeJson{})}}},
            {"sensors",
             {{"flow_valid", fresh && state.value("flow_valid", false)},
              {"gps_valid", fresh && state.value("gps_valid", false)}}},
            {"history", impl_->history}};
}
bool ZeroMqFollowBackend::validate_operator_request(const NativeJson& request) const {
    try {
        if (request.value("action", "") == "status") {
            return true;
        }
        std::lock_guard<std::mutex> lock(impl_->mutex);
        return impl_->fresh_locked() && request.at("expected_uid") == impl_->config.expected_uid &&
               request.at("controller_session") == impl_->client_session &&
               request.at("onboard_session") == impl_->onboard_session &&
               request.at("fc_epoch") == impl_->epoch;
    } catch (...) {
        return false;
    }
}
void ZeroMqFollowBackend::publish_status(const NativeJson& status) {
    if (impl_->config.read_only) {
        return;
    }
    auto coarse = status;
    std::function<void(NativeJson&)> strip_times = [&](NativeJson& value) {
        if (value.is_object()) {
            for (auto item = value.begin(); item != value.end();) {
                const auto& key = item.key();
                if (key == "seq" || key.find("timestamp") != std::string::npos ||
                    (key.size() >= 3 && (key.substr(key.size() - 3) == "_ns" ||
                                         key.substr(key.size() - 3) == "_at"))) {
                    item = value.erase(item);
                } else {
                    strip_times(item.value());
                    ++item;
                }
            }
        } else if (value.is_array()) {
            for (auto& item : value) {
                strip_times(item);
            }
        }
    };
    strip_times(coarse);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->status_dirty = impl_->status_dirty || coarse != impl_->last_status;
    impl_->last_status = std::move(coarse);
    impl_->latest_status = status;
}
std::vector<NativeJson> ZeroMqFollowBackend::take_hold_diagnostics() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<NativeJson> output;
    output.reserve(impl_->hold_diagnostics.size() + (impl_->hold_diagnostics_dropped ? 1 : 0));
    while (!impl_->hold_diagnostics.empty()) {
        output.push_back(std::move(impl_->hold_diagnostics.front()));
        impl_->hold_diagnostics.pop_front();
    }
    if (impl_->hold_diagnostics_dropped) {
        output.push_back({{"schema", "argus.follow.hold_diagnostic.loss.v1"},
                          {"monotonic_ns", ns()},
                          {"dropped_before", impl_->hold_diagnostics_dropped}});
        impl_->hold_diagnostics_dropped = 0;
    }
    return output;
}
NativeJson ZeroMqFollowBackend::operator_context() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return {{"onboard_session", impl_->onboard_session},
            {"session_id", impl_->onboard_session},
            {"fc_epoch", impl_->epoch},
            {"clock_domain", "host_monotonic"}};
}
std::string ZeroMqFollowBackend::controller_session() const { return impl_->client_session; }
}  // namespace follow
