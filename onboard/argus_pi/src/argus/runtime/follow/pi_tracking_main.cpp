#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <zmq.hpp>

#include "argus/bus/typed.hpp"
#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/perception/geometry/include/alignment_failure_window.hpp"
#include "argus/perception/geometry/include/pi_ground_projector.hpp"
#include "argus/perception/geometry/include/receipt_alignment_retry.hpp"
#include "argus/perception/geometry/include/static_camera_projector.hpp"
#include "argus/runtime/follow/include/decision_log.hpp"
#include "argus/runtime/follow/include/latest_pixel_receive.hpp"
#include "argus/runtime/follow/include/projection_hysteresis.hpp"
#include "argus/tracking/contracts/include/thermal_signature.hpp"
#include "argus/tracking/contracts/include/tracks_v1.hpp"
#include "argus/tracking/identity/include/persistent_generation.hpp"
#include "argus/tracking/ucmc/include/ucmc_tracker_adapter.hpp"
#include "argus/transport/include/observation_source.hpp"
#include "argus/transport/team_wire/include/pi_codec_table.hpp"
#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/zeromq/include/zeromq_operations.hpp"
#include "argus_msg/pi_tracking_status.pb.h"
#include "argus_msg/tracking_tracks.pb.h"
#ifdef ARGUS_WITH_DDS
#include "argus/transport/dds/dds_observation_source.hpp"
#endif
#ifdef ARGUS_FOLLOW_B_TRACKER
#include "argus/tracking/contracts/include/tracking_status_snapshot.hpp"
#include "argus/transport/follow_b/include/zeromq_follow_backend.hpp"
#endif

namespace {
template <typename Operation>
bool diagnostic_only(Operation&& operation) noexcept {
    try {
        operation();
        return true;
    } catch (...) {
        return false;
    }
}

volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }

// Only a bounded passive GET to the existing loopback FC observation service.
#ifndef ARGUS_FOLLOW_B_TRACKER
follow::NativeJson read_fc(unsigned port) {
    int socket = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket < 0) {
        throw std::runtime_error("FC observation socket failed");
    }
    struct Guard {
        int fd;
        ~Guard() { ::close(fd); }
    } guard{socket};
    timeval timeout{0, 200000};
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    if (::connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        throw std::runtime_error("FC observation HTTP unavailable");
    }
    const std::string request =
        "GET /observations HTTP/1.0\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n";
    if (::send(socket, request.data(), request.size(), MSG_NOSIGNAL) !=
        static_cast<ssize_t>(request.size())) {
        throw std::runtime_error("FC observation GET failed");
    }
    std::string response;
    char buffer[8192];
    for (;;) {
        auto count = ::recv(socket, buffer, sizeof(buffer), 0);
        if (count == 0) {
            break;
        }
        if (count < 0) {
            throw std::runtime_error("FC observation response timed out");
        }
        response.append(buffer, count);
        if (response.size() > 1024 * 1024) {
            throw std::runtime_error("FC observation response too large");
        }
    }
    auto boundary = response.find("\r\n\r\n");
    if (boundary == std::string::npos ||
        response.substr(0, response.find("\r\n")).find(" 200 ") == std::string::npos) {
        throw std::runtime_error("FC observation HTTP rejected");
    }
    return follow::NativeJson::parse(response.substr(boundary + 4));
}
#endif
using TracksValue = argus::team_wire::PbJson<argus::msg::TrackingTracks>;
using TrackingStatusValue = argus::team_wire::PbJson<argus::msg::PiTrackingStatus>;

// 2026-11-25 batch 2: tracking/tracks, tracking/status 를 팀 규약(ZeroMQ + protobuf, 6375)으로
// 보낸다. 메인 루프 스레드 하나가 팀 ZmqBus 와 발행자를 소유한다 (팀 ZmqBus 단일 스레드 규칙).
// 발행자만 있어 ZmqPoller 에 등록되는 소켓은 없고 process_events 를 부르지 않는다. 두 토픽은 같은
// PUB 소켓(output 엔드포인트 하나)으로 나간다. 큐 길이는 팀 값(kTrackingTracksQueueDepth,
// kTrackingStatusQueueDepth = 4, 예전 SNDHWM 2). v1 (카메라 전용) 과 v5 (FC 결합) 는 본문의
// pi_schema 로 구별한다.
class TeamTrackingBus {
public:
    explicit TeamTrackingBus(const std::string& endpoint)
        : bus_(context_, poller_, clock_,
               argus::bus_zmq::ZmqEndpointTable{
                   {std::string(argus::team_wire::kTrackingTracks), endpoint},
                   {std::string(argus::team_wire::kTrackingStatus), endpoint}},
               argus::team_wire::pi_codec_table(stats_)),
          tracks_(argus::bus::make_publisher<TracksValue>(
              bus_, argus::team_wire::kTrackingTracks,
              argus::team_wire::pb_json_type<argus::msg::TrackingTracks>(),
              argus::team_wire::kTrackingTracksQueueDepth)) {
        if (!tracks_) {
            throw std::runtime_error("team bus tracking/tracks publisher unavailable on " +
                                     endpoint);
        }
    }

    // 추적 상태는 B 추적기만 보낸다 (예전과 같다)
    void enable_status(const std::string& endpoint) {
        status_ = argus::bus::make_publisher<TrackingStatusValue>(
            bus_, argus::team_wire::kTrackingStatus,
            argus::team_wire::pb_json_type<argus::msg::PiTrackingStatus>(),
            argus::team_wire::kTrackingStatusQueueDepth);
        if (!status_) {
            throw std::runtime_error("team bus tracking/status publisher unavailable on " +
                                     endpoint);
        }
    }

    bool publish_tracks(const TracksValue& message) { return tracks_->publish(message); }

    bool publish_status(const TrackingStatusValue& message) {
        return status_ && status_->publish(message);
    }

private:
    zmq::context_t context_;
    argus::core::SteadyClock clock_;
    argus::bus_zmq::ZmqPoller poller_{clock_};
    std::shared_ptr<argus::team_wire::CodecStats> stats_ =
        std::make_shared<argus::team_wire::CodecStats>();
    argus::bus_zmq::ZmqBus bus_;
    std::unique_ptr<argus::bus::Publisher<TracksValue>> tracks_;
    std::unique_ptr<argus::bus::Publisher<TrackingStatusValue>> status_;
};
}  // namespace

int main(int argc, char** argv) {
    try {
        std::string calibration, measurement, profile, uid;
        std::string geometry_mode = "auto", camera_only_calibration;
        // 2026-11-25 batch 4: 관측은 팀 vision 6374 (argus-onboard 가 bind, protobuf). 예전 기본값
        // 5556 (vmy) 은 아무것도 bind 하지 않던 포트, 5558 (B) 은 argus_preview_bus 였다
        std::string input{argus::protocol::kDefaultVisionEndpoint};
        std::string output{argus::protocol::kDefaultTrackingEndpoint};
        unsigned http_port = 8768;
        std::string id_state_file = follow::PersistentGeneration::default_path();
        bool initialize_id_state = false;
        bool check_config = false;
        int receipt_retry_ms = 0;
        std::string frame_diagnostics;
        if (const auto path = std::getenv("ARGUS_V21_FRAME_DIAG_PATH")) {
            frame_diagnostics = path;
        }
#ifdef ARGUS_FOLLOW_B_TRACKER
        std::string fc_endpoint = "tcp://127.0.0.1:6370";

#endif
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            const auto next = [&] {
                if (++i >= argc) {
                    throw std::invalid_argument("missing " + key);
                }
                return std::string(argv[i]);
            };
            if (key == "--geometry") {
                geometry_mode = next();
            } else if (key == "--camera-only-calibration") {
                camera_only_calibration = next();
            } else if (key == "--id-state-file") {
                id_state_file = next();
            } else if (key == "--check-config") {
                check_config = true;
            } else if (key == "--frame-diagnostics") {
                frame_diagnostics = next();
            } else if (key == "--initialize-id-state") {
                initialize_id_state = true;
            } else if (key == "--calibration") {
                calibration = next();
            } else if (key == "--measurement") {
                measurement = next();
            } else if (key == "--ucmc-profile") {
                profile = next();
            } else if (key == "--expected-uid") {
                uid = next();
            } else if (key == "--input") {
                input = next();
            } else if (key == "--output") {
                output = next();
            } else if (key == "--fc-http-port") {
#ifdef ARGUS_FOLLOW_B_TRACKER
                throw std::invalid_argument("B tracker uses onboard telemetry, not HTTP FC access");
#else
                http_port = std::stoul(next());
#endif
#ifdef ARGUS_FOLLOW_B_TRACKER
            } else if (key == "--fc-state-endpoint") {
                fc_endpoint = next();
            } else if (key == "--receipt-retry-ms") {
                const auto value = next();
                std::size_t used = 0;
                receipt_retry_ms = std::stoi(value, &used);
                if (used != value.size() || receipt_retry_ms < 0 || receipt_retry_ms > 50) {
                    throw std::invalid_argument("receipt retry must be between 0 and 50ms");
                }
#endif
            } else {
                throw std::invalid_argument("unsupported Pi tracking option");
            }
        }
        if (check_config && initialize_id_state) {
            throw std::invalid_argument("config check cannot initialize identity state");
        }
        if (initialize_id_state) {
            follow::PersistentGeneration state{id_state_file, true};
            std::cout << "tracking identity state initialized; no FC/camera connection\n";
            return 0;
        }
        if (geometry_mode != "auto" && geometry_mode != "camera" && geometry_mode != "fc-ground") {
            throw std::invalid_argument("geometry must be auto, camera or fc-ground");
        }
        if (http_port < 1024 || http_port > 65535 ||
            (geometry_mode != "camera" && uid.size() != 36)) {
            throw std::invalid_argument("valid UID/port required");
        }
        const auto record = follow::read_native_json(calibration);
        if (geometry_mode != "camera" && record.value("expected_uid", "") != uid) {
            throw std::invalid_argument("calibration UID mismatch");
        }
        const auto geometry = record.at("geometry");
        std::unique_ptr<follow::PiGroundProjector> projector;
        if (geometry_mode != "camera") {
            projector = std::make_unique<follow::PiGroundProjector>(
                geometry, follow::read_native_json(measurement), uid);
            // 2026-11-12 임시 카메라 자세 보정 (camera_pitch_correction.sh on|off).
            // 없거나 1.0/0이면 기존 동작. 잘못된 값이면 추적기를 시작하지 않는다.
            const char* scale_env = std::getenv("ARGUS_CAMERA_PITCH_SCALE");
            const char* offset_env = std::getenv("ARGUS_CAMERA_ATTITUDE_OFFSET_MS");
            double pitch_scale = 1.;
            double offset_ms = 0.;
            try {
                if (scale_env != nullptr && *scale_env != '\0') {
                    pitch_scale = std::stod(scale_env);
                }
                if (offset_env != nullptr && *offset_env != '\0') {
                    offset_ms = std::stod(offset_env);
                }
            } catch (const std::exception&) {
                throw std::runtime_error(
                    "ARGUS_CAMERA_PITCH_SCALE / ARGUS_CAMERA_ATTITUDE_OFFSET_MS must be numbers");
            }
            if (!std::isfinite(offset_ms)) {
                throw std::runtime_error("ARGUS_CAMERA_ATTITUDE_OFFSET_MS must be finite");
            }
            projector->set_attitude_correction(pitch_scale, std::llround(offset_ms * 1e6));
            std::cerr << "UCMC camera attitude correction "
                      << (projector->attitude_correction_active() ? "ON" : "off")
                      << " pitch_scale=" << pitch_scale << " attitude_offset_ms=" << offset_ms
                      << '\n';
            // 2026-11-16 FC 자세 외삽 (config/tracking/attitude-extrapolation.env). 없거나 0이면
            // 꺼짐.
            const char* extrapolation_env = std::getenv("ARGUS_ATTITUDE_EXTRAPOLATION_MS");
            const char* extrapolation_rate_env =
                std::getenv("ARGUS_ATTITUDE_EXTRAPOLATION_MAX_RATE_DPS");
            double extrapolation_ms = 0.;
            double extrapolation_rate = 40.;
            try {
                if (extrapolation_env != nullptr && *extrapolation_env != '\0') {
                    extrapolation_ms = std::stod(extrapolation_env);
                }
                if (extrapolation_rate_env != nullptr && *extrapolation_rate_env != '\0') {
                    extrapolation_rate = std::stod(extrapolation_rate_env);
                }
            } catch (const std::exception&) {
                throw std::runtime_error(
                    "ARGUS_ATTITUDE_EXTRAPOLATION_MS / _MAX_RATE_DPS must be numbers");
            }
            projector->set_attitude_extrapolation(extrapolation_ms, extrapolation_rate);
            std::cerr << "UCMC attitude extrapolation "
                      << (projector->attitude_extrapolation_active() ? "ON" : "off")
                      << " max_ms=" << extrapolation_ms << " max_rate_deg_s=" << extrapolation_rate
                      << '\n';
        }
        std::unique_ptr<follow::StaticCameraProjector> camera_projector;
        if (geometry_mode != "fc-ground") {
            if (camera_only_calibration.empty()) {
                camera_only_calibration =
                    (std::filesystem::path(calibration).parent_path().parent_path() /
                     "ucmc-camera-estimate.txt")
                        .string();
            }
            camera_projector = std::make_unique<follow::StaticCameraProjector>(
                camera_only_calibration, geometry.at("image_width"), geometry.at("image_height"));
        }
        const auto values = follow::read_native_json(profile);
        follow::UcmcParameters parameters;
        parameters.a1 = values.at("a1");
        parameters.a2 = values.at("a2");
        parameters.wx = values.at("wx");
        parameters.wy = values.at("wy");
        parameters.vmax = values.at("vmax");
        parameters.max_age = values.at("max_age");
        parameters.fps = values.at("fps");
        parameters.high_score = values.at("high_score");
        if (check_config) {
            (void)follow::UcmcTrackerAdapter(parameters);
            std::cout
                << "tracker configuration valid; no sockets, FC, camera or generation write"
                << "; camera_offset=" << geometry.at("camera_offset_body_m").dump() << "; radius="
                << (record.contains("measurement_overrides")
                        ? record.at("measurement_overrides").at("ground_center_radius_m").dump()
                        : follow::read_native_json(measurement).at("ground_center_radius_m").dump())
                << "; receipt_retry_ms=" << receipt_retry_ms << '\n';
            return 0;
        }
        if (!follow::v21_enabled("ARGUS_V21_ENABLED")) {
            frame_diagnostics.clear();
        }
        follow::DecisionLog frame_log{frame_diagnostics};
        const auto flight_trace_flag = std::getenv("ARGUS_FLIGHT_RECORD");
        const auto flight_trace_path = std::getenv("ARGUS_FLIGHT_CMC_LOG");
        const bool flight_recording = flight_trace_flag && flight_trace_flag[0] == '1' &&
                                      flight_trace_flag[1] == '\0' && flight_trace_path;
        follow::DecisionLog flight_log{flight_recording ? flight_trace_path : ""};
        std::int64_t last_diagnostic_ns = 0;
        std::uint64_t suppressed_diagnostics = 0;
        // 최신 프레임만 처리하느라 건너뛴 메시지 수 (진단 기록 사이 누적, 유실과 구분)
        std::uint64_t skipped_for_latest = 0;
        std::string last_diagnostic_reason;
        follow::PersistentGeneration generations{id_state_file};
        auto generation = generations.allocate();
        std::uint64_t published_sequence = 0;
        std::map<std::pair<std::int32_t, std::string>, std::unique_ptr<follow::UcmcTrackerAdapter>>
            trackers;
        follow::ZeroMqContext context;
        // 탐지 결과 수신: ARGUS_OBSERVATION_TRANSPORT=zmq(기본) | dds
        std::unique_ptr<follow::ObservationSource> observations;
        const char* transport_env = std::getenv("ARGUS_OBSERVATION_TRANSPORT");
        const std::string observation_transport =
            transport_env && *transport_env ? transport_env : "zmq";
        if (observation_transport == "dds") {
#ifdef ARGUS_WITH_DDS
            observations = std::make_unique<follow::DdsObservationSource>();
#else
            throw std::runtime_error(
                "ARGUS_OBSERVATION_TRANSPORT=dds but this build has no DDS (ARGUS_WITH_DDS=OFF)");
#endif
        } else if (observation_transport == "zmq") {
            observations = std::make_unique<follow::ZmqObservationSource>(context, input);
        } else {
            throw std::runtime_error("ARGUS_OBSERVATION_TRANSPORT must be zmq or dds");
        }
        std::cerr << "observation transport=" << observation_transport << '\n';
        // 시작할 때 한 번 bind 한다. 실패하면 추적기를 시작하지 않는다 (예전 bind 실패와 같은 자리)
        std::unique_ptr<TeamTrackingBus> team_bus;
        try {
            team_bus = std::make_unique<TeamTrackingBus>(output);
#ifdef ARGUS_FOLLOW_B_TRACKER
            team_bus->enable_status(output);
#endif
        } catch (const std::exception& error) {
            throw std::runtime_error(std::string("native Pi bus ownership failed: ") +
                                     error.what());
        }
        std::cerr << "tracking output team bus " << output << " (tracking/tracks"
#ifdef ARGUS_FOLLOW_B_TRACKER
                  << ", tracking/status"
#endif
                  << ", protobuf)\n";
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        std::int64_t previous = 0;
        std::int64_t previous_motion = 0;
        std::string last_alignment_clock;
        std::uint64_t epoch = 0;
        std::optional<std::uint64_t> previous_frame_id;
        std::string source, last_error, previous_geometry, projection_error;
        bool last_camera_only = true, last_frame_valid = false;
        std::int64_t last_frame_sample = 0;
        // 2026-11-10 auto 투영 전환 히스테리시스(짧은 FC 정렬 실패로 ID·선택이 초기화되지 않게).
        follow::ProjectionHysteresis projection_hysteresis;
        // 2026-11-10 시계 정렬 실패가 3초 넘게 이어지면 정렬 추정을 다시 잡는다(재시작과 같은
        // 효과).
        std::int64_t alignment_failure_since_ns = 0;
        constexpr std::int64_t kAlignmentReanchorNs = 3000000000LL;
        // 2026-11-12 실패가 성공과 섞여 이어질 때도 다시 잡는다 (alignment_failure_window.hpp).
        follow::AlignmentFailureWindow alignment_window;
#ifdef ARGUS_FOLLOW_B_TRACKER
        follow::ZeroMqFollowBackendConfig fc_config;
        fc_config.telemetry_endpoint = fc_endpoint;
        fc_config.expected_uid = uid;
        fc_config.read_only = true;
        std::unique_ptr<follow::ZeroMqFollowBackend> fc_reader;
        if (geometry_mode != "camera") {
            fc_reader = std::make_unique<follow::ZeroMqFollowBackend>(fc_config);
        }
        follow::TrackingStatusSnapshot tracking_status{follow::native_now_ns()};
#endif
        while (!stopped) {
#ifdef ARGUS_FOLLOW_B_TRACKER
            const auto status_now = follow::native_now_ns();
            if (tracking_status.due(status_now)) {
                const auto fc_status = fc_reader ? fc_reader->observations() : follow::NativeJson{};
                auto status = tracking_status.snapshot(
                    status_now, generation,
                    fc_reader && fc_status.at("state").value("fresh", false),
                    !last_camera_only && fc_reader &&
                        fc_status.value("epoch", std::uint64_t{0}) == epoch);
                status["projection_mode"] =
                    previous_geometry.empty() ? "waiting" : previous_geometry;
                status["tracking_valid"] = last_frame_valid && last_frame_sample > 0 &&
                                           status_now >= last_frame_sample &&
                                           status_now - last_frame_sample < 250000000;
                status["fc_projection_error"] = projection_error;
                if (!team_bus->publish_status(TrackingStatusValue{std::move(status), {}})) {
                    std::cerr << "native Pi tracking status publish failed\n";
                }
            }
#endif
            if (!observations->wait(std::chrono::milliseconds{25})) {
                continue;
            }
            follow::NativeJson diagnostic_pixels = follow::NativeJson::object();
            follow::NativeJson diagnostic_fc;
            const auto diagnostic_begin = follow::native_now_ns();
            int diagnostic_fc_reads = 0;
            std::int64_t diagnostic_extrapolated_ns = 0;  // 2026-11-16 자세 외삽 시간 (진단)
            follow::NativeJson flight_trace;
            if (flight_recording) {
                if (!diagnostic_only([&] {
                        flight_trace = {{"schema", "argus.flight.cmc.v1"},
                                        {"begin_monotonic_ns", diagnostic_begin},
                                        {"associations", follow::NativeJson::array()},
                                        {"kalman_after_update", follow::NativeJson::array()}};
                    })) {
                    flight_log.note_dropped();
                }
            }
            const auto record_flight = [&](const char* outcome,
                                           const std::string& reason) noexcept {
                if (!flight_recording) {
                    return;
                }
                try {
                    auto evidence = flight_trace;
                    evidence["outcome"] = outcome;
                    evidence["reason"] = reason;
                    evidence["done_monotonic_ns"] = follow::native_now_ns();
                    evidence["pixels"] = diagnostic_pixels;
                    // Failure retains the actual FC histories used by the existing retry path.
                    if (std::string(outcome) != "published") {
                        evidence["fc_input"] = diagnostic_fc;
                    }
                    flight_log.try_record([record = std::move(evidence)] { return record; });
                } catch (...) {
                    flight_log.note_dropped();
                }
            };
            const auto diagnose = [&](const char* event, const std::string& reason,
                                      double alignment, int track_count = -1,
                                      int real_count = -1) noexcept {
                try {
                    if (frame_diagnostics.empty()) {
                        return;
                    }
                    const auto diagnostic_now = follow::native_now_ns();
                    if (diagnostic_now - last_diagnostic_ns < 250000000) {
                        ++suppressed_diagnostics;
                        return;
                    }
                    last_diagnostic_ns = diagnostic_now;
                    follow::NativeJson record{
                        {"schema", "argus.tracking.frame_diagnostic.v1"},
                        {"suppressed_since_last", suppressed_diagnostics},
                        {"skipped_for_latest", skipped_for_latest},
                        {"event", event},
                        {"begin_ns", diagnostic_begin},
                        {"done_ns", follow::native_now_ns()},
                        {"fc_reads", diagnostic_fc_reads},
                        {"reason", reason},
                        {"alignment_ms", alignment},
                        {"track_count", track_count},
                        {"real_count", real_count},
                        {"attitude_extrapolated_ms",
                         static_cast<double>(diagnostic_extrapolated_ns) / 1e6},
                        {"frame_id", diagnostic_pixels.is_object()
                                         ? diagnostic_pixels.value("frame_id", follow::NativeJson{})
                                         : follow::NativeJson{}},
                        {"source_ns", diagnostic_pixels.is_object()
                                          ? diagnostic_pixels.value("captured_monotonic_ns",
                                                                    follow::NativeJson{})
                                          : follow::NativeJson{}}};
                    // 2026-11-12: 정렬 추정 상태(카메라·FC offset, 최소 offset)와 최근 1초 실패 수.
                    if (projector) {
                        record["clock_alignment"] = projector->clock_alignment_state();
                        record["clock_alignment"]["window_failures"] = alignment_window.failures();
                        record["clock_alignment"]["window_attempts"] = alignment_window.attempts();
                    }
                    if (std::string(event) == "rejected") {
                        record["pixels"] = diagnostic_pixels;
                        if (diagnostic_fc.is_object()) {
                            record["fc_history"] =
                                diagnostic_fc.value("history", follow::NativeJson::object());
                            record["fc_epoch"] = diagnostic_fc.value("epoch", follow::NativeJson{});
                            record["fc_state"] = follow::NativeJson::object();
                            const auto state =
                                diagnostic_fc.value("state", follow::NativeJson::object());
                            if (state.is_object()) {
                                for (const auto* key :
                                     {"identity_ok", "fresh", "local_ok", "flow_quality"}) {
                                    record["fc_state"][key] =
                                        state.value(key, follow::NativeJson{});
                                }
                            }
                            const auto sensors =
                                diagnostic_fc.value("sensors", follow::NativeJson{});
                            record["flow_valid"] =
                                sensors.is_object()
                                    ? sensors.value("flow_valid", follow::NativeJson{})
                                    : follow::NativeJson{};
                        }
                    }
                    if (record.contains("pixels") && record["pixels"].is_object()) {
                        auto& list = record["pixels"]["detections"];
                        if (list.is_array() && list.size() > 64) {
                            record["diagnostic_detection_truncated"] = list.size() - 64;
                            list.erase(list.begin() + 64, list.end());
                        }
                    }
                    if (record.contains("fc_history") && record["fc_history"].is_object()) {
                        for (auto& item : record["fc_history"].items()) {
                            auto& history = item.value();
                            if (history.is_array() && history.size() > 16) {
                                history.erase(history.begin(), history.end() - 16);
                                record["diagnostic_history_truncated"] = true;
                            }
                        }
                    }
                    const bool critical =
                        std::string(event) != "published" && reason != last_diagnostic_reason;
                    if (frame_log.try_record([record = std::move(record)] { return record; },
                                             critical)) {
                        suppressed_diagnostics = 0;
                        skipped_for_latest = 0;
                        last_diagnostic_reason = reason;
                    }
                } catch (...) {
                    std::cerr << "ARGUS_FRAME_DIAG_WRITE_FAILED\n";
                }
            };
            try {
                // 2026-11-09: 밀린 프레임은 건너뛰고 가장 최신 관측 하나만 처리한다.
                auto pixels = observations->take(skipped_for_latest);
                diagnostic_pixels = pixels;
                follow::PiProjectedFrame frame{};
                bool camera_only = geometry_mode == "camera";
                projection_error.clear();
                if (!camera_only) {
                    try {
#ifdef ARGUS_FOLLOW_B_TRACKER
                        frame = follow::project_with_receipt_retry(
                            *projector, pixels,
                            [&] {
                                ++diagnostic_fc_reads;
                                diagnostic_fc = fc_reader->observations();
                                return diagnostic_fc;
                            },
                            [] { return follow::native_now_ns(); },
                            [] { std::this_thread::sleep_for(std::chrono::milliseconds{2}); },
                            std::chrono::milliseconds{receipt_retry_ms});
                        diagnostic_extrapolated_ns = frame.attitude_extrapolated_ns;
#else
                        const auto fc = read_fc(http_port);
                        frame = projector->project(pixels, fc, follow::native_now_ns());
#endif
                    } catch (const std::exception& error) {
                        {
                            const std::string failure = error.what();
                            const auto failure_ns = follow::native_now_ns();
                            const bool alignment_failure =
                                follow::is_waitable_alignment_failure(failure) ||
                                failure == "mapped SDK sample expired or future";
                            const bool mixed_reanchor =
                                alignment_window.record(failure_ns, alignment_failure);
                            if (alignment_failure) {
                                if (alignment_failure_since_ns == 0) {
                                    alignment_failure_since_ns = failure_ns;
                                }
                                if (failure_ns - alignment_failure_since_ns >
                                        kAlignmentReanchorNs ||
                                    mixed_reanchor) {
                                    const auto before = projector->clock_alignment_state();
                                    const int failures = alignment_window.failures();
                                    const int attempts = alignment_window.attempts();
                                    projector->reset_clock_alignment();
                                    alignment_window.reanchored(failure_ns);
                                    std::cerr << "UCMC clock alignment re-anchored at "
                                              << failure_ns << "ns after "
                                              << (mixed_reanchor
                                                      ? std::to_string(failures) + "/" +
                                                            std::to_string(attempts) +
                                                            " alignment failures in 1s"
                                                      : std::string("3s of alignment failures"))
                                              << ": " << failure << " state=" << before.dump()
                                              << '\n';
                                    alignment_failure_since_ns = 0;
                                }
                            } else {
                                alignment_failure_since_ns = 0;
                            }
                        }
                        if (geometry_mode == "fc-ground") {
                            throw;
                        }
                        // Preserve the existing FC observation grace window. A single
                        // alignment miss must not churn identities before the old
                        // sample expires; it still cannot refresh that sample.
                        const auto failure_now = follow::native_now_ns();
                        // 2026-11-10: 유예 250ms → 1.5초 (projection_hysteresis.hpp).
                        if (projection_hysteresis.hold_fc_ground_on_failure(
                                previous_geometry, last_frame_sample, failure_now)) {
                            throw;
                        }
                        projection_error = error.what();
                        camera_only = true;
                    }
                }
                if (!camera_only) {
                    alignment_failure_since_ns = 0;  // FC 투영 성공
                    alignment_window.record(follow::native_now_ns(), false);
                }
                // 2026-11-10: 카메라 전용에서는 FC 투영이 1초 계속 성공해야 fc-ground로 돌아간다.
                if (!camera_only && geometry_mode == "auto" &&
                    !projection_hysteresis.accept_fc_ground(previous_geometry,
                                                            follow::native_now_ns())) {
                    camera_only = true;
                }
                if (camera_only) {
                    frame = camera_projector->project(pixels, follow::native_now_ns());
                }
                const std::string current_geometry = camera_only ? "camera" : "fc-ground";
                auto current_source = pixels.at("source").get<std::string>();
                const auto& pixel_id = pixels.at("frame_id");
                if ((!pixel_id.is_number_unsigned() && !pixel_id.is_number_integer()) ||
                    (pixel_id.is_number_integer() && !pixel_id.is_number_unsigned() &&
                     pixel_id.get<std::int64_t>() < 0)) {
                    throw std::runtime_error("original pixel frame ID invalid");
                }
                const auto frame_id = pixels.at("frame_id").get<std::uint64_t>();
                if (current_source.empty() || current_source.size() > 128) {
                    throw std::runtime_error("original pixel source invalid");
                }
                // 같은 카메라 source라도 생산자 재시작으로 프레임 번호가 되돌아갈 수 있다.
                if ((!source.empty() && (source != current_source || epoch != frame.pose.epoch ||
                                         previous_geometry != current_geometry)) ||
                    (previous_frame_id && frame_id < *previous_frame_id)) {
                    trackers.clear();
                    generation = generations.allocate();
                    previous = 0;
                    previous_motion = 0;
                    previous_frame_id.reset();
                }
                if (previous_geometry != current_geometry) {
                    std::cerr << "UCMC projection=" << current_geometry
                              << (camera_only ? "; tracking only; flight disabled"
                                              : "; FC ground input")
                              << '\n';
                }
                source = current_source;
                epoch = frame.pose.epoch;
                previous_geometry = current_geometry;
                last_camera_only = camera_only;
                if (frame.sample_ns <= previous ||
                    (previous_frame_id && frame_id == *previous_frame_id)) {
                    throw std::runtime_error("original pixel sample replayed");
                }
                previous = frame.sample_ns;
                previous_frame_id = frame_id;
                const auto motion_stamp =
                    frame.motion_sample_ns > 0 ? frame.motion_sample_ns : frame.sample_ns;
                if (motion_stamp <= previous_motion) {
                    throw std::runtime_error("SDK motion sample did not advance");
                }
                previous_motion = motion_stamp;
                if (frame.alignment_clock != last_alignment_clock) {
                    std::cerr << "UCMC clock=" << frame.alignment_clock
                              << " exposure_verified=false\n";
                    last_alignment_clock = frame.alignment_clock;
                }
                using Key = std::pair<std::int32_t, std::string>;
                std::map<Key, std::vector<follow::MetricBoxObservation>> groups;
                std::map<Key, std::vector<follow::NativeJson>> group_thermal;
                for (const auto& detection : frame.detections) {
                    groups[{detection.class_id, detection.label}].push_back(detection.metric);
                    group_thermal[{detection.class_id, detection.label}].push_back(
                        detection.thermal);
                }
                for (const auto& pair : trackers) {
                    groups.try_emplace(pair.first);
                }
                follow::NativeJson tracks = follow::NativeJson::array();
                for (const auto& group : groups) {
                    auto& tracker = trackers[group.first];
                    if (!tracker) {
                        tracker = std::make_unique<follow::UcmcTrackerAdapter>(parameters);
                    }
                    for (const auto& association :
                         tracker->update(group.second, motion_stamp / 1e9)) {
                        if (flight_recording) {
                            if (!diagnostic_only([&] {
                                    flight_trace["associations"].push_back(
                                        {{"class_id", group.first.first},
                                         {"label", group.first.second},
                                         {"detection_index_in_class", association.detection},
                                         {"track_id", association.id}});
                                })) {
                                flight_log.note_dropped();
                            }
                        }
                        const auto& m = group.second.at(association.detection);
                        const auto& box = m.box;
                        follow::NativeJson track{
                            {"track_id", association.id},
                            {"class_id", group.first.first},
                            {"label", group.first.second},
                            {"confidence", box[4]},
                            {"x1", box[0]},
                            {"y1", box[1]},
                            {"x2", box[2]},
                            {"y2", box[3]},
                            {"state", "CONFIRMED"},
                            {"real_detection", true},
                            {"last_real_detection_monotonic_ns", frame.sample_ns}};
                        if (!camera_only) {
                            track["ground"] = {m.point[0],         m.point[1],
                                               m.covariance(0, 0), m.covariance(0, 1),
                                               m.covariance(1, 1), frame.pose.alignment_error_ms};
                            track["ground_anchor"] = m.anchor;
                            // 2026-11-09 열 외형은 v5 트랙에만 싣는다 (형식이 맞을 때만).
                            const auto& thermal =
                                group_thermal.at(group.first).at(association.detection);
                            if (follow::thermal_wire_valid(thermal)) {
                                track.emplace("thermal", thermal);
                            }
                        }
                        tracks.push_back(std::move(track));
                    }
                    if (flight_recording) {
                        if (!diagnostic_only([&] {
                                for (const auto& tr : tracker->diagnostic_tracks()) {
                                    follow::NativeJson state{
                                        {"class_id", group.first.first},
                                        {"label", group.first.second},
                                        {"track_id", tr.id},
                                        {"status", static_cast<int>(tr.status)},
                                        {"age", tr.age},
                                        {"death_count", tr.death_count},
                                        {"birth_count", tr.birth_count},
                                        {"detection_index_in_class", tr.detidx},
                                        {"x", {tr.x[0], tr.x[1], tr.x[2], tr.x[3]}}};
                                    state["P"] = follow::NativeJson::array();
                                    for (int row = 0; row < 4; ++row) {
                                        for (int col = 0; col < 4; ++col) {
                                            state["P"].push_back(tr.P(row, col));
                                        }
                                    }
                                    flight_trace["kalman_after_update"].push_back(std::move(state));
                                }
                            })) {
                            flight_log.note_dropped();
                        }
                    }
                }
                if (flight_recording) {
                    if (!diagnostic_only([&] {
                            flight_trace["cmc"] = frame.diagnostic;
                            flight_trace["cmc_diagnostic_lost"] = frame.diagnostic_lost;
                            if (frame.diagnostic_lost) {
                                flight_log.note_dropped();
                            }
                            flight_trace["generation"] = generation;
                            flight_trace["projection_mode"] = current_geometry;
                            flight_trace["measurements"] = follow::NativeJson::array();
                            for (const auto& d : frame.detections) {
                                flight_trace["measurements"].push_back(
                                    {{"class_id", d.class_id},
                                     {"label", d.label},
                                     {"box", d.metric.box},
                                     {"ground_ne", {d.metric.point[0], d.metric.point[1]}},
                                     {"ground_covariance",
                                      {d.metric.covariance(0, 0), d.metric.covariance(0, 1),
                                       d.metric.covariance(1, 1)}},
                                     {"anchor", d.metric.anchor}});
                            }
                        })) {
                        flight_log.note_dropped();
                    }
                }
                follow::NativeJson message{
                    {"schema", camera_only ? follow::kTracksV1Schema : follow::kTracksV5Schema},
                    {"source", current_source},
                    {"frame_id", frame_id},
                    {"sample_monotonic_ns", frame.sample_ns},
                    {"clock_domain", "host_monotonic"},
                    {"timestamp_source",
                     camera_only ? follow::kCameraOnlyTimestampSource
                     : frame.alignment_clock == "sdk_fc_boot_receipt_estimate_unverified"
                         ? "pi_sdk_fc_boot_estimate_unverified"
                         : "pi_original_sample_fc_host_receive_estimate_unverified"},
                    {"exposure_monotonic_ns", nullptr},
                    {"exposure_time_valid", false},
                    {"image_width", pixels.at("image_width")},
                    {"image_height", pixels.at("image_height")},
                    {"coordinate_frame", "camera_pixel"},
                    {"generation", generation},
                    {"tracks", tracks}};
                ++published_sequence;
                if (!camera_only) {
                    constexpr double degrees = 180 / 3.14159265358979323846;
                    message["seq"] = published_sequence;
                    message["fc_epoch"] = epoch;
                    message["motion_sample_ns"] = motion_stamp;
                    message["ground_pose"] = {
                        frame.pose.position[0],      frame.pose.position[1],
                        frame.pose.position[2],      frame.pose.rpy[0] * degrees,
                        frame.pose.rpy[1] * degrees, frame.pose.rpy[2] * degrees};
                    // 2026-11-19 PAMPC #5 v5 선택 확장: 지면 투영에 실제로 쓴 평면 down(m, NED)과
                    // 투영 자세(deg, 자세 시각 보정 뒤). ground_pose와 다른 필드의 의미는 그대로다.
                    if (std::isfinite(frame.ground_plane_down_m) &&
                        frame.projection_rpy.allFinite()) {
                        message["ground_plane_down_m"] = frame.ground_plane_down_m;
                        message["projection_attitude_deg"] = {frame.projection_rpy[0] * degrees,
                                                              frame.projection_rpy[1] * degrees,
                                                              frame.projection_rpy[2] * degrees};
                    }
                }
                if (follow::native_now_ns() - frame.sample_ns >= 250000000) {
                    throw std::runtime_error("original sample expired during processing");
                }
                const TracksValue outgoing{std::move(message), {}};
                if (!team_bus->publish_tracks(outgoing)) {
                    throw std::runtime_error("native Pi tracks publish failed");
                }
                if (!last_error.empty()) {
                    std::cout << "native Pi tracking recovered; projection=" << current_geometry
                              << '\n';
                }
#ifdef ARGUS_FOLLOW_B_TRACKER
                tracking_status.record_success(frame.sample_ns);
                diagnose(
                    "published", "", frame.pose.alignment_error_ms, static_cast<int>(tracks.size()),
                    static_cast<int>(std::count_if(tracks.begin(), tracks.end(), [](const auto& t) {
                        return t.value("real_detection", false);
                    })));
#endif
                if (flight_recording) {
                    if (!diagnostic_only([&] {
                            flight_trace["published_tracks"] = outgoing.value;
                            record_flight("published", "");
                        })) {
                        flight_log.note_dropped();
                    }
                }
                last_error.clear();
                last_frame_valid = true;
                last_frame_sample = frame.sample_ns;
            } catch (const follow::TrackingIdentityError& error) {
#ifdef ARGUS_FOLLOW_B_TRACKER
                diagnose("identity_fatal", error.what(), -1.);
#endif
                record_flight("identity_fatal", error.what());
                // 재사용 가능한 ID가 없으면 출력/진단을 계속 갱신하지 않고 종료한다.
                throw;
            } catch (const std::exception& error) {
                last_frame_valid = false;
#ifdef ARGUS_FOLLOW_B_TRACKER
                tracking_status.record_failure();
                diagnose("rejected", error.what(), -1.);
#endif
                record_flight("rejected", error.what());
                if (last_error != error.what()) {
                    std::cerr << "native Pi frame rejected: " << error.what() << '\n';
                }
                last_error = error.what();
            }
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "native Pi tracker: " << error.what() << '\n';
        return 1;
    }
}
