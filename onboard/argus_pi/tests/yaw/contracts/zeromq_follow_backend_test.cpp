#include "argus/transport/follow_b/include/zeromq_follow_backend.hpp"

#include <zmq.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

#include "argus/transport/team_wire/include/team_wire.hpp"

using namespace follow;
namespace {
const std::string uid = "000600000000353733323333511700280027";
// 2026-11-25 batch 6a: 6370 은 팀 [이름][protobuf]. 시험의 짧은 FC 값을 Pi 게이트웨이 모양
// (follow_owner.cpp publish, attitude_wire, mode_event 의 키 전부) 으로 채운다. 받는 쪽 to_json 은
// 늘 모든 키를 채우므로 시험 입력도 실제 생산자 모양이어야 한다 (tests/protobuf/team_fc_wire.py 와
// 같은 규칙). distance_cm 이 없으면 distance_m 에서, 실수이면 정수로 바꾼다 (게이트웨이는 늘 정수
// cm 와 m 을 함께 보낸다)
void fill_default(NativeJson& value, const char* key, NativeJson fallback) {
    if (!value.contains(key)) {
        value[key] = std::move(fallback);
    }
}
NativeJson fc_state_shape(NativeJson value) {
    fill_default(value, "source", "argus-onboard");
    fill_default(value, "session_id", value.value("onboard_session", std::string{}));
    fill_default(value, "fc_link_connected", value.value("connected", false));
    for (const auto* key :
         {"connected", "armed", "in_air", "offboard", "hold", "position", "rc_override",
          "offboard_hover", "ground_known", "landed", "local_ok", "flow_valid", "gps_valid"}) {
        fill_default(value, key, false);
    }
    fill_default(value, "flight_environment", "unknown");
    fill_default(value, "flow_quality", 0);
    fill_default(value, "fault_reason", "");
    fill_default(value, "position_time_boot_ms", nullptr);
    fill_default(value, "distance_time_boot_ms", nullptr);
    for (const auto* key :
         {"position_monotonic_ns", "distance_monotonic_ns", "distance_orientation"}) {
        fill_default(value, key, 0);
    }
    for (const auto& [si, cm] : {std::pair<const char*, const char*>{"distance_m", "distance_cm"},
                                 {"min_distance_m", "min_distance_cm"},
                                 {"max_distance_m", "max_distance_cm"}}) {
        if (!value.contains(cm)) {
            value[cm] = value.contains(si) ? std::llround(value.at(si).get<double>() * 100.0) : 0;
        } else if (value.at(cm).is_number_float()) {
            value[cm] = std::llround(value.at(cm).get<double>());
        }
        fill_default(value, si, value.at(cm).get<double>() / 100.0);
    }
    if (value.contains("cmc_pose_history")) {
        auto& history = value["cmc_pose_history"];
        for (auto it = history.begin(); it != history.end(); ++it) {
            const std::string& kind = it.key();
            for (auto& row : it.value()) {
                auto& data = row["data"];
                data["mavpackettype"] = kind;
                if (kind == "ATTITUDE") {
                    for (const auto* key : {"rollspeed", "pitchspeed", "yawspeed"}) {
                        fill_default(data, key, 0.0);
                    }
                } else if (kind == "DISTANCE_SENSOR") {
                    for (const auto* key : {"type", "id", "covariance", "signal_quality"}) {
                        fill_default(data, key, 0);
                    }
                    fill_default(data, "horizontal_fov", 0.0);
                    fill_default(data, "vertical_fov", 0.0);
                    fill_default(data, "quaternion", NativeJson::array({0.0, 0.0, 0.0, 0.0}));
                }
            }
        }
    }
    return value;
}
NativeJson fc_attitude_shape(NativeJson value) {
    fill_default(value, "source", "argus-onboard");
    fill_default(value, "session_id", value.value("onboard_session", std::string{}));
    for (const auto* key :
         {"fc_time_boot_ms", "fc_time_monotonic_ns", "roll_rad_s", "pitch_rad_s", "yaw_rad_s"}) {
        fill_default(value, key, nullptr);
    }
    fill_default(value, "timesync_valid", false);
    return value;
}
NativeJson fc_mode_event_shape(NativeJson value) {
    fill_default(value, "source", "argus-onboard");
    fill_default(value, "session_id", value.value("onboard_session", std::string{}));
    // protobuf 에서는 owner_token 이 늘 있다 (받는 쪽은 있으면 지금 토큰과 비교한다). 게이트웨이는
    // 지금 토큰을 싣는다
    fill_default(value, "owner_token", "fixture-owner-token");
    for (const auto* key : {"requester_session_id", "command_id", "reason"}) {
        fill_default(value, key, "");
    }
    fill_default(value, "revision", 0);
    fill_default(value, "command_epoch", 0);
    return value;
}
// 팀 seq 는 TeamWire 의 토픽별 발행 번호. Pi seq (pi_seq) 가 없으면 이 값을 seq 로 쓴다
void send_fc(void* socket, argus::team_wire::TeamWire& wire, const char* topic, NativeJson value) {
    const std::string name(topic);
    if (name == "fc/state") {
        value = fc_state_shape(std::move(value));
    } else if (name == "fc/attitude") {
        value = fc_attitude_shape(std::move(value));
    } else {
        value = fc_mode_event_shape(std::move(value));
    }
    if (!value.contains("seq")) {
        value["seq"] = wire.last_sequence(name) + 1;
    }
    if (!wire.publish(zmq::socket_ref(zmq::from_handle, socket), name, value)) {
        std::cerr << "fixture " << name << " encode: " << wire.last_error() << '\n';
        std::abort();
    }
}
std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
        .count();
}
// 2026-11-25 batch 6b: 6372 는 팀 [이름][protobuf] (추종 제어 ZmqBus). 받는 이름
const std::vector<std::string_view>& control_topics() {
    static const std::vector<std::string_view> kTopics{"control/setpoint", "follow/status"};
    return kTopics;
}
// 실기 기록 fixture follow_status_0 의 status (추종 제어 projected_status 모양). protobuf 는 키가
// 모두 있어야 보낸다 (예전 시험의 짧은 {"mode": "idle"} 은 to_proto 가 거부한다)
NativeJson recorded_status() {
    std::ifstream file(std::string(ARGUS_PB_FIXTURES) + "/follow_status_0.msgpack",
                       std::ios::binary);
    const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)),
                                          std::istreambuf_iterator<char>());
    if (bytes.empty()) {
        throw std::runtime_error("follow_status_0.msgpack fixture missing");
    }
    return NativeJson::from_msgpack(bytes).at("status");
}
// command_results 를 rows 행으로 (보관 한도 32. 넘으면 생산자 to_proto 가 거부한다)
NativeJson status_with_rows(std::size_t rows) {
    auto status = recorded_status();
    const auto row = status.at("command_results").at(0);
    status["command_results"] = NativeJson::array();
    for (std::size_t i = 0; i < rows; ++i) {
        auto copy = row;
        copy["command_id"] = "row-" + std::to_string(i);
        status["command_results"].push_back(copy);
    }
    return status;
}
void require(bool result, const char* text) {
    if (!result) {
        throw std::runtime_error(text);
    }
}
template <typename F>
void wait(F condition, const char* text) {
    auto until = Clock::now() + 2s;
    while (Clock::now() < until) {
        if (condition()) {
            return;
        }
        std::this_thread::sleep_for(5ms);
    }
    require(false, text);
}
std::string bind_random(void* socket) {
    require(zmq_bind(socket, "tcp://127.0.0.1:*") == 0, "bind");
    char endpoint[128];
    std::size_t size = sizeof(endpoint);
    zmq_getsockopt(socket, ZMQ_LAST_ENDPOINT, endpoint, &size);
    return endpoint;
}
// 2026-11-25 batch 6c 리뷰 수정: 예전에는 fixture 의 context 로 포트를 잡고 zmq_close 한 뒤 바로
// SUB 를 connect 했다. zmq_close 는 listener 를 I/O 스레드에서 나중에 닫으므로, 부하가 있으면 SUB
// 가 아직 살아 있는 listener 에 붙었다가 listener 쪽이 끊어 그 포트에 TIME-WAIT 이 남고 백엔드 bind
// 가 EADDRINUSE 로 실패했다 (부하에서 측정: 실패 때 ss 에 그 포트의 TIME-WAIT 이 있었다). 따로 만든
// context 로 잡고 zmq_ctx_term 으로 listener 가 실제로 닫힌 뒤에 돌려준다
std::string reserve_control_endpoint() {
    void* scratch = zmq_ctx_new();
    require(scratch != nullptr, "reservation context");
    void* reservation = zmq_socket(scratch, ZMQ_PUB);
    const int linger = 0;
    zmq_setsockopt(reservation, ZMQ_LINGER, &linger, sizeof(linger));
    std::string endpoint = bind_random(reservation);
    zmq_close(reservation);
    zmq_ctx_term(scratch);
    return endpoint;
}
struct Fixture {
    std::atomic<bool> stop{false}, silent{false}, rc{false}, capability{true}, wrong_id{false};
    std::atomic<bool> offboard{false};
    std::atomic<bool> reject_hold{false};
    std::atomic<std::uint64_t> epoch{1};
    std::atomic<int> rpc_count{0}, velocity_count{0}, heading_count{0};
    std::atomic<int> status_count{0};
    std::atomic<int> rpc_delay_ms{0};
    std::atomic<bool> si_only{false}, unit_conflict{false};
    std::atomic<bool> indoor_batch{false};
    // 모드 요청에 응답하기 전에 완료 이벤트부터 보낸다 (이미 그 모드인 게이트웨이와 같은 순서)
    std::atomic<bool> event_before_reply{false};
    // 발행 스레드가 내보낸 모드 이벤트 수 (요청 응답 스레드가 이벤트 송신을 기다린다)
    std::atomic<std::uint64_t> events_queued{0}, events_published{0};
    // 이벤트가 나간 뒤 응답까지 더 기다리는 시간. rpc_timeout보다 길면 응답은 REQ 수신 제한 뒤에
    // 나간다 (백엔드는 빈 응답을 받는다)
    std::atomic<int> reply_after_event_ms{20};
    // 먼저 보내는 이벤트를 해석할 수 없는 본문으로 보낸다 (2026-11-25 batch 6a: protobuf 에서는
    // 비교 필드 revision 이 빠진 이벤트를 만들 수 없다. to_json 이 늘 채운다)
    std::atomic<bool> malformed_early_event{false};
    // 보낸 응답 수 (응답 대기 중에 검사했는지 확인한다)
    std::atomic<int> reply_count{0};
    // 마지막 요청을 받은 시각 (now_ns 기준). 응답 대기 시간을 백엔드 쪽 sleep과 무관하게 잰다
    std::atomic<std::int64_t> last_rpc_ns{0};
    // 2026-11-25 batch 6b: 6372 받기 (team_wire). 팀 seq 와 이름, 본문 규칙 위반 수
    std::atomic<std::uint64_t> last_velocity_team_seq{0}, last_status_team_seq{0};
    std::atomic<int> control_rejected{0};
    // 2026-11-25 batch 7: 6371 받기 (team_wire). [control/mode_request][ControlModeRequest] 가 아닌
    // 요청 수와 응답 모양 (0 정상, 1 빈 본문, 2 예전 이름 control/mode_result, 3 예전 한 프레임
    // msgpack)
    std::atomic<int> rpc_rejected{0};
    std::atomic<int> reply_shape{0};
    std::mutex mutex;
    NativeJson last_velocity, last_heading;
    NativeJson last_status;
    NativeJson last_release;
    NativeJson last_mode;
    std::vector<NativeJson> events;
    std::string telemetry_endpoint, rpc_endpoint, control_endpoint;
    void* context{zmq_ctx_new()};
    // 실제 게이트웨이처럼 FC 텔레메트리 발행(worker)과 요청 응답(responder)은 따로 돈다. 응답이
    // 늦어도 발행은 계속된다. 소켓은 각 스레드에서만 만들고 쓰고 닫는다
    std::thread worker, responder;
    Fixture() {
        auto promise = std::make_shared<std::promise<void>>();
        auto future = promise->get_future();
        worker = std::thread([this, promise] {
            void* pub = zmq_socket(context, ZMQ_PUB);
            void* sub = zmq_socket(context, ZMQ_SUB);
            telemetry_endpoint = bind_random(pub);
            control_endpoint = reserve_control_endpoint();
            zmq_connect(sub, control_endpoint.c_str());
            zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
            promise->set_value();
            argus::team_wire::TeamWire wire;          // 6370 팀 seq (토픽별 발행 번호)
            argus::team_wire::TeamWire control_wire;  // 6372 받기
            while (!stop) {
                if (!silent) {
                    auto stamp = now_ns();
                    NativeJson common{{"uid", uid},
                                      {"fc_epoch", epoch.load()},
                                      {"onboard_session", "onboard-test"},
                                      {"clock_domain", "host_monotonic"},
                                      {"received_monotonic_ns", stamp}};
                    auto attitude = common;
                    attitude.update({{"schema", "argus.fc.attitude.v1"},
                                     {"roll_rad", .1},
                                     {"pitch_rad", .2},
                                     {"yaw_rad", .3}});
                    auto state = common;
                    state.update({{"schema", "argus.fc.state.v1"},
                                  {"connected", true},
                                  {"armed", true},
                                  {"in_air", true},
                                  {"offboard", offboard.load()},
                                  {"hold", !offboard.load()},
                                  {"position", false},
                                  {"rc_override", rc.load()},
                                  {"ground_known", true},
                                  {"landed", false},
                                  {"local_ok", true},
                                  {"flow_valid", true},
                                  {"flow_quality", 90},
                                  {"north_m", 1.0},
                                  {"east_m", 2.0},
                                  {"down_m", -1.0},
                                  {"vn_m_s", .1},
                                  {"ve_m_s", .2},
                                  {"vd_m_s", .3},
                                  {"distance_cm", 100},
                                  {"min_distance_cm", 5},
                                  {"max_distance_cm", 500},
                                  {"distance_orientation", 25},
                                  {"position_monotonic_ns", stamp},
                                  {"distance_monotonic_ns", stamp}});
                    if (indoor_batch) {
                        state["flight_environment"] = "indoor";
                        for (const auto* kind :
                             {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
                            state["cmc_pose_history"][kind] = NativeJson::array();
                            for (int i = 0; i < 4; ++i) {
                                NativeJson raw;
                                if (std::string(kind) == "ATTITUDE") {
                                    raw = {{"roll", .1}, {"pitch", .2}, {"yaw", .3}};
                                } else if (std::string(kind) == "LOCAL_POSITION_NED") {
                                    raw = {{"x", 1.},  {"y", 2.},  {"z", -1.},
                                           {"vx", .1}, {"vy", .2}, {"vz", .3}};
                                } else {
                                    raw = {{"orientation", 25},
                                           {"current_distance", 100},
                                           {"min_distance", 5},
                                           {"max_distance", 500}};
                                }
                                raw["time_boot_ms"] = 123456 + i * 16;
                                state["cmc_pose_history"][kind].push_back(
                                    {{"received_monotonic_ns", stamp - (4 - i) * 16000000},
                                     {"data", raw}});
                            }
                        }
                    }
                    if (si_only) {
                        state["distance_m"] = 1.;
                        state["min_distance_m"] = .05;
                        state["max_distance_m"] = 5.;
                        for (auto key : {"distance_cm", "min_distance_cm", "max_distance_cm"}) {
                            state.erase(key);
                        }
                        if (unit_conflict) {
                            state["distance_cm"] = 101.;
                        }
                    }
                    for (const auto& item : {std::make_pair("fc/attitude", attitude),
                                             std::make_pair("fc/state", state)}) {
                        send_fc(pub, wire, item.first, item.second);
                    }
                }
                zmq_pollitem_t item{sub, 0, ZMQ_POLLIN, 0};
                zmq_poll(&item, 1, 5);
                std::vector<NativeJson> queued;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    queued.swap(events);
                }
                for (const auto& event : queued) {
                    // 2026-11-25 6a 리뷰: 구조가 틀린 fc/mode_event (1 프레임, 또는 본문 뒤 프레임
                    // 하나 더)
                    const int frames = event.value("fixture_frames", 2);
                    if (frames == 1) {
                        zmq_send(pub, "fc/mode_event", 13, 0);
                        continue;
                    }
                    if (frames == 3) {
                        const argus::team_wire::Bytes extra{0x01};
                        argus::team_wire::send_raw(zmq::socket_ref(zmq::from_handle, pub),
                                                   "fc/mode_event", {0x0a, 0x00}, &extra);
                        continue;
                    }
                    if (event.value("fixture_oversized_state", false)) {
                        // 2026-11-25 batch 6c: 본문 상한 (team_wire 1 MiB) 을 넘는 fc/state
                        const argus::team_wire::Bytes big(argus::team_wire::kMaxHeaderBytes + 1, 0);
                        argus::team_wire::send_raw(zmq::socket_ref(zmq::from_handle, pub),
                                                   "fc/state", big, nullptr);
                        continue;
                    }
                    if (event.value("fixture_undecodable", false)) {
                        argus::team_wire::send_raw(zmq::socket_ref(zmq::from_handle, pub),
                                                   "fc/mode_event", {0xff, 0xff, 0xff}, nullptr);
                        continue;
                    }
                    send_fc(pub, wire, "fc/mode_event", event);
                }
                events_published += queued.size();
                if (item.revents & ZMQ_POLLIN) {
                    // 2026-11-25 batch 6b: 이름 정확 일치, 프레임 수, 크기 상한, protobuf 해석
                    argus::team_wire::Received got;
                    const auto status = control_wire.receive(zmq::socket_ref(zmq::from_handle, sub),
                                                             control_topics(), got);
                    if (status == argus::team_wire::ReceiveStatus::Ok) {
                        const std::scoped_lock lock(mutex);
                        if (got.name == "control/setpoint") {
                            last_velocity = got.value;
                            last_velocity_team_seq = got.sequence;
                            ++velocity_count;
                        } else {
                            last_status = got.value;
                            last_status_team_seq = got.sequence;
                            ++status_count;
                        }
                    } else if (status != argus::team_wire::ReceiveStatus::NoMessage) {
                        ++control_rejected;
                    }
                }
            }
            int zero = 0;
            for (auto socket : {pub, sub}) {
                zmq_setsockopt(socket, ZMQ_LINGER, &zero, sizeof(zero));
                zmq_close(socket);
            }
        });
        future.get();
        auto answering = std::make_shared<std::promise<void>>();
        auto answered = answering->get_future();
        responder = std::thread([this, answering] {
            void* rep = zmq_socket(context, ZMQ_REP);
            rpc_endpoint = bind_random(rep);
            answering->set_value();
            int pending = 0;
            argus::team_wire::TeamWire wire;  // 6371 받기와 응답 (이 스레드만)
            while (!stop) {
                zmq_pollitem_t item{rep, 0, ZMQ_POLLIN, 0};
                zmq_poll(&item, 1, 5);
                if ((item.revents & ZMQ_POLLIN) != 0) {
                    answer(rep, pending, wire);
                }
            }
            int zero = 0;
            zmq_setsockopt(rep, ZMQ_LINGER, &zero, sizeof(zero));
            zmq_close(rep);
        });
        answered.get();
    }
    // 요청 하나에 응답한다 (responder 스레드)
    // 2026-11-25 batch 7: 6371 은 팀 [control/mode_request][ControlModeRequest] 하나다 (team_wire).
    // 모드 요청은 값의 schema 로 구분한다 (게이트웨이와 같다). 해석하지 못한 요청은 게이트웨이처럼
    // [이름][빈 본문] 으로 답하고 센다
    void answer(void* rep, int& pending, argus::team_wire::TeamWire& wire) {
        const zmq::socket_ref socket(zmq::from_handle, rep);
        NativeJson request;
        const auto received = wire.receive_request(socket, argus::team_wire::kControlModeRequest,
                                                   request, zmq::recv_flags::none);
        if (received != argus::team_wire::ReceiveStatus::Ok) {
            ++rpc_rejected;
            wire.send_empty_reply(socket, argus::team_wire::kControlModeRequest);
            return;
        }
        const bool mode_wire = request.value("schema", "") == "argus.control.mode_request.v1";
        last_rpc_ns = now_ns();
        ++rpc_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(rpc_delay_ms.load()));
        NativeJson response{{"schema", "argus.follow.rpc.v1"},
                            {"request_id", request.at("request_id")},
                            {"client_session", request.at("client_session")},
                            {"onboard_session", request.at("onboard_session")},
                            {"fc_epoch", request.at("fc_epoch")},
                            {"status", "accepted"}};
        auto action = request.at("action").get<std::string>();
        if (wrong_id) {
            response["request_id"] = "incorrect";
        }
        if (action == "acquire") {
            response["owner_token"] = "fixture-owner-token";
            response["capabilities"] =
                capability ? NativeJson::array({"context_v1", "absolute_heading_rad"})
                           : NativeJson::array({"context_v1"});
        }
        if (action == "start_offboard" || action == "hold") {
            response["status"] = "pending";
            pending = 0;
        }
        if (action == "hold" && reject_hold.load()) {
            response["status"] = "failed";
            response["reason"] = "fixture_hold_denied";
        }
        if (action == "request_status") {
            response["status"] = ++pending > 1 ? "accepted" : "pending";
        }
        if (action == "heading") {
            ++heading_count;
            const std::scoped_lock lock(mutex);
            last_heading = request;
        }
        if (action == "release") {
            const std::scoped_lock lock(mutex);
            last_release = request;
        }
        if (mode_wire) {
            {
                const std::scoped_lock lock(mutex);
                last_mode = request;
            }
            response.update({{"schema", "argus.control.mode_result.v1"},
                             {"command_id", request.at("command_id")},
                             {"session_id", "onboard-test"},
                             {"requester_session_id", request.at("session_id")},
                             {"uid", uid},
                             {"result", "ACCEPTED"},
                             {"source", "argus-onboard"},
                             {"seq", rpc_count.load()},
                             {"clock_domain", "host_monotonic"},
                             {"published_monotonic_ns", now_ns()}});
            if (event_before_reply) {
                publish_early_event(request, action);
            }
        }
        // 보내기 전에 센다. 보낸 뒤에 세면 백엔드가 응답을 먼저 처리해 다음 시험 단계가 이전
        // 응답 수를 읽을 수 있다
        ++reply_count;
        switch (reply_shape.load()) {
            case 1:  // 팀 replier 의 처리 실패 응답 모양
                wire.send_empty_reply(socket, argus::team_wire::kControlModeRequest);
                break;
            case 2: {  // 예전 응답 이름에 맞는 protobuf 본문
                argus::team_wire::Bytes body;
                std::vector<std::string> unknown;
                std::string error;
                argus::team_wire::encode_body(argus::team_wire::kControlModeRequest, true, response,
                                              0, body, unknown, error);
                argus::team_wire::send_raw(socket, "control/mode_result", body, nullptr);
                break;
            }
            case 3: {  // 예전 한 프레임 msgpack
                const auto bytes = NativeJson::to_msgpack(response);
                zmq_send(rep, bytes.data(), bytes.size(), 0);
                break;
            }
            default:
                wire.send_reply(socket, argus::team_wire::kControlModeRequest, response);
                break;
        }
    }
    // 응답 전에 완료 이벤트를 발행 스레드로 내보내고, 나간 뒤 reply_after_event_ms를 더 기다린다
    // (이벤트가 응답보다 먼저 닿게 한다)
    void publish_early_event(const NativeJson& request, const std::string& action) {
        static std::uint64_t early_seq = 1000000;
        NativeJson early{{"schema", "argus.fc.mode_event.v1"},
                         {"uid", uid},
                         {"onboard_session", "onboard-test"},
                         {"session_id", "onboard-test"},
                         {"requester_session_id", request.at("session_id")},
                         {"fc_epoch", epoch.load()},
                         {"clock_domain", "host_monotonic"},
                         {"received_monotonic_ns", now_ns()},
                         {"seq", ++early_seq},
                         {"command_id", request.at("command_id")},
                         {"event", "MODE_COMPLETED"},
                         {"actual_mode", action == "hold" ? "HOLD" : "OFFBOARD"},
                         {"revision", request.at("revision")},
                         {"command_epoch", request.at("command_epoch")},
                         {"owner_token", "fixture-owner-token"},
                         {"source", "argus-onboard"},
                         {"reason", "actual_mode_confirmed"}};
        if (malformed_early_event) {
            early = NativeJson{{"fixture_undecodable", true}};
        }
        std::uint64_t target = 0;
        {
            const std::scoped_lock lock(mutex);
            events.push_back(early);
            target = ++events_queued;
        }
        const auto until = Clock::now() + 500ms;
        while (events_published.load() < target && Clock::now() < until) {
            std::this_thread::sleep_for(1ms);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(reply_after_event_ms.load()));
    }
    ~Fixture() {
        stop = true;
        worker.join();
        responder.join();
        zmq_ctx_term(context);
    }
};
void mode_event_test() {
    Fixture fixture;
    fixture.si_only = true;
    ZeroMqFollowBackendConfig config;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    config.mode_result_timeout_ms = 250;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "mode-event telemetry");
    require(backend.acquire(), "mode-event acquire");
    require(backend.observations()
                    .at("history")
                    .at("DISTANCE_SENSOR")
                    .back()
                    .at("data")
                    .at("current_distance") == 100.,
            "SI distance converted once at legacy projector boundary");
    std::uint64_t sequence = 0;
    auto request = [&] {
        backend.update_context({1, 2, 3, now_ns(), false});
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            fixture.last_mode = nullptr;
        }
        require(backend.mode().start_offboard(), "event mode queued");
        wait(
            [&] {
                std::lock_guard<std::mutex> lock(fixture.mutex);
                return fixture.last_mode.is_object();
            },
            "mode multipart received");
        std::this_thread::sleep_for(10ms);
        std::lock_guard<std::mutex> lock(fixture.mutex);
        return fixture.last_mode;
    };
    auto event_for = [&](const NativeJson& original) {
        return NativeJson{{"schema", "argus.fc.mode_event.v1"},
                          {"uid", uid},
                          {"onboard_session", "onboard-test"},
                          {"session_id", "onboard-test"},
                          {"requester_session_id", backend.controller_session()},
                          {"fc_epoch", fixture.epoch.load()},
                          {"clock_domain", "host_monotonic"},
                          {"received_monotonic_ns", now_ns()},
                          {"seq", ++sequence},
                          {"command_id", original.at("request_id")},
                          {"event", "MODE_COMPLETED"},
                          {"actual_mode", "OFFBOARD"},
                          {"revision", original.at("revision")},
                          {"command_epoch", 1}};
    };
    auto emit = [&](NativeJson event) {
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            fixture.events.push_back(event);
        }
        std::this_thread::sleep_for(20ms);
    };
    auto original = request();
    require(!backend.mode().start_result().has_value(), "ACCEPTED is not mode completion");
    auto wrong = event_for(original);
    wrong["command_id"] = "wrong";
    emit(wrong);
    wrong = event_for(original);
    wrong["onboard_session"] = "old-session";
    emit(wrong);
    wrong = event_for(original);
    wrong["received_monotonic_ns"] = now_ns() - 1000000000;
    emit(wrong);
    wrong = event_for(original);
    wrong["requester_session_id"] = "other-controller";
    emit(wrong);
    require(backend.owns() && !backend.mode().start_result().has_value(),
            "old events do not complete or replace current context");
    emit(event_for(original));
    require(backend.mode().start_result() == true && backend.flight_state().read().hold,
            "event ACK still does not overwrite actual FC telemetry");
    fixture.wrong_id = true;
    original = request();
    const auto after_admission = fixture.rpc_count.load();
    require(!backend.mode().start_result().has_value(),
            "unconfirmed REP remains pending, not automatically retried");
    emit(event_for(original));
    require(backend.mode().start_result() == true && fixture.rpc_count == after_admission,
            "correlated event resolves unknown REP without duplicate execution");
    fixture.wrong_id = false;
    original = request();
    auto failed = event_for(original);
    failed["event"] = "MODE_FAILED";
    emit(failed);
    require(backend.mode().start_result() == false, "failure event fails closed");
    original = request();
    backend.vehicle().revoke_commands();
    emit(event_for(original));
    require(backend.mode().start_result() != true, "revoked request late event rejected");
    original = request();
    wait([&] { return backend.mode().start_result().has_value(); },
         "missing event bounded timeout");
    require(backend.mode().start_result() == false, "missing event fails closed");
    emit(event_for(original));
    require(backend.mode().start_result() == false, "late event never revives timed-out request");
    original = request();
    auto exit = event_for(original);
    exit["event"] = "MODE_EXIT";
    exit["reason"] = "pilot_took_over";
    exit["owner_token"] = "old-token";
    emit(exit);
    require(backend.owns(), "old lease exit must not revoke a new owner");
    exit["seq"] = ++sequence;
    exit["owner_token"] = "fixture-owner-token";
    emit(exit);
    require(!backend.owns(), "unsolicited pilot mode exit revokes authority");
    std::this_thread::sleep_for(20ms);
    require(!backend.owns(), "telemetry alone does not resume ownership");
    fixture.unit_conflict = true;
    wait([&] { return !backend.identity_permit(); }, "conflicting distance aliases rejected");
    fixture.unit_conflict = false;
    wait([&] { return backend.identity_permit(); }, "SI telemetry recovers");
    require(!backend.owns(), "unit recovery does not resume authority");
}

// 2026-11-24 요청이 응답을 기다리는 동안에도 FC 자세와 상태를 계속 받는다 (요청 처리와
// 텔레메트리 수신 분리). 예전 구조(한 스레드)에서는 응답 대기 230 ms 동안 텔레메트리가 갱신되지
// 않아 신선도(이 시험은 100 ms)가 끊기고 제어권이 무효화됐다
void telemetry_during_slow_rpc_test() {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.mode_events = false;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    config.rpc_timeout_ms = 250;
    config.telemetry_ttl_ms = 100;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.pose().read_aligned(now_ns(), "host_monotonic", 50ms).has_value(); },
         "slow-rpc fixture telemetry");
    require(backend.acquire(), "slow-rpc fixture authority");
    fixture.rpc_delay_ms = 230;  // rpc_timeout(250 ms)보다 짧다: 응답은 결국 온다
    const auto before = fixture.rpc_count.load();
    const auto replies_before = fixture.reply_count.load();
    backend.update_context({1, 2, 3, now_ns(), false});
    require(backend.mode().stop_offboard(), "slow HOLD queued");
    wait([&] { return fixture.rpc_count.load() > before; }, "slow HOLD reached fixture");
    // 횟수가 아니라 시점으로 판정한다 (부하로 sleep이 길어져도 흔들리지 않게). 응답이 나가기 전,
    // 신선도 한도(100 ms)를 넘긴 시점에 한 번 이상 검사했으면 된다
    int late_checks = 0;
    const auto until = Clock::now() + 1s;
    while (fixture.reply_count.load() == replies_before && Clock::now() < until) {
        const auto elapsed = std::chrono::nanoseconds(now_ns() - fixture.last_rpc_ns.load());
        require(backend.pose().read_aligned(now_ns(), "host_monotonic", 50ms).has_value(),
                "pose stays aligned while an RPC waits for its reply");
        require(backend.identity_permit(), "telemetry stays fresh while an RPC waits");
        require(backend.owns(), "authority stays valid while an RPC waits");
        if (elapsed >= 110ms && fixture.reply_count.load() == replies_before) {
            ++late_checks;
        }
        std::this_thread::sleep_for(2ms);
    }
    require(late_checks >= 1, "checked past the telemetry TTL while the RPC still waited");
    wait([&] { return backend.mode().hold_result().has_value(); }, "slow HOLD answered");
    require(backend.owns(), "authority survives the slow RPC");
    fixture.rpc_delay_ms = 0;
    backend.release();
}

// 2026-11-24 게이트웨이가 모드 요청 응답보다 완료 이벤트를 먼저 보내도 결과를 잃지 않는다
// (요청 처리와 텔레메트리 수신이 다른 스레드라 이벤트가 응답 처리보다 먼저 들어올 수 있다)
void early_mode_event_test() {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    config.mode_result_timeout_ms = 500;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "early-event telemetry");
    require(backend.acquire(), "early-event authority");
    fixture.event_before_reply = true;
    backend.update_context({1, 2, 3, now_ns(), false});
    require(backend.mode().start_offboard(), "early-event start queued");
    wait([&] { return backend.mode().start_result().has_value(); }, "early-event start result");
    require(backend.mode().start_result() == true,
            "completion event before the RPC reply is applied");
    backend.update_context({1, 2, 3, now_ns(), false});
    require(backend.mode().stop_offboard(), "early-event hold queued");
    wait([&] { return backend.mode().hold_result().has_value(); }, "early-event hold result");
    require(backend.mode().hold_result() == true,
            "HOLD completion before the RPC reply is applied");
    // 이벤트는 REQ 수신 제한(rpc_timeout 100 ms) 전에 오고 응답은 그 뒤에 나간다. 백엔드는 빈
    // 응답을 받지만 FC가 이미 완료했으므로 결과는 true다 (예전 단일 스레드 구조와 같다)
    fixture.reply_after_event_ms = 130;
    backend.update_context({1, 2, 3, now_ns(), false});
    require(backend.mode().start_offboard(), "late-reply start queued");
    wait([&] { return backend.mode().start_result().has_value(); }, "late-reply start result");
    require(backend.mode().start_result() == true,
            "completion event before an RPC timeout survives the empty reply");
    backend.update_context({1, 2, 3, now_ns(), false});
    require(backend.mode().stop_offboard(), "late-reply hold queued");
    wait([&] { return backend.mode().hold_result().has_value(); }, "late-reply hold result");
    require(backend.mode().hold_result() == true,
            "HOLD completion before an RPC timeout survives the empty reply");
    fixture.reply_after_event_ms = 20;
    // 2026-11-25 batch 6a: 6370 protobuf 에서는 비교 필드가 빠진 이벤트를 보낼 수 없다 (받는 쪽
    // to_json 이 revision 을 늘 채운다). 해석할 수 없는 본문의 이른 이벤트는 버려지고 요청은
    // 완료되지 않는다 (mode_result_timeout 뒤 실패. 예전 msgpack 해석 실패 본문과 같은 경로)
    std::this_thread::sleep_for(150ms);  // 앞 요청의 늦은 응답이 끝나기를 기다린다
    fixture.malformed_early_event = true;
    backend.update_context({1, 2, 3, now_ns(), false});
    require(backend.mode().start_offboard(), "malformed-event start queued");
    wait([&] { return backend.mode().start_result().has_value(); },
         "undecodable early event result");
    require(backend.mode().start_result() != true, "undecodable early event never completes");
    fixture.malformed_early_event = false;
    fixture.event_before_reply = false;
    backend.release();
}

// 2026-11-25 6a 리뷰: fc/mode_event 의 구조 오류는 예전 msgpack 수신과 같게 나눈다. 본문 뒤에
// 프레임이 더 있는 이벤트는 예전처럼 빈 값 경로로 버리고 (권한과 신선도 유지), 본문 프레임이 없는 1
// 프레임 이벤트는 예전처럼 바로 무효다. f934c74 는 둘 다 무효로 했다
void malformed_mode_event_frames_test() {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "frame fixture telemetry");
    require(backend.acquire(), "frame fixture authority");
    const auto emit = [&](int frames) {
        const auto before = fixture.events_published.load();
        {
            const std::scoped_lock lock(fixture.mutex);
            fixture.events.push_back({{"fixture_frames", frames}});
        }
        wait([&] { return fixture.events_published.load() > before; }, "malformed event published");
        std::this_thread::sleep_for(60ms);  // 수신 스레드가 처리할 시간 (state 는 5 ms 마다 온다)
    };
    emit(3);
    require(backend.owns() && backend.identity_permit(),
            "three-frame mode_event is dropped like the old msgpack path (authority kept)");
    emit(1);
    wait([&] { return !backend.owns(); }, "one-frame mode_event invalidates like the old path");
    backend.release();
}

// 2026-11-25 batch 6c: 6370 fc/ SUB 에 ZMQ_MAXMSGSIZE 를 두지 않는다. 본문 상한을 넘는 fc/state 는
// 예전 처리처럼 해석 실패 경로 (권한 무효, telemetry_decode_failures 1 증가) 이고, 같은 연결로 그
// 뒤 FC 메시지를 계속 받아 신선도가 돌아온다 (자동 재획득은 없다). 8ca3440 은 MAXMSGSIZE 로 연결이
// 끊겨 돌아오지 않았다 (drone libzmq 4.3.5 는 다시 맺지 않는다, ~/work/b7-probe/m.c)
void oversized_fc_frame_keeps_connection_test() {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    config.telemetry_ttl_ms = 100;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "oversized fixture telemetry");
    require(backend.acquire(), "oversized fixture authority");
    const auto before = backend.telemetry_decode_failures();
    const auto published = fixture.events_published.load();
    {
        const std::scoped_lock lock(fixture.mutex);
        fixture.events.push_back({{"fixture_oversized_state", true}});
    }
    wait([&] { return fixture.events_published.load() > published; }, "oversized fc/state sent");
    wait([&] { return backend.telemetry_decode_failures() == before + 1; },
         "oversized fc/state is received and counted as a decode failure");
    wait([&] { return !backend.owns(); },
         "oversized fc/state invalidates like the old empty value");
    // 신선도 한도를 넘게 기다린 뒤에도 신선하면 같은 연결로 새 FC 메시지를 받고 있다
    std::this_thread::sleep_for(150ms);
    wait(
        [&] {
            return backend.identity_permit() &&
                   backend.pose().read_aligned(now_ns(), "host_monotonic", 50ms).has_value();
        },
        "FC telemetry continues on the same connection after the oversized frame");
    require(!backend.owns(), "no automatic reacquire after the oversized frame");
    require(backend.telemetry_decode_failures() == before + 1,
            "later small frames are not failures");
    backend.release();
}

// 2026-11-25 batch 6b: command_results 행 수. 생산자 보관 한도 (IcdCommandProgress) 는 32 라 넘는
// 일이 없다. 넘으면 to_proto 가 거부해 follow/status 를 보내지 않는다 (예전 msgpack 은 보냈다).
// 게이트웨이는 3 s 뒤 follow_status_timeout 으로 멈춘다 (fail closed). 32 행은 보낸다
void command_results_limit_test() {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    ZeroMqFollowBackend backend(config);
    backend.publish_status(status_with_rows(32));
    wait([&] { return fixture.status_count.load() == 1; }, "32-row status published");
    {
        const std::scoped_lock lock(fixture.mutex);
        require(fixture.last_status.at("status").at("command_results").size() == 32,
                "32 command_results rows kept");
    }
    backend.publish_status(status_with_rows(33));
    // 바뀐 status 와 1 s 마다 다시 보내기 모두 변환 실패로 나가지 않는다
    std::this_thread::sleep_for(1300ms);
    require(fixture.status_count.load() == 1 && fixture.control_rejected.load() == 0,
            "33-row status is not published (encode rejected, fail closed)");
    backend.publish_status(status_with_rows(31));
    wait([&] { return fixture.status_count.load() == 2; }, "status resumes within the limit");
    require(fixture.last_status_team_seq.load() >= 3,
            "team seq counts the rejected publishes (ZmqBus numbers every attempt)");
}

// 2026-11-25 batch 6b 합성 측정: setpoint 50 ms 주기 도착 간격. 추종 제어처럼 50 ms 마다 문맥을
// 갱신하고 속도를 보내며 (status 도 tick 마다 바뀌어 함께 나간다), 따로 연결한 SUB 가
// control/setpoint 의 도착 시각을 잰다. 이름만 보므로 예전 msgpack 백엔드에도 같은 측정을 쓸 수
// 있다. 장비 없음, loopback tcp
void measure_setpoint(int count) {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.mode_events = false;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "measure telemetry");
    require(backend.acquire(), "measure authority");
    fixture.offboard = true;
    wait([&] { return backend.flight_state().read().offboard; }, "measure Offboard telemetry");
    std::vector<std::int64_t> sent, arrived;
    sent.reserve(count);
    arrived.reserve(count);
    std::atomic<bool> done{false};
    void* sub = zmq_socket(fixture.context, ZMQ_SUB);
    zmq_connect(sub, fixture.control_endpoint.c_str());
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "control/setpoint", 16);
    int timeout = 20;
    zmq_setsockopt(sub, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    std::this_thread::sleep_for(200ms);
    std::thread reader([&] {
        while (!done) {
            zmq_msg_t name;
            zmq_msg_init(&name);
            if (zmq_msg_recv(&name, sub, 0) < 0) {
                zmq_msg_close(&name);
                continue;
            }
            const auto stamp = now_ns();
            int more = zmq_msg_more(&name);
            zmq_msg_close(&name);
            while (more) {
                zmq_msg_t part;
                zmq_msg_init(&part);
                zmq_msg_recv(&part, sub, 0);
                more = zmq_msg_more(&part);
                zmq_msg_close(&part);
            }
            arrived.push_back(stamp);
        }
    });
    auto status = recorded_status();
    auto next = Clock::now() + 50ms;
    for (int i = 0; i < count; ++i) {
        std::this_thread::sleep_until(next);
        next += 50ms;
        backend.update_context({1, 2, 3, now_ns(), false});
        status["mission_state_code"] = i % 2;  // tick 마다 바뀌는 status (함께 나간다)
        backend.publish_status(status);
        sent.push_back(now_ns());
        backend.vehicle().send_until({0.1, 0, 0, 0, "follow"}, Clock::now() + 150ms);
    }
    std::this_thread::sleep_for(200ms);
    done = true;
    reader.join();
    zmq_close(sub);
    std::vector<double> intervals, deviations;
    for (std::size_t i = 1; i < arrived.size(); ++i) {
        const double interval = static_cast<double>(arrived[i] - arrived[i - 1]) / 1e6;
        intervals.push_back(interval);
        deviations.push_back(std::abs(interval - 50.0));
    }
    auto report = [](const char* label, std::vector<double> values) {
        if (values.empty()) {
            std::printf("%s n=0\n", label);
            return;
        }
        std::sort(values.begin(), values.end());
        double sum = 0;
        for (const double value : values) {
            sum += value;
        }
        const auto at = [&](double q) {
            return values[static_cast<std::size_t>(q * static_cast<double>(values.size() - 1))];
        };
        std::printf("%s n=%zu mean=%.3f p50=%.3f p99=%.3f max=%.3f ms\n", label, values.size(),
                    sum / static_cast<double>(values.size()), at(.5), at(.99), values.back());
    };
    std::printf("SETPOINT_MEASURE sent=%zu arrived=%zu\n", sent.size(), arrived.size());
    report("SETPOINT_INTERVAL", intervals);
    report("SETPOINT_JITTER |interval-50|", deviations);
    backend.release();
}

// 2026-11-25 batch 7: 6371 요청-응답 왕복 시간. acquire 와 release 를 번갈아 count 번씩 부르고
// 호출부터 돌아올 때까지를 잰다 (요청 대기열, worker 깨우기, REQ 연결, 변환, 가짜 게이트웨이
// 응답, 신원 검사 포함). 예전 백엔드 (msgpack) 에도 같은 함수를 쓴다. 장비 없음, loopback tcp
void measure_rpc(int count) {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.mode_events = false;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "rpc measure telemetry");
    std::vector<double> acquire_ms, release_ms;
    int failures = 0;
    for (int i = 0; i < count; ++i) {
        auto start = now_ns();
        if (!backend.acquire()) {
            ++failures;
        }
        acquire_ms.push_back(static_cast<double>(now_ns() - start) / 1e6);
        start = now_ns();
        backend.release();
        release_ms.push_back(static_cast<double>(now_ns() - start) / 1e6);
    }
    auto report = [](const char* label, std::vector<double> values) {
        std::sort(values.begin(), values.end());
        double sum = 0;
        for (const double value : values) {
            sum += value;
        }
        const auto at = [&](double q) {
            return values[static_cast<std::size_t>(q * static_cast<double>(values.size() - 1))];
        };
        std::printf("%s n=%zu mean=%.3f p50=%.3f p99=%.3f max=%.3f ms\n", label, values.size(),
                    sum / static_cast<double>(values.size()), at(.5), at(.99), values.back());
    };
    std::printf("RPC_MEASURE count=%d acquire_failures=%d rpc_count=%d\n", count, failures,
                fixture.rpc_count.load());
    if (count > 0) {
        report("RPC_ACQUIRE", acquire_ms);
        report("RPC_RELEASE", release_ms);
    }
}

// 2026-11-25 batch 7: 6371 응답 모양. 팀 [control/mode_request][ControlModeReply] 만 받는다. 빈
// 본문 (팀 replier 와 게이트웨이의 처리 실패 응답), 예전 이름 control/mode_result, 예전 한 프레임
// msgpack 은 응답 없음과 같다 (acquire 실패, 제어권 없음). 가짜 게이트웨이가 받은 요청은 모두 팀
// 모양이어야 한다 (rpc_rejected 0)
void rpc_reply_shapes_test() {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.mode_events = false;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "reply shape telemetry");
    for (const int shape : {1, 2, 3}) {
        fixture.reply_shape = shape;
        const auto before = fixture.rpc_count.load();
        require(!backend.acquire(), "acquire fails on a non-team 6371 reply");
        require(fixture.rpc_count.load() == before + 1, "the request reached the fixture");
        require(!backend.owns(), "no authority from a non-team 6371 reply");
    }
    fixture.reply_shape = 0;
    require(backend.acquire(), "acquire succeeds on the team reply again (fresh REQ per call)");
    require(backend.owns(), "authority from the team reply");
    backend.release();
    require(fixture.rpc_rejected.load() == 0, "every 6371 request was [control/mode_request][pb]");
}

void hold_queue_full_diagnostic_test() {
    Fixture fixture;
    ZeroMqFollowBackendConfig config;
    config.mode_events = false;
    config.expected_uid = uid;
    config.telemetry_endpoint = fixture.telemetry_endpoint;
    config.rpc_endpoint = fixture.rpc_endpoint;
    config.control_endpoint = fixture.control_endpoint;
    ZeroMqFollowBackend backend(config);
    backend.set_writer_guard([] { return true; });
    backend.update_context({1, 2, 3, now_ns(), false});
    wait([&] { return backend.identity_permit(); }, "queue fixture telemetry");
    require(backend.acquire(), "queue fixture authority");
    fixture.rpc_delay_ms = 150;
    // A blocked mock RPC leaves the actor unable to consume the 16-slot queue.
    bool denied = false;
    for (int i = 0; i < 40; ++i) {
        denied = !backend.mode().stop_offboard() || denied;
    }
    const auto events = backend.take_hold_diagnostics();
    require(denied && std::any_of(events.begin(), events.end(),
                                  [](const auto& event) {
                                      return event.value("stage", "") == "queue" &&
                                             event.value("outcome", "") == "queue_full" &&
                                             event.value("queue_size", 0u) >= 16u;
                                  }),
            "queue-full HOLD refusal and observed capacity recorded");
    fixture.rpc_delay_ms = 0;
    backend.release();
}
}  // namespace
int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "measure") {
            measure_setpoint(std::stoi(argv[2]));
            return 0;
        }
        if (argc == 3 && std::string(argv[1]) == "measure_rpc") {
            measure_rpc(std::stoi(argv[2]));
            return 0;
        }
        {
            Fixture batch;
            batch.indoor_batch = true;
            ZeroMqFollowBackendConfig read;
            read.expected_uid = uid;
            read.read_only = true;
            read.telemetry_endpoint = batch.telemetry_endpoint;
            read.rpc_endpoint = batch.rpc_endpoint;
            read.control_endpoint = batch.control_endpoint;
            ZeroMqFollowBackend observer(read);
            wait(
                [&] {
                    const auto data = observer.observations();
                    for (const auto* kind : {"ATTITUDE", "LOCAL_POSITION_NED", "DISTANCE_SENSOR"}) {
                        bool retained = false;
                        for (const auto& row : data["history"][kind]) {
                            if (!row["data"].value("time_boot_ms", NativeJson{}).is_null()) {
                                retained = true;
                            }
                        }
                        if (!retained) {
                            return false;
                        }
                    }
                    return true;
                },
                "indoor relay preserves intermediate original FC timestamps");
            require(batch.rpc_count == 0 && batch.velocity_count == 0,
                    "sample observation sends no control");
        }
        Fixture fixture;
        ZeroMqFollowBackendConfig config;
        config.mode_events = false;  // 구 RPC의 회귀도 명시적으로 보존한다.
        config.expected_uid = uid;
        config.telemetry_endpoint = fixture.telemetry_endpoint;
        config.rpc_endpoint = fixture.rpc_endpoint;
        config.control_endpoint = fixture.control_endpoint;
        ZeroMqFollowBackend backend(config);
        require(!backend.mode().stop_offboard(), "HOLD without writer guard fails closed");
        auto denied_hold = backend.take_hold_diagnostics();
        require(!denied_hold.empty() && denied_hold.back().at("stage") == "precheck" &&
                    denied_hold.back().at("outcome") == "writer_permit_denied" &&
                    denied_hold.back().at("writer_permit") == false &&
                    !denied_hold.back().contains("owner_token"),
                "HOLD guard denial is recorded without token");
        backend.set_writer_guard([] { return true; });
        auto context = [&] { backend.update_context({1, 2, 3, now_ns(), false}); };
        context();
        wait([&] { return backend.identity_permit(); }, "telemetry identity");
        require(!backend.mode().stop_offboard(), "HOLD without authority denied");
        denied_hold = backend.take_hold_diagnostics();
        require(!denied_hold.empty() &&
                    denied_hold.back().at("outcome") == "authority_or_freshness_denied" &&
                    denied_hold.back().at("authority") == false &&
                    denied_hold.back().at("fc_fresh") == true,
                "authority failure and fresh telemetry explicitly recorded");
        require(std::abs(backend.pose().read().yaw_deg - .3 * 180 / 3.14159265358979323846) < 1e-9,
                "rad/deg conversion");
        require(backend.pose().read_aligned(now_ns(), "host_monotonic", 50ms).has_value(),
                "bounded pose alignment");
        require(!backend.pose().read_aligned(now_ns(), "unknown", 50ms), "clock mismatch rejected");
        auto observations = backend.observations();
        require(observations.at("schema") == "argus.fc.observations.v1", "tracker envelope");
        require(observations.at("history").at("ATTITUDE").size() <= 64, "bounded history");
        fixture.capability = false;
        require(!backend.acquire(), "missing heading capability rejected");
        fixture.capability = true;
        fixture.wrong_id = true;
        require(!backend.acquire(), "request correlation");
        fixture.wrong_id = false;
        require(backend.acquire() && backend.owns(), "authority handshake");
        context();
        require(!backend.vehicle().send({}), "unbounded velocity rejected");
        require(!backend.vehicle().send_until({}, Clock::now() - 1ms),
                "expired caller deadline rejected");
        backend.update_context({1, 2, 3, now_ns() + 1000000000, false});
        require(!backend.vehicle().send_until({}, Clock::now() + 100ms), "future source rejected");
        require(!backend.mode().stop_offboard(), "future source also blocks HOLD");
        denied_hold = backend.take_hold_diagnostics();
        require(!denied_hold.empty() &&
                    denied_hold.back().at("outcome") == "command_permit_denied" &&
                    denied_hold.back().at("request_source_not_future") == false,
                "HOLD failure identifies future source sample");
        context();
        const auto rpc_before_reacquire = fixture.rpc_count.load();
        require(backend.acquire(), "already-owned acquire is idempotent");
        require(fixture.rpc_count.load() == rpc_before_reacquire,
                "live ownership never issues duplicate remote acquire");
        fixture.offboard = true;
        wait([&] { return backend.flight_state().read().offboard; }, "actual Offboard telemetry");
        require(backend.vehicle().send_until({1, 2, 3, 90, "follow"}, Clock::now() + 150ms),
                "bounded velocity");
        wait(
            [&] {
                if (fixture.velocity_count.load() > 0) {
                    return true;
                }
                context();
                backend.vehicle().send_until({1, 2, 3, 90, "follow"}, Clock::now() + 150ms);
                return false;
            },
            "velocity SUB");
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            require(std::abs(fixture.last_velocity.at("yaw_rate_rad_s").get<double>() -
                             3.14159265358979323846 / 2) < 1e-9,
                    "wire yaw radians");
            require(fixture.last_velocity.at("generation") == 2 &&
                        fixture.last_velocity.at("track_id") == 3 &&
                        fixture.last_velocity.at("command_epoch") == 1 &&
                        fixture.last_velocity.at("owner_token") == "fixture-owner-token",
                    "full command metadata");
        }
        {
            // 2026-11-25 batch 6b: 팀 [control/setpoint][protobuf]. 팀 seq 는 발행 번호 (1 부터),
            // Pi 공용 카운터는 seq 키 (pi_seq) 로 그대로, 별칭 forward 등도 되살린다
            const std::scoped_lock lock(fixture.mutex);
            require(fixture.last_velocity.at("schema") == "argus.control.setpoint.v1" &&
                        fixture.last_velocity_team_seq.load() >= 1 &&
                        fixture.last_velocity.at("seq").get<std::uint64_t>() > 0 &&
                        fixture.last_velocity.at("forward") == fixture.last_velocity.at("vx_m_s") &&
                        fixture.control_rejected.load() == 0,
                    "team protobuf setpoint (team seq, pi seq, aliases)");
        }
        context();
        auto before_clamped = fixture.velocity_count.load();
        require(backend.vehicle().send_until({}, Clock::now() + 300ms),
                "production priming deadline shortened to transport TTL");
        wait([&] { return fixture.velocity_count.load() > before_clamped; },
             "clamped priming output");
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            require(fixture.last_velocity.at("valid_until_monotonic_ns").get<std::int64_t>() -
                            fixture.last_velocity.at("issued_monotonic_ns").get<std::int64_t>() <=
                        250000000,
                    "wire priming TTL never exceeds 250 ms");
        }
        std::uint64_t previous_wire_epoch;
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            previous_wire_epoch = fixture.last_velocity.at("command_epoch");
        }
        const auto rpc_before_binding = fixture.rpc_count.load();
        const auto sent_before_binding = fixture.velocity_count.load();
        backend.update_context({1, 2, 4, now_ns(), false});
        require(backend.vehicle().send_until({}, Clock::now() + 150ms),
                "rebound target bounded command");
        wait([&] { return fixture.velocity_count.load() > sent_before_binding; },
             "rebound wire output");
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            require(fixture.last_velocity.at("track_id") == 4 &&
                        fixture.last_velocity.at("command_epoch").get<std::uint64_t>() >
                            previous_wire_epoch &&
                        fixture.last_velocity.at("owner_token") == "fixture-owner-token",
                    "new binding advances remote epoch, retains same authority");
        }
        require(backend.owns() && fixture.rpc_count.load() == rpc_before_binding,
                "binding update neither acquires nor releases authority");
        context();
        fixture.offboard = false;
        wait([&] { return backend.flight_state().read().hold; }, "actual HOLD telemetry");
        context();
        require(backend.send_heading(.7, Clock::now() + 150ms, false), "absolute heading RPC");
        require(fixture.heading_count == 1, "heading never approximated by velocity");
        backend.update_context({1, 2, 3, now_ns() - 1000000000, false});
        require(!backend.send_heading(.7, Clock::now() + 150ms, false),
                "normal heading rejects stale image");
        require(backend.vehicle().send_until({}, Clock::now() + 150ms),
                "bounded pure zero permits stale image in HOLD");
        require(backend.send_heading(.7, Clock::now() + 150ms, true),
                "HOLD terminal freeze accepts fresh pose with stale image");
        context();
        require(backend.mode().start_offboard(), "async mode queued");
        wait([&] { return backend.mode().start_result().has_value(); }, "pending ACK completes");
        require(backend.mode().start_result() == true && backend.flight_state().read().hold,
                "ACK does not overwrite actual mode");
        fixture.rc = true;
        wait([&] { return !backend.owns(); }, "RC revocation");
        fixture.rc = false;
        std::this_thread::sleep_for(30ms);
        require(!backend.owns(), "no RC autoresume");
        context();
        require(backend.acquire(), "explicit reacquire");
        fixture.epoch = 2;
        wait([&] { return !backend.owns(); }, "FC epoch revocation");
        // Epoch revocation arrives with the first sample; both pose streams must be ready.
        wait(
            [&] {
                return backend.identity_permit() &&
                       backend.operator_context().at("fc_epoch") == fixture.epoch.load();
            },
            "new epoch complete telemetry before explicit acquire");
        require(!backend.owns(), "fresh epoch telemetry never restores authority");
        context();
        require(backend.acquire(), "new epoch explicit acquire");
        backend.vehicle().revoke_commands();
        require(backend.owns(), "cache revoke preserves independent authority");
        context();
        require(backend.vehicle().send_until({}, Clock::now() + 150ms),
                "zero remains allowed after cache revoke");
        require(backend.send_heading(.3, Clock::now() + 150ms, false),
                "Scan heading remains allowed after cache revoke");
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            require(fixture.last_heading.at("owner_token") == "fixture-owner-token",
                    "same token survives cache revoke");
        }
        require(backend.mode().stop_offboard(), "HOLD remains allowed after cache revoke");
        wait([&] { return backend.mode().hold_result().has_value(); },
             "HOLD ACK after cache revoke");
        require(backend.mode().hold_result() == true, "HOLD accepted after cache revoke");
        const auto hold_events = backend.take_hold_diagnostics();
        require(std::any_of(hold_events.begin(), hold_events.end(),
                            [](const auto& event) {
                                return event.value("stage", "") == "queue" &&
                                       event.value("outcome", "") == "accepted";
                            }),
                "HOLD queued stage recorded");
        require(std::any_of(hold_events.begin(), hold_events.end(),
                            [](const auto& event) {
                                return event.value("stage", "") == "rpc_response" &&
                                       event.value("outcome", "") == "pending";
                            }),
                "HOLD pending RPC response stage recorded");
        fixture.reject_hold = true;
        context();
        require(backend.mode().stop_offboard(), "HOLD rejection test queued");
        wait([&] { return backend.mode().hold_result().has_value(); }, "HOLD reject observed");
        require(backend.mode().hold_result() == false, "remote HOLD rejection retained");
        const auto rejected_events = backend.take_hold_diagnostics();
        require(std::any_of(rejected_events.begin(), rejected_events.end(),
                            [](const auto& event) {
                                return event.value("stage", "") == "rpc_response" &&
                                       event.value("outcome", "") == "rejected" &&
                                       event.value("remote_reason", "") == "fixture_hold_denied";
                            }),
                "remote HOLD rejection reason recorded");
        fixture.reject_hold = false;
        backend.release();
        require(!backend.owns(), "explicit release removes authority");
        require(!backend.vehicle().send_until({}, Clock::now() + 150ms) &&
                    !backend.send_heading(.3, Clock::now() + 150ms, false),
                "released writer cannot send");
        // 2026-11-25 batch 6b: 실제 추종 제어 모양의 status (protobuf 는 키가 모두 있어야 보낸다)
        auto status = recorded_status();
        backend.publish_status(status);
        wait([&] { return fixture.status_count.load() == 1; }, "follow status publication");
        {
            const std::scoped_lock lock(fixture.mutex);
            require(fixture.last_status.at("schema") == "argus.follow.status.v1" &&
                        fixture.last_status_team_seq.load() == 1 &&
                        fixture.last_status.at("seq").get<std::uint64_t>() > 0 &&
                        fixture.last_status.at("status") == status &&
                        fixture.control_rejected.load() == 0,
                    "team protobuf follow/status round trip (team seq 1, pi seq, body)");
        }
        status["observation_valid_until_monotonic_ns"] = now_ns();
        backend.publish_status(status);
        std::this_thread::sleep_for(30ms);
        require(fixture.status_count == 1 && !backend.owns(),
                "timestamp-only status neither floods nor renews authority");
        const auto operator_context = backend.operator_context();
        require(operator_context.at("onboard_session") == "onboard-test" &&
                    operator_context.at("fc_epoch") == fixture.epoch.load() &&
                    operator_context.at("clock_domain") == "host_monotonic",
                "operator bootstrap exposes current onboard context");
        require(backend.validate_operator_request({{"action", "status"}}),
                "status is not authority");
        require(!backend.validate_operator_request({{"action", "select"}}),
                "old operator requests fail closed");
        require(backend.validate_operator_request(
                    {{"action", "select"},
                     {"expected_uid", uid},
                     {"fc_epoch", fixture.epoch.load()},
                     {"onboard_session", "onboard-test"},
                     {"controller_session", backend.controller_session()}}),
                "operator identity envelope");
        context();
        require(backend.acquire(), "guard fixture acquire");
        backend.set_writer_guard([&] {
            return backend.identity_permit() && backend.owns() &&
                   backend.pose().read().have_attitude;
        });
        require(backend.vehicle().send_until({}, Clock::now() + 150ms),
                "guard reentrant state reads do not deadlock");
        backend.set_writer_guard({});
        require(!backend.vehicle().send_until({}, Clock::now() + 150ms) && !backend.owns(),
                "cleared guard revokes output");
        backend.release();
        {
            std::lock_guard<std::mutex> lock(fixture.mutex);
            require(fixture.last_release.at("owner_token") == "fixture-owner-token",
                    "guard clear preserves token for remote release");
        }
        backend.set_writer_guard([] { return true; });
        auto read_only = config;
        read_only.read_only = true;
        ZeroMqFollowBackend observer(read_only);
        wait([&] { return observer.identity_permit(); }, "read-only state");
        auto before = fixture.rpc_count.load();
        require(!observer.acquire(), "read-only authority blocked");
        std::this_thread::sleep_for(30ms);
        require(fixture.rpc_count == before, "read-only no RPC");
        fixture.silent = true;
        wait([&] { return !backend.identity_permit() && !observer.identity_permit(); },
             "stale telemetry invalidates");
        require(!backend.pose().read().have_attitude, "stale pose unavailable");
        require(!backend.mode().stop_offboard(), "HOLD with stale telemetry denied");
        denied_hold = backend.take_hold_diagnostics();
        require(!denied_hold.empty() &&
                    denied_hold.back().at("outcome") == "authority_or_freshness_denied" &&
                    denied_hold.back().at("fc_fresh") == false,
                "stale telemetry denial recorded");
        fixture.silent = false;
        wait([&] { return backend.identity_permit(); }, "telemetry recovers without authority");
        require(!backend.owns(), "no stale autoresume");
        fixture.rpc_delay_ms = 150;
        auto began = Clock::now();
        require(!backend.acquire(), "late RPC cannot acquire");
        require(Clock::now() - began < 200ms, "bounded acquire timeout");
        std::this_thread::sleep_for(170ms);
        require(!backend.owns(), "late accepted response discarded");
        mode_event_test();
        hold_queue_full_diagnostic_test();
        telemetry_during_slow_rpc_test();
        early_mode_event_test();
        malformed_mode_event_frames_test();
        oversized_fc_frame_keeps_connection_test();
        command_results_limit_test();
        rpc_reply_shapes_test();
        std::cout << "B loopback transport passed: RPC correlation/capability, async ACK/events, "
                     "pose, units, TTL, RC/epoch/stale revoke, read-only\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
