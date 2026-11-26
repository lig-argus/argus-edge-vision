// Real inproc ZeroMQ REP/REQ, fake backend permit and handler; no FC/network devices.
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "argus/control/contracts/include/follow_b_request.hpp"
#include "argus/runtime/follow/include/follow_operator_gateway.hpp"
#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"

using namespace follow;
namespace {
std::int64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
        .count();
}

NativeJson command(const std::string& id, const std::string& action, std::uint64_t seq) {
    const auto now = now_ns();
    return {{"schema", "argus.follow.command.v1"},
            {"command_id", id},
            {"action", action},
            {"source", "operator-test"},
            {"seq", seq},
            {"expected_uid", "test-uid"},
            {"controller_session", "controller-1"},
            {"onboard_session", "onboard-1"},
            {"fc_epoch", 4},
            {"issued_monotonic_ns", now},
            {"valid_until_monotonic_ns", now + 900000000},
            {"sensor", "camera-original"},
            {"generation", 16777215},
            {"class_id", 2},
            {"label", "car"},
            {"track_id", 16777215}};
}

struct Handler final : IFollowCommand {
    unsigned calls{0};
    std::string last_action;
    NativeJson last_input;
    NativeJson override_reply;
    NativeJson execute(const NativeJson& input) override {
        ++calls;
        last_action = input.at("action");
        last_input = input;
        if (!override_reply.is_null()) {
            return override_reply;
        }
        if (last_action == "search") {
            ScanRequest scan;
            if (input.contains("yaw_range_rad")) {
                scan.yaw_range_rad = input.at("yaw_range_rad");
            }
            if (input.contains("yaw_rate_rad_s")) {
                scan.yaw_rate_rad_s = input.at("yaw_rate_rad_s");
            }
            if (input.contains("timeout_s")) {
                scan.timeout_s = input.at("timeout_s");
            }
            (void)parse_scan_request(scan);
        }
        return {
            {"accepted", true}, {"reason", "fake-handler"}, {"controller_session", "controller-1"}};
    }
};

// 2026-11-25 batch 8: B 게이트웨이는 팀 [follow/command][FollowCommand] 만 받는다 (Team).
// B 가 아닌 게이트웨이 (6363) 는 예전 msgpack 그대로다 (LegacyFollow, LegacySingle)
enum class Shape : std::uint8_t { Team, LegacyFollow, LegacySingle };

// 프레임을 보내고 gateway.poll 로 응답 메시지 하나를 받는다
std::vector<std::string> raw_exchange(VMyOperatorGateway& gateway, void* client, Handler& handler,
                                      const std::vector<std::string>& frames) {
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const int flags = i + 1 < frames.size() ? ZMQ_SNDMORE : 0;
        assert(zmq_send(client, frames[i].data(), frames[i].size(), flags) >= 0);
    }
    zmq_pollitem_t readable{client, 0, ZMQ_POLLIN, 0};
    bool ready = false;
    for (int i = 0; i < 1000; ++i) {
        gateway.poll(handler);
        if (zmq_poll(&readable, 1, 1) > 0) {
            ready = true;
            break;
        }
    }
    assert(ready);
    std::vector<std::string> received;
    bool more = true;
    while (more) {
        std::string bytes, error;
        zmq_receive_part(client, &bytes, 1 << 20, more, error, "test init", "test recv",
                         "test size");
        assert(error.empty());
        received.push_back(bytes);
    }
    return received;
}

// 팀 FollowCommand 본문으로 바꿀 수 있는지 (protobuf 에 실을 수 없는 값은 보내는 쪽이 거부한다)
bool team_body(const NativeJson& input, std::string& body) {
    argus::team_wire::Bytes bytes;
    std::vector<std::string> unknown;
    std::string error;
    if (!argus::team_wire::encode_body(argus::team_wire::kFollowCommand, false, input, 0, bytes,
                                       unknown, error)) {
        return false;
    }
    body.assign(bytes.begin(), bytes.end());
    return true;
}

NativeJson exchange(VMyOperatorGateway& gateway, void* client, Handler& handler,
                    const NativeJson& input, Shape shape = Shape::Team) {
    if (shape == Shape::Team) {
        std::string body;
        assert(team_body(input, body));
        const auto reply = raw_exchange(gateway, client, handler, {"follow/command", body});
        assert(reply.size() == 2 && reply[0] == "follow/command" && !reply[1].empty());
        NativeJson value;
        std::uint64_t sequence = 0;
        std::string error;
        const argus::team_wire::Bytes bytes(reply[1].begin(), reply[1].end());
        assert(argus::team_wire::decode_body(argus::team_wire::kFollowCommand, true, bytes, value,
                                             sequence, error));
        return value;
    }
    const auto packed = NativeJson::to_msgpack(input);
    const std::string body(packed.begin(), packed.end());
    if (shape == Shape::LegacyFollow) {
        const auto reply = raw_exchange(gateway, client, handler, {"follow/command", body});
        assert(reply.size() == 2 && reply[0] == "follow/result");
        return NativeJson::from_msgpack(reply[1].begin(), reply[1].end());
    }
    const auto reply = raw_exchange(gateway, client, handler, {body});
    assert(reply.size() == 1);
    return NativeJson::from_msgpack(reply[0].begin(), reply[0].end());
}

// B 게이트웨이가 해석하지 못한 요청: [follow/command][빈 본문] 으로 답하고 handler 를 부르지 않는다
void expect_empty_reply(VMyOperatorGateway& gateway, void* client, Handler& handler,
                        const std::vector<std::string>& frames) {
    const auto calls = handler.calls;
    const auto reply = raw_exchange(gateway, client, handler, frames);
    assert(reply.size() == 2 && reply[0] == "follow/command" && reply[1].empty());
    assert(handler.calls == calls);
}

void run() {
    ZeroMqContext context;
    bool permit = true;
    auto checker = [&](const NativeJson& request) {
        return permit && request.value("controller_session", "") == "controller-1" &&
               request.value("onboard_session", "") == "onboard-1" && request.at("fc_epoch") == 4;
    };
    VMyOperatorGateway gateway{
        context, "inproc://b-command-test", true, checker, {{1, "camera-original"}}};
    auto client = zmq_open_socket(context, ZMQ_REQ, "test request");
    const int linger = 0;
    zmq_set_option(client.get(), ZMQ_LINGER, &linger, sizeof(linger), "test linger");
    assert(zmq_connect(client.get(), "inproc://b-command-test") == 0);
    Handler handler;
    auto status = NativeJson{{"schema", "argus.follow.command.v1"},
                             {"command_id", "bootstrap"},
                             {"action", "STATUS"},
                             {"expected_uid", "test-uid"}};
    auto result = exchange(gateway, client.get(), handler, status);
    assert(result.at("schema") == "argus.follow.result.v1" && result.at("accepted") == true);
    assert(result.at("controller_session") == "controller-1");
    status["controller_session"] = "";
    result = exchange(gateway, client.get(), handler, status);
    assert(result.at("controller_session") == "controller-1");
    assert(exchange(gateway, client.get(), handler, status).at("accepted") == true &&
           handler.calls == 3);

    auto first = command("start", "START", 1);
    auto oversized = command("oversized-generation", "START", 1);
    oversized["generation"] = 16777216;
    assert(exchange(gateway, client.get(), handler, oversized).at("accepted") == false);
    oversized = command("oversized-class", "START", 1);
    oversized["class_id"] = 16777216;
    assert(exchange(gateway, client.get(), handler, oversized).at("accepted") == false);
    oversized = command("unknown-sensor", "START", 1);
    oversized["sensor"] = 0;
    assert(exchange(gateway, client.get(), handler, oversized).at("accepted") == false);
    oversized = command("oversized-track", "START", 1);
    oversized["track_id"] = 16777216;
    assert(exchange(gateway, client.get(), handler, oversized).at("accepted") == false);
    result = exchange(gateway, client.get(), handler, first);
    assert(result.at("accepted") == true && handler.last_action == "track_start");
    assert(result.at("command_id") == "start" && result.at("fc_epoch") == 4);
    const auto after_start = handler.calls;
    assert(exchange(gateway, client.get(), handler, first).at("accepted") == true &&
           handler.calls == after_start);
    auto refreshed = first;
    refreshed["seq"] = 2;
    refreshed["issued_monotonic_ns"] = now_ns();
    refreshed["valid_until_monotonic_ns"] = now_ns() + 900000000;
    assert(exchange(gateway, client.get(), handler, refreshed).at("accepted") == true &&
           handler.calls == after_start);
    permit = false;
    assert(exchange(gateway, client.get(), handler, first).at("accepted") == false &&
           handler.calls == after_start);
    permit = true;
    auto conflict = first;
    conflict["label"] = "truck";
    assert(exchange(gateway, client.get(), handler, conflict).at("accepted") == false);
    auto stop = command("stop", "STOP", 3);
    assert(exchange(gateway, client.get(), handler, stop).at("accepted") == true &&
           handler.last_action == "stop");
    const auto after_stop = handler.calls;
    assert(exchange(gateway, client.get(), handler, first).at("accepted") == false &&
           handler.calls == after_stop);
    refreshed["seq"] = 4;
    assert(exchange(gateway, client.get(), handler, refreshed).at("accepted") == false &&
           handler.calls == after_stop);

    // 2026-11-25 batch 8: null label 과 실수 seq 는 protobuf (string, uint64) 에 실을 수 없어
    // 보내는 쪽 변환이 거부한다 (예전에는 추종 제어가 받아 거부했다)
    std::string unused;
    auto bad_label = command("bad-label", "START", 4);
    bad_label["label"] = nullptr;
    assert(!team_body(bad_label, unused));
    auto expired = command("expired", "START", 3);
    expired["issued_monotonic_ns"] = now_ns() - 2000000000;
    expired["valid_until_monotonic_ns"] = now_ns() - 1000000000;
    assert(exchange(gateway, client.get(), handler, expired).at("accepted") == false);
    auto future = command("future", "START", 3);
    future["issued_monotonic_ns"] = now_ns() + 1000000000;
    future["valid_until_monotonic_ns"] = now_ns() + 1500000000;
    assert(exchange(gateway, client.get(), handler, future).at("accepted") == false);
    auto long_ttl = command("long-ttl", "START", 3);
    long_ttl["valid_until_monotonic_ns"] =
        long_ttl.at("issued_monotonic_ns").get<std::int64_t>() + 1000000001;
    assert(exchange(gateway, client.get(), handler, long_ttl).at("accepted") == false);
    auto wrong_session = command("wrong-controller", "START", 3);
    wrong_session["controller_session"] = "old";
    assert(exchange(gateway, client.get(), handler, wrong_session).at("accepted") == false);
    auto fractional = command("float-seq", "START", 3);
    fractional["seq"] = 3.0;
    assert(!team_body(fractional, unused));
    auto scan = command("scan", "SCAN_START", 4);
    scan["yaw_range_rad"] = std::numeric_limits<double>::quiet_NaN();
    assert(exchange(gateway, client.get(), handler, scan).at("accepted") == true &&
           handler.last_action == "search");

    // 2026-11-25 batch 8: B 는 예전 한 프레임 argus.vmy.request.v1 과 [follow/command][msgpack] 을
    // 받지 않는다. 해석하지 못한 요청과 같이 [follow/command][빈 본문] 으로 답하고 (REP 상태 유지)
    // handler 를 부르지 않는다
    NativeJson legacy{{"schema", "argus.vmy.request.v1"},
                      {"request_id", "legacy-nan"},
                      {"action", "stop"},
                      {"yaw_range_rad", std::numeric_limits<double>::quiet_NaN()},
                      {"controller_session", "controller-1"},
                      {"onboard_session", "onboard-1"},
                      {"fc_epoch", 4},
                      {"source", "operator-test"},
                      {"seq", 5},
                      {"issued_monotonic_ns", now_ns()},
                      {"valid_until_monotonic_ns", now_ns() + 900000000}};
    const auto legacy_packed = NativeJson::to_msgpack(legacy);
    const std::string legacy_body(legacy_packed.begin(), legacy_packed.end());
    expect_empty_reply(gateway, client.get(), handler, {legacy_body});
    const auto old_packed = NativeJson::to_msgpack(command("old-wire", "STOP", 5));
    expect_empty_reply(gateway, client.get(), handler,
                       {"follow/command", std::string(old_packed.begin(), old_packed.end())});
    std::string valid_body;
    assert(team_body(command("frame-shape", "STOP", 5), valid_body));
    expect_empty_reply(gateway, client.get(), handler, {"follow/command", valid_body, "extra"});
    expect_empty_reply(gateway, client.get(), handler, {"follow/result", valid_body});
    expect_empty_reply(gateway, client.get(), handler, {"follow/command", ""});
    expect_empty_reply(gateway, client.get(), handler, {"follow/command", "\xff\xff\xff"});
    expect_empty_reply(gateway, client.get(), handler,
                       {"follow/command", std::string((256 << 10) + 1, '\x0a')});
    // 거부 뒤에도 REP 는 다음 요청에 답한다 (거부한 요청은 seq 를 쓰지 않았다)
    const auto legacy_calls = handler.calls;
    assert(exchange(gateway, client.get(), handler, command("after-reject", "STOP", 5))
               .at("accepted") == true);
    assert(handler.calls == legacy_calls + 1 && handler.last_action == "stop");

    auto mapped = command("mapped", "START", 6);
    mapped["sensor"] = 1;
    mapped.erase("label");
    assert(exchange(gateway, client.get(), handler, mapped).at("accepted") == true);
    assert(handler.last_input.at("sensor") == "camera-original" &&
           !handler.last_input.contains("label"));
    mapped["command_id"] = "unmapped";
    mapped["sensor"] = 8;
    mapped["seq"] = 7;
    assert(exchange(gateway, client.get(), handler, mapped).at("accepted") == false);
    auto v2 = command("v2", "STOP", 8);
    v2["schema"] = "argus.follow.command.v2";
    v2["session_id"] = "onboard-1";
    v2.erase("onboard_session");
    v2["future_optional"] = 42;
    result = exchange(gateway, client.get(), handler, v2);
    assert(result.at("schema") == "argus.follow.result.v2" && result.at("result") == "ACCEPTED");
    assert(result.at("outcome") == "SUCCEEDED" && result.at("request_accepted") == true);
    assert(!handler.last_input.contains("future_optional") &&
           handler.last_input.at("onboard_session") == "onboard-1");
    v2["future_optional"] = 43;
    v2["seq"] = 9;
    assert(exchange(gateway, client.get(), handler, v2).at("result") == "ACCEPTED");
    v2["command_id"] = "bad-session-alias";
    v2["onboard_session"] = "different";
    result = exchange(gateway, client.get(), handler, v2);
    assert(result.at("result") == "REJECTED");
    assert(result.at("outcome") == "REJECTED" && result.at("request_accepted") == false);

    // Both versions preserve admission across a cached async failure; no re-execution.
    for (int version : {1, 2}) {
        const auto id = "async-failure-" + std::to_string(version);
        auto async = command(id, "START", 20 + version);
        if (version == 2) {
            async["schema"] = "argus.follow.command.v2";
            async["session_id"] = "onboard-1";
        }
        handler.override_reply = {{"accepted", true},
                                  {"pending", true},
                                  {"execution_complete", false},
                                  {"reason", "waiting"}};
        auto admitted = exchange(gateway, client.get(), handler, async);
        assert(admitted.at("outcome") == "IN_PROGRESS" && admitted.at("request_accepted") == true);
        handler.override_reply = {
            {"command_results", NativeJson::array({{{"command_id", id},
                                                    {"pending", false},
                                                    {"accepted", false},
                                                    {"reason", "execution_interrupted_MANUAL"}}})}};
        const auto count = handler.calls;
        auto failed = exchange(gateway, client.get(), handler, async);
        assert(handler.calls == count + 1 && handler.last_action == "status");
        assert(failed.at("outcome") == "FAILED" && failed.at("request_accepted") == true);
        assert(failed.at("result") == (version == 2 ? "FAILED" : "failed"));
        auto again = exchange(gateway, client.get(), handler, async);
        assert(handler.calls == count + 1 && again.at("outcome") == "FAILED");
    }
    handler.override_reply = nullptr;

    VMyOperatorGateway disabled{context, "inproc://disabled-command-test"};
    auto disabled_client = zmq_open_socket(context, ZMQ_REQ, "disabled test");
    zmq_set_option(disabled_client.get(), ZMQ_LINGER, &linger, sizeof(linger), "disabled linger");
    assert(zmq_connect(disabled_client.get(), "inproc://disabled-command-test") == 0);
    // B 가 아닌 게이트웨이 (6363): 예전 msgpack 그대로. follow/command 는 거부, 한 프레임은 받는다
    assert(exchange(disabled, disabled_client.get(), handler, status, Shape::LegacyFollow)
               .at("accepted") == false);
    NativeJson native{
        {"schema", "argus.vmy.request.v1"}, {"request_id", "native-status"}, {"action", "status"}};
    const auto native_calls = handler.calls;
    const auto native_reply =
        exchange(disabled, disabled_client.get(), handler, native, Shape::LegacySingle);
    assert(native_reply.at("schema") == "argus.vmy.result.v1" &&
           native_reply.at("accepted") == true);
    assert(handler.calls == native_calls + 1);
}

// ---- 왕복 시간 측정 (WSL 합성, 장비 없음) ----
// 운용자 요청 6373 의 STATUS (운용 상태가 펼쳐진 큰 응답) 와 START (수락) 를 loopback tcp 로
// count 번씩 보내고, 보낸 때부터 응답을 해석할 때까지를 잰다. 응답자는 별도 스레드에서
// gateway.poll 을 20 us 간격으로 부른다 (추종 제어의 주기 tick 대신)
std::string free_tcp_endpoint() {
    void* probe_context = zmq_ctx_new();
    void* probe = zmq_socket(probe_context, ZMQ_REP);
    assert(zmq_bind(probe, "tcp://127.0.0.1:*") == 0);
    std::array<char, 256> endpoint{};
    std::size_t size = endpoint.size();
    assert(zmq_getsockopt(probe, ZMQ_LAST_ENDPOINT, endpoint.data(), &size) == 0);
    zmq_close(probe);
    zmq_ctx_term(probe_context);
    return endpoint.data();
}

NativeJson measure_status_reply() {
    NativeJson candidates = NativeJson::array();
    for (int i = 1; i <= 4; ++i) {
        candidates.push_back({{"generation", 7},
                              {"sensor", "camera-original"},
                              {"track_id", i},
                              {"class_id", 2},
                              {"label", "car"}});
    }
    return {{"accepted", true},
            {"reason", ""},
            {"controller_session", "controller-1"},
            {"onboard_session", "onboard-1"},
            {"fc_epoch", 4},
            {"observation_ready", true},
            {"follow_environment", "outdoor"},
            {"follow_controller", "predictive"},
            {"configured_follow_environment", "outdoor"},
            {"configured_follow_controller", "predictive"},
            {"profile_replace_ready", true},
            {"selection_revision", 3},
            {"tracking_only", false},
            {"expected_uid", "test-uid"},
            {"identity_ok", true},
            {"connected_uid", "test-uid"},
            {"connected", true},
            {"armed", false},
            {"in_air", false},
            {"ground", true},
            {"ground_known", true},
            {"landed", true},
            {"actual_hold", false},
            {"active", false},
            {"mode", "yaw"},
            {"tracks_fresh", true},
            {"offboard_hover", false},
            {"input_contract_v5", true},
            {"alignment_verified", true},
            {"selected", nullptr},
            {"controls",
             {{"mode", true},
              {"select", true},
              {"start", false},
              {"stop", false},
              {"search", false},
              {"reset_input", true}}},
            {"reasons", {{"start", "not_in_air"}, {"mode", ""}}},
            {"scan_progress_deg", 0.0},
            {"heading_stop_pending", false},
            {"candidates", candidates},
            {"target_phase", "idle"},
            {"scan_state", 0},
            {"scan_state_name", "IDLE"},
            {"scan_remaining_s", nullptr},
            {"scan_yaw_done_rad", nullptr},
            {"mission_state", "IDLE"},
            {"target_state", "NONE"},
            {"prediction_valid_until_monotonic_ns", nullptr},
            {"observation_valid_until_monotonic_ns", nullptr},
            {"mission_state_code", 0},
            {"target_state_code", 0},
            {"recovery_reason", ""},
            {"ambiguous", false},
            {"command_results", NativeJson::array()}};
}

// 요청 하나를 보내고 응답 본문을 해석한다. 2026-11-25 batch 8: 팀 [follow/command][FollowCommand]
// -> [follow/command][FollowResult] (변환 포함). 예전 측정은 같은 함수의 msgpack 판
// ([follow/command][msgpack] -> [follow/result][msgpack]) 으로 했다 (proto/README.md batch 8)
NativeJson measure_round_trip(void* client, const NativeJson& request) {
    std::string body;
    assert(team_body(request, body));
    assert(zmq_send(client, "follow/command", 14, ZMQ_SNDMORE) == 14);
    assert(zmq_send(client, body.data(), body.size(), 0) >= 0);
    std::string bytes, error;
    bool more = false;
    zmq_receive_part(client, &bytes, 1 << 20, more, error, "m init", "m recv", "m size");
    assert(error.empty() && bytes == "follow/command" && more);
    zmq_receive_part(client, &bytes, 1 << 20, more, error, "m init", "m recv", "m size");
    assert(error.empty() && !more);
    NativeJson value;
    std::uint64_t sequence = 0;
    const argus::team_wire::Bytes reply(bytes.begin(), bytes.end());
    assert(argus::team_wire::decode_body(argus::team_wire::kFollowCommand, true, reply, value,
                                         sequence, error));
    return value;
}

void measure_rpc(int count) {
    ZeroMqContext context;
    const auto endpoint = free_tcp_endpoint();
    VMyOperatorGateway gateway{
        context, endpoint, true, [](const NativeJson&) { return true; }, {{1, "camera-original"}}};
    struct Reply final : IFollowCommand {
        NativeJson status = measure_status_reply();
        NativeJson execute(const NativeJson& input) override {
            if (input.at("action") == "status") {
                return status;
            }
            return {{"accepted", true}, {"reason", ""}, {"controller_session", "controller-1"}};
        }
    } handler;
    std::atomic<bool> stop{false};
    std::thread server([&] {
        while (!stop.load()) {
            gateway.poll(handler);
            std::this_thread::sleep_for(std::chrono::microseconds(20));
        }
    });
    auto client = zmq_open_socket(context, ZMQ_REQ, "measure request");
    const int linger = 0;
    zmq_set_option(client.get(), ZMQ_LINGER, &linger, sizeof(linger), "measure linger");
    assert(zmq_connect(client.get(), endpoint.c_str()) == 0);
    std::vector<double> status_ms, start_ms;
    int failures = 0;
    for (int i = 0; i < count; ++i) {
        const NativeJson status{{"schema", "argus.follow.command.v1"},
                                {"command_id", "status-" + std::to_string(i)},
                                {"action", "STATUS"},
                                {"expected_uid", "test-uid"}};
        auto begin = now_ns();
        if (!measure_round_trip(client.get(), status).value("accepted", false)) {
            ++failures;
        }
        status_ms.push_back(static_cast<double>(now_ns() - begin) / 1e6);
        const auto start =
            command("start-" + std::to_string(i), "START", static_cast<std::uint64_t>(i) + 1);
        begin = now_ns();
        if (!measure_round_trip(client.get(), start).value("accepted", false)) {
            ++failures;
        }
        start_ms.push_back(static_cast<double>(now_ns() - begin) / 1e6);
    }
    stop = true;
    server.join();
    auto report = [](const char* label, std::vector<double> values) {
        std::sort(values.begin(), values.end());
        double sum = 0;
        for (const double value : values) {
            sum += value;
        }
        const auto at = [&](double q) {
            return values[static_cast<std::size_t>(q * static_cast<double>(values.size() - 1))];
        };
        std::printf("%s n=%zu mean=%.3f p50=%.3f p90=%.3f p99=%.3f max=%.3f ms\n", label,
                    values.size(), sum / static_cast<double>(values.size()), at(.5), at(.9),
                    at(.99), values.back());
    };
    std::printf("CMD_MEASURE count=%d failures=%d\n", count, failures);
    if (count > 0) {
        report("CMD_STATUS", status_ms);
        report("CMD_START", start_ms);
    }
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "measure_rpc") {
            measure_rpc(std::stoi(argv[2]));
            return 0;
        }
        run();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "예외: %s\n", error.what());
        return 1;
    }
    return 0;
}
