// 게이트웨이 6371 요청-응답 시험 (2026-11-25 batch 7, 장비 없음, 합성 입력).
// - ZmqFollowTransport::rpc_receive 가 추종 제어의 팀 [control/mode_request][ControlModeRequest] 를
//   [control/mode_request][msgpack] 으로 되살리고 (제어권 RPC 와 모드 요청의 값이 키, 정수와 실수
//   구분까지 같다), rpc_send 가 응답 msgpack 을 [control/mode_request][ControlModeReply] 로 보낸다
//   (응답 셋 모양: RPC 응답, 모드 결과, 예외 응답).
// - 해석하지 못한 요청 (예전 한 프레임 msgpack, 예전 [control/mode_request][msgpack], 다른 이름,
//   빈 본문, 깨진 본문, 256 KiB 초과, 세 프레임) 은 FollowBOwner 에 넘기지 않고
//   [control/mode_request][빈 본문] 으로 답하며 센다. 그 뒤 같은 REP 로 정상 요청을 받는다.
// - 변환하지 못한 응답 (schema 없음) 은 빈 본문으로 보내고 센다.
// - 실제 FollowBOwner 가 ZmqFollowTransport 로 받은 제어권 RPC 와 모드 요청에 팀 모양으로 답한다.
// 사용: GatewayModeRpcProtobufTest
#include <unistd.h>
#include <zmq.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/follow_owner.hpp"
#include "argus/gateway/follow/include/zmq_transport.hpp"
#include "tests/gateway/gateway_control_fixture.hpp"
#include "tests/gateway/gateway_fc_fixture.hpp"

namespace gw = argus::gateway;
namespace fx = gateway_fc_fixture;
namespace cx = gateway_control_fixture;
using namespace std::chrono_literals;

namespace {

const char* const kUid = "000600000000353733323333511700280027";
const char* const kName = "control/mode_request";

int& failures() {
    static int count = 0;
    return count;
}

void check(bool condition, const std::string& what) {
    if (!condition) {
        ++failures();
        std::cerr << "실패: " << what << '\n';
    }
}

// 20000~29999 에서 고른다 (stage4, FC, control 시험과 다른 자리)
std::string endpoint(int offset) {
    return "tcp://127.0.0.1:" +
           std::to_string(20000 + (((::getpid() * 7) + 3000 + offset) % 10000));
}

gw::Bytes text(const std::string& value) { return {value.begin(), value.end()}; }

void send_parts(void* socket, const gw::Parts& parts) {
    for (std::size_t i = 0; i < parts.size(); ++i) {
        zmq_send(socket, parts.at(i).data(), parts.at(i).size(),
                 i + 1 < parts.size() ? ZMQ_SNDMORE : 0);
    }
}

gw::Parts receive_parts(void* socket, int timeout_ms) {
    zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
    gw::Parts parts;
    for (;;) {
        zmq_msg_t message;
        zmq_msg_init(&message);
        if (zmq_msg_recv(&message, socket, 0) < 0) {
            zmq_msg_close(&message);
            return {};
        }
        const auto* data = static_cast<const std::uint8_t*>(zmq_msg_data(&message));
        parts.emplace_back(data,
                           std::next(data, static_cast<std::ptrdiff_t>(zmq_msg_size(&message))));
        const bool more = zmq_msg_more(&message) != 0;
        zmq_msg_close(&message);
        if (!more) {
            return parts;
        }
    }
}

// 응답 본문을 해석한 값 (해석 실패면 nullopt)
std::optional<gw::Wire> reply_value(const gw::Parts& reply) {
    if (reply.size() != 2 || reply.front() != text(kName)) {
        return std::nullopt;
    }
    gw::Bytes packed;
    std::string error;
    if (!cx::decode_mode_reply(reply.back(), packed, error)) {
        return std::nullopt;
    }
    return gw::msgpack_unpack(packed);
}

// 6371 REP 를 가진 전송 하나와 그 엔드포인트의 REQ
struct Rig {
    void* context{zmq_ctx_new()};
    void* request{nullptr};
    gw::FollowEndpoints endpoints;
    std::unique_ptr<gw::ZmqFollowTransport> transport;

    explicit Rig(int offset)
        : endpoints{endpoint(offset), endpoint(offset + 1), endpoint(offset + 2), std::nullopt} {
        transport = std::make_unique<gw::ZmqFollowTransport>(endpoints);
        request = zmq_socket(context, ZMQ_REQ);
        const int linger = 0;
        zmq_setsockopt(request, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_connect(request, endpoints.rpc.c_str());
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;
    Rig(Rig&&) = delete;
    Rig& operator=(Rig&&) = delete;
    ~Rig() {
        transport.reset();
        zmq_close(request);
        zmq_ctx_term(context);
    }
    // 요청 하나를 보내고 전송이 FollowBOwner 에 넘긴 값을 받는다 (거부면 nullopt, 전송이 이미 답함)
    std::optional<gw::Parts> deliver(const gw::Parts& parts) {
        send_parts(request, parts);
        for (int i = 0; i < 400; ++i) {
            if (auto got = transport->rpc_receive()) {
                return got;
            }
            if (transport->rpc_decode_failures() != decode_seen) {
                decode_seen = transport->rpc_decode_failures();
                return std::nullopt;
            }
            std::this_thread::sleep_for(5ms);
        }
        return std::nullopt;
    }
    std::uint64_t decode_seen{0};
};

void transport_round_trip() {
    Rig rig(0);
    // 제어권 RPC (acquire) 와 모드 요청 (HOLD): 값 그대로 되살린다
    for (const auto& [action, mode] :
         {std::pair<const char*, bool>{"acquire", false},
          std::pair<const char*, bool>{"revoke", false}, std::pair<const char*, bool>{"hold", true},
          std::pair<const char*, bool>{"heading", true}}) {
        const gw::Bytes packed = cx::mode_rpc_request(action, mode, 7, "onboard");
        const gw::Bytes body = cx::encode_mode_request(packed);
        check(!body.empty(), std::string("요청 본문 만들기 ") + action);
        const auto got = rig.deliver({text(kName), body});
        check(got && got->size() == 2 && got->front() == text(kName),
              std::string("요청이 [control/mode_request][msgpack] 으로 넘어온다 ") + action);
        if (!got || got->size() != 2) {
            continue;
        }
        const std::string difference = fx::msgpack_difference(packed, got->back());
        check(difference.empty(), std::string("요청 값 왕복 ") + action + ": " + difference);
        // 응답 셋 모양
        gw::Wire reply = gw::Wire::object();
        if (mode) {
            for (const auto& [key, value] : std::initializer_list<std::pair<const char*, gw::Wire>>{
                     {"schema", "argus.control.mode_result.v1"},
                     {"source", "argus-onboard"},
                     {"command_id", "a1b2c3d4e5f60718-7"},
                     {"session_id", "onboard"},
                     {"requester_session_id", "a1b2c3d4e5f60718"},
                     {"client_session", "a1b2c3d4e5f60718"},
                     {"request_id", "a1b2c3d4e5f60718-7"},
                     {"onboard_session", "onboard"},
                     {"uid", kUid},
                     {"fc_epoch", 0},
                     {"result", "ACCEPTED"},
                     {"reason", "request_admitted"},
                     {"seq", 3},
                     {"clock_domain", "host_monotonic"},
                     {"published_monotonic_ns", 1500}}) {
                gw::wire_set(reply, key, value);
            }
        } else {
            for (const auto& [key, value] : std::initializer_list<std::pair<const char*, gw::Wire>>{
                     {"request_id", "a1b2c3d4e5f60718-7"},
                     {"client_session", "a1b2c3d4e5f60718"},
                     {"schema", "argus.follow.rpc.v1"},
                     {"onboard_session", "onboard"},
                     {"fc_epoch", 0},
                     {"status", "accepted"},
                     {"owner_token", "tok"},
                     {"capabilities", gw::Wire::array({"context_v1", "absolute_heading_rad"})}}) {
                gw::wire_set(reply, key, value);
            }
        }
        const gw::Bytes reply_packed = gw::msgpack_pack(reply);
        rig.transport->rpc_send({text(kName), reply_packed});
        const gw::Parts answer = receive_parts(rig.request, 2000);
        check(answer.size() == 2 && answer.front() == text(kName),
              std::string("응답 첫 프레임은 요청 이름 ") + action);
        gw::Bytes decoded;
        std::string error;
        check(answer.size() == 2 && cx::decode_mode_reply(answer.back(), decoded, error) &&
                  fx::msgpack_difference(reply_packed, decoded).empty(),
              std::string("응답 값 왕복 ") + action + " " + error);
    }
    // 예외 응답 (schema, status, reason 만): 팀 id 가 없는 본문으로 나간다
    const auto got = rig.deliver({text(kName), cx::encode_mode_request(cx::mode_rpc_request(
                                                   "acquire", false, 8, "onboard"))});
    check(got.has_value(), "예외 응답 시험 요청");
    const gw::Wire failure{
        {"schema", "argus.follow.rpc.v1"}, {"status", "failed"}, {"reason", "rpc_multipart_shape"}};
    rig.transport->rpc_send({text(kName), gw::msgpack_pack(failure)});
    const auto value = reply_value(receive_parts(rig.request, 2000));
    check(value && value->at("status") == "failed" &&
              value->at("reason") == "rpc_multipart_shape" && !value->contains("fc_epoch"),
          "예외 응답은 그대로 (id 없음)");
    check(rig.transport->rpc_decode_failures() == 0 && rig.transport->rpc_encode_failures() == 0,
          "정상 요청과 응답은 실패로 세지 않는다");
}

void rejected_requests() {
    Rig rig(10);
    const gw::Bytes packed = cx::mode_rpc_request("acquire", false, 1, "onboard");
    const gw::Bytes body = cx::encode_mode_request(packed);
    const std::vector<std::pair<std::string, gw::Parts>> cases{
        {"예전 한 프레임 msgpack", {packed}},
        {"예전 [control/mode_request][msgpack]", {text(kName), packed}},
        {"예전 응답 이름", {text("control/mode_result"), body}},
        {"접두만 같은 이름", {text("control/mode_request_x"), body}},
        {"빈 본문", {text(kName), gw::Bytes{}}},
        {"깨진 본문", {text(kName), gw::Bytes{0xff, 0xff, 0xff}}},
        {"256 KiB 초과", {text(kName), gw::Bytes((std::size_t{256} << 10U) + 1, 0)}},
        {"세 프레임", {text(kName), body, text("x")}},
    };
    std::uint64_t expected = 0;
    for (const auto& [label, parts] : cases) {
        const auto got = rig.deliver(parts);
        ++expected;
        check(!got.has_value(), "FollowBOwner 에 넘기지 않는다: " + label);
        const gw::Parts answer = receive_parts(rig.request, 2000);
        check(answer.size() == 2 && answer.front() == text(kName) && answer.back().empty(),
              "[control/mode_request][빈 본문] 으로 답한다: " + label);
        check(rig.transport->rpc_decode_failures() == expected, "해석 실패로 센다: " + label);
    }
    // 같은 REP 로 정상 요청을 받는다 (REP 상태를 지켰다)
    const auto ok = rig.deliver({text(kName), body});
    check(ok && ok->size() == 2 && fx::msgpack_difference(packed, ok->back()).empty(),
          "거부 뒤 정상 요청");
    // 변환하지 못하는 응답 (schema 없음): 빈 본문으로 보내고 센다
    rig.transport->rpc_send({text(kName), gw::msgpack_pack(gw::Wire{{"status", "accepted"}})});
    const gw::Parts answer = receive_parts(rig.request, 2000);
    check(answer.size() == 2 && answer.front() == text(kName) && answer.back().empty(),
          "변환하지 못한 응답은 빈 본문");
    check(rig.transport->rpc_encode_failures() == 1, "응답 변환 실패를 센다");
    // 그 뒤에도 같은 REP 로 요청과 응답
    const auto again = rig.deliver({text(kName), body});
    check(again.has_value(), "응답 변환 실패 뒤 정상 요청");
    rig.transport->rpc_send(
        {text(kName),
         gw::msgpack_pack(gw::Wire{{"schema", "argus.follow.rpc.v1"}, {"status", "failed"}})});
    check(reply_value(receive_parts(rig.request, 2000)).has_value(), "응답 변환 실패 뒤 정상 응답");
}

// 실제 FollowBOwner 가 ZmqFollowTransport 로 받은 요청에 팀 모양으로 답한다. FC 가 없어 제어권은
// 주지 않는다 (예전과 같은 판정). 응답의 이름, 요청 id, 모양만 본다
void owner_replies() {
    std::atomic<double> now{300};
    gw::FlightLinkOptions options;
    options.expected_uid = kUid;
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-session"; };
    gw::FlightLink link(std::move(options));
    const gw::FollowEndpoints endpoints{endpoint(20), endpoint(21), endpoint(22), std::nullopt};
    void* context = zmq_ctx_new();
    void* request = zmq_socket(context, ZMQ_REQ);
    const int linger = 0;
    zmq_setsockopt(request, ZMQ_LINGER, &linger, sizeof(linger));
    {
        auto owner = std::make_unique<gw::FollowBOwner>(
            link, std::make_unique<gw::ZmqFollowTransport>(endpoints), gw::FollowOwnerOptions{});
        gw::FollowBOwner& b = *owner;
        zmq_connect(request, endpoints.rpc.c_str());
        const auto exchange = [&](const gw::Parts& parts) {
            send_parts(request, parts);
            for (int i = 0; i < 400; ++i) {
                b.tick();
                gw::Parts got = receive_parts(request, 5);
                if (!got.empty()) {
                    return got;
                }
            }
            return gw::Parts{};
        };
        const auto rpc =
            reply_value(exchange({text(kName), cx::encode_mode_request(cx::mode_rpc_request(
                                                   "acquire", false, 1, "test-session"))}));
        check(rpc && rpc->at("schema") == "argus.follow.rpc.v1" &&
                  rpc->at("request_id") == "a1b2c3d4e5f60718-1" && rpc->contains("status") &&
                  rpc->contains("fc_epoch"),
              "FollowBOwner 의 제어권 RPC 응답 (팀 모양)");
        check(rpc && rpc->at("status") != "accepted", "FC 없이 제어권을 주지 않는다");
        const auto mode =
            reply_value(exchange({text(kName), cx::encode_mode_request(cx::mode_rpc_request(
                                                   "hold", true, 2, "test-session"))}));
        check(mode && mode->at("schema") == "argus.control.mode_result.v1" &&
                  mode->at("command_id") == "a1b2c3d4e5f60718-2" &&
                  mode->at("result") != "ACCEPTED",
              "FollowBOwner 의 모드 요청 응답 (팀 모양, 제어권 없어 거부)");
        const gw::Parts old = exchange({cx::mode_rpc_request("acquire", false, 3, "test-session")});
        check(old.size() == 2 && old.front() == text(kName) && old.back().empty(),
              "예전 한 프레임 msgpack 은 빈 본문 응답");
    }
    zmq_close(request);
    zmq_ctx_term(context);
}

}  // namespace

int main() {
    try {
        transport_round_trip();
        rejected_requests();
        owner_replies();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayModeRpcProtobufTest 통과\n";
    return 0;
}
