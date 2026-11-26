// 게이트웨이 4단계 단위 시험: 실제 libzmq 루프백으로 추종 송신부 전송을 확인한다.
// REQ→REP 요청·응답, PUB 발행 수신, SUB 주제 필터, 추적 1MB 제한, 추적 주소 없을 때 소켓 없음.
// Python 원본과의 대량 비교는 tests/gateway/differential/stage4a_diff.py, stage4b_diff.py가 맡는다.
#include <unistd.h>
#include <zmq.h>

#include <chrono>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/zmq_transport.hpp"
#include "tests/gateway/gateway_control_fixture.hpp"
#include "tests/gateway/gateway_fc_fixture.hpp"

namespace gw = argus::gateway;
namespace cx = gateway_control_fixture;
namespace fx = gateway_fc_fixture;

namespace {

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

// Linux 임시 포트 범위(32768~)와 겹치지 않는 20000~29999에서 고른다.
std::string endpoint(int offset) {
    return "tcp://127.0.0.1:" + std::to_string(20000 + (((::getpid() * 7) + offset) % 10000));
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

template <typename F>
std::optional<gw::Parts> wait_for(F&& poll) {  // NOLINT(cppcoreguidelines-missing-std-forward)
    for (int i = 0; i < 200; ++i) {
        if (auto parts = poll()) {
            return parts;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::nullopt;
}

void loopback() {
    void* context = zmq_ctx_new();
    const int linger = 0;
    void* command_pub = zmq_socket(context, ZMQ_PUB);
    void* track_pub = zmq_socket(context, ZMQ_PUB);
    zmq_setsockopt(command_pub, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(track_pub, ZMQ_LINGER, &linger, sizeof(linger));
    const gw::FollowEndpoints endpoints{endpoint(1), endpoint(2), endpoint(3), endpoint(4)};
    zmq_bind(command_pub, endpoints.command.c_str());
    zmq_bind(track_pub, endpoints.tracks->c_str());
    {
        gw::ZmqFollowTransport transport(endpoints);
        check(transport.has_tracks(), "cv_gap 추적 소켓");
        void* state_sub = zmq_socket(context, ZMQ_SUB);
        zmq_setsockopt(state_sub, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_setsockopt(state_sub, ZMQ_SUBSCRIBE, "", 0);
        zmq_connect(state_sub, endpoints.state.c_str());
        void* request = zmq_socket(context, ZMQ_REQ);
        zmq_setsockopt(request, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_connect(request, endpoints.rpc.c_str());

        // REQ → REP. 2026-11-25 batch 7: 6371 은 팀 [control/mode_request][protobuf]. 전송이
        // [control/mode_request][msgpack] 으로 넘기고 응답 msgpack 을 protobuf 로 바꾼다 (값 대조와
        // 거부는 GatewayModeRpcProtobufTest)
        const gw::Bytes payload = cx::mode_rpc_request("acquire", false, 1, "onboard");
        send_parts(request, {text("control/mode_request"), cx::encode_mode_request(payload)});
        const auto incoming = wait_for([&] { return transport.rpc_receive(); });
        check(incoming && incoming->size() == 2 &&
                  incoming->at(0) == text("control/mode_request") &&
                  fx::msgpack_difference(payload, incoming->at(1)).empty(),
              "RPC 요청 수신");
        transport.rpc_send({text("control/mode_request"),
                            gw::msgpack_pack(gw::Wire{{"schema", "argus.follow.rpc.v1"},
                                                      {"status", "accepted"}})});
        const gw::Parts reply = receive_parts(request, 2000);
        gw::Bytes reply_packed;
        std::string reply_error;
        check(reply.size() == 2 && reply.at(0) == text("control/mode_request") &&
                  cx::decode_mode_reply(reply.at(1), reply_packed, reply_error) &&
                  gw::msgpack_unpack(reply_packed).at("status") == "accepted",
              "RPC 응답 " + reply_error);

        // PUB 발행 (늦게 붙는 구독자를 위해 몇 번 보낸다). 2026-11-25 batch 6a: 6370 은 팀
        // [이름][protobuf] 라 Pi 게이트웨이 모양의 fc/attitude 를 보낸다 (값 대조는
        // GatewayFcProtobufTest)
        gw::Parts got;
        for (int i = 0; i < 100 && got.empty(); ++i) {
            gw::Wire attitude = gw::Wire::object();
            for (const auto& [key, value] : std::initializer_list<std::pair<const char*, gw::Wire>>{
                     {"source", "argus-onboard"},
                     {"seq", i + 1},
                     {"session_id", "s"},
                     {"uid", "u"},
                     {"onboard_session", "s"},
                     {"fc_epoch", 1},
                     {"clock_domain", "host_monotonic"},
                     {"schema", "argus.fc.attitude.v1"},
                     {"received_monotonic_ns", 1},
                     {"roll_rad", 0.0},
                     {"pitch_rad", 0.0},
                     {"yaw_rad", 0.0},
                     {"fc_time_boot_ms", nullptr},
                     {"fc_time_monotonic_ns", nullptr},
                     {"timesync_valid", false},
                     {"roll_rad_s", nullptr},
                     {"pitch_rad_s", nullptr},
                     {"yaw_rad_s", nullptr}}) {
                gw::wire_set(attitude, key, value);
            }
            transport.publish({text("fc/attitude"), gw::msgpack_pack(attitude)});
            got = receive_parts(state_sub, 20);
        }
        check(got.size() == 2 && std::string(got.at(0).begin(), got.at(0).end()) == "fc/attitude",
              "fc/attitude 발행");

        // SUB 주제 필터: 구독하지 않은 주제는 받지 않는다
        std::optional<gw::Parts> command;
        for (int i = 0; i < 100 && !command; ++i) {
            send_parts(command_pub, {text("other/topic"), text("x")});
            send_parts(command_pub, {text("control/setpoint"), text("y")});
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            command = transport.command_receive();
        }
        check(command &&
                  std::string(command->at(0).begin(), command->at(0).end()) == "control/setpoint",
              "control/setpoint 수신");
        bool other = false;
        while (auto extra = transport.command_receive()) {
            other = other || std::string(extra->at(0).begin(), extra->at(0).end()) == "other/topic";
        }
        check(!other, "구독하지 않은 주제 거부");

        // 추적 1MB 제한: 큰 메시지는 버려지고 작은 메시지는 받는다. 2026-11-25 batch 6c: 소켓에
        // MAXMSGSIZE 가 없어 큰 메시지는 받은 뒤 버리고 세며 (track_oversized) 같은 연결로 작은
        // 메시지를 받는다 (예전에는 큰 메시지가 연결을 끊어 작은 메시지를 받았는지 보지 않았다)
        std::optional<gw::Parts> track;
        bool oversized = false;
        for (int i = 0; i < 100 && !(track && transport.track_oversized() > 0); ++i) {
            send_parts(track_pub,
                       {text("tracking/tracks"), gw::Bytes(std::size_t{2} * 1024 * 1024, 0x90)});
            send_parts(track_pub, {text("tracking/tracks"), text("small")});
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            while (auto item = transport.track_receive()) {
                oversized = oversized || item->at(1).size() > std::size_t{1024} * 1024;
                // 큰 메시지를 센 뒤에 받은 small 만 쓴다. PUB 는 연결마다 순서를 지키므로 큰 메시지
                // 뒤에 같은 연결로 왔다는 뜻이다 (앞 반복의 small 로 통과하지 않게 한다)
                if (item->at(1) == text("small") && transport.track_oversized() > 0) {
                    track = item;
                }
            }
        }
        check(!oversized, "1MB 넘는 추적 메시지 버림");
        check(track.has_value() && transport.track_oversized() > 0,
              "큰 추적 메시지를 세고 같은 연결로 작은 메시지 수신");
        zmq_close(state_sub);
        zmq_close(request);
    }
    {
        const gw::FollowEndpoints plain{endpoint(5), endpoint(6), endpoint(7), std::nullopt};
        gw::ZmqFollowTransport transport(plain);
        check(!transport.has_tracks() && !transport.track_receive(), "추적 주소 없으면 소켓 없음");
    }
    zmq_close(command_pub);
    zmq_close(track_pub);
    zmq_ctx_term(context);
}

}  // namespace

int main() {
    try {
        loopback();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayStage4Test 통과\n";
    return 0;
}
