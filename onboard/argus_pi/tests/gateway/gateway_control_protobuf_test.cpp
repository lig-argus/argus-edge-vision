// 게이트웨이 6372 받기 시험 (2026-11-25 batch 6b, 장비 없음, 합성 입력과 기록 fixture).
// - ZmqFollowTransport::command_receive 가 추종 제어의 팀 [이름][protobuf] (control/setpoint,
//   follow/status) 를 예전 [이름][msgpack] 으로 되살린다 (Pi 모양 setpoint 와 기록 fixture
//   follow_status_0 의 값이 키, null, 정수와 실수 구분까지 같다).
// - 이름이 정확히 같지 않은 메시지 (구독 접두만 같은 이름) 는 msgpack nil 본문이 되어 FollowBOwner
// 가
//   건너뛴다. 해석할 수 없는 본문은 빈 본문이 되어 FollowBOwner 가 예전 msgpack 실패처럼
//   invalid_setpoint_wire 로 남긴다. 두 프레임이 아닌 메시지는 예전처럼 그대로 (FollowBOwner 가
//   무시).
// - command_results 32 행 초과: 생산자 쪽 변환 (to_proto) 은 거부하고 (보내지 않음), 받는 쪽 해석은
//   행 수를 보지 않고 그대로 되살린다 (지금 동작 유지, proto/README.md batch 6b).
// - 2026-11-25 batch 6c: 6372 SUB 에 ZMQ_MAXMSGSIZE 가 없다. tcp 로 본문 상한을 넘는 프레임은 받은
//   뒤 해석 실패 (빈 본문, command_decode_failures) 로 버리고 같은 연결로 뒤 setpoint 를 받는다.
// 사용: GatewayControlProtobufTest <ARGUS root>
#include <unistd.h>
#include <zmq.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/control_protobuf.hpp"
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

// Linux 임시 포트 범위(32768~)와 겹치지 않는 20000~29999 에서 고른다 (stage4, FC 시험과 다른 자리)
std::string endpoint(int offset) {
    return "tcp://127.0.0.1:" +
           std::to_string(20000 + (((::getpid() * 7) + 1500 + offset) % 10000));
}

gw::Bytes text(const std::string& value) { return {value.begin(), value.end()}; }

std::string str(const gw::Bytes& value) { return {value.begin(), value.end()}; }

void send_parts(void* socket, const gw::Parts& parts) {
    for (std::size_t i = 0; i < parts.size(); ++i) {
        zmq_send(socket, parts.at(i).data(), parts.at(i).size(),
                 i + 1 < parts.size() ? ZMQ_SNDMORE : 0);
    }
}

// 추종 제어 ZeroMqFollowBackend 의 setpoint 모양 (envelope_locked, submit, 송신부의 키 전부)
gw::Bytes setpoint_msgpack() {
    gw::Wire value = gw::Wire::object();
    for (const auto& [key, item] : std::initializer_list<std::pair<const char*, gw::Wire>>{
             {"schema", "argus.control.setpoint.v1"},
             {"source", "argus-follow-controller"},
             {"request_id", "controller-5"},
             {"client_session", "controller"},
             {"session_id", "controller"},
             {"command_id", "controller-5"},
             {"action", "velocity"},
             {"uid", kUid},
             {"clock_domain", "host_monotonic"},
             {"onboard_session", "onboard"},
             {"fc_epoch", 1},
             {"owner_token", "token"},
             {"command_epoch", 1},
             {"generation", 2},
             {"track_id", 3},
             {"source_sample_monotonic_ns", 100},
             {"bench", false},
             {"issued_monotonic_ns", 200},
             {"seq", 5},
             {"revision", 0},
             {"valid_until_monotonic_ns", 300},
             {"stopping", false},
             {"vx_m_s", 0.5},
             {"vy_m_s", 0.0},
             {"vz_m_s", -0.25},
             {"yaw_rate_rad_s", 0.1},
             {"coordinate_frame", "body_frd"},
             {"cv_gap_active", false},
             {"forward", 0.5},
             {"right", 0.0},
             {"down", -0.25},
             {"yaw_rate", 0.1}}) {
        gw::wire_set(value, key, item);
    }
    return gw::msgpack_pack(value);
}

// 6372 PUB (추종 제어 자리) 와 그 엔드포인트에 연결한 전송 하나
struct Rig {
    void* context{zmq_ctx_new()};
    void* command_pub{nullptr};
    gw::FollowEndpoints endpoints;
    std::unique_ptr<gw::ZmqFollowTransport> transport;

    explicit Rig(int offset)
        : endpoints{endpoint(offset), endpoint(offset + 1), endpoint(offset + 2), std::nullopt} {
        const int linger = 0;
        command_pub = zmq_socket(context, ZMQ_PUB);
        zmq_setsockopt(command_pub, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_bind(command_pub, endpoints.command.c_str());
        transport = std::make_unique<gw::ZmqFollowTransport>(endpoints);
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;
    Rig(Rig&&) = delete;
    Rig& operator=(Rig&&) = delete;
    ~Rig() {
        transport.reset();
        zmq_close(command_pub);
        zmq_ctx_term(context);
    }
    // 받을 때까지 보낸다 (늦게 붙는 구독). 받은 첫 메시지
    std::optional<gw::Parts> exchange(const gw::Parts& parts) {
        for (int i = 0; i < 200; ++i) {
            send_parts(command_pub, parts);
            std::this_thread::sleep_for(5ms);
            if (auto got = transport->command_receive()) {
                while (transport->command_receive()) {
                }
                return got;
            }
        }
        return std::nullopt;
    }
};

gw::Bytes read_file(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

void transport_round_trip(const std::string& root) {
    Rig rig(0);
    const gw::Bytes setpoint = setpoint_msgpack();
    const gw::Bytes setpoint_body = fx::encode("control/setpoint", setpoint);
    check(!setpoint_body.empty(), "setpoint 본문 만들기");
    const auto got = rig.exchange({text("control/setpoint"), setpoint_body});
    check(got && got->size() == 2 && str(got->front()) == "control/setpoint",
          "control/setpoint 받기");
    if (got && got->size() == 2) {
        const std::string difference = fx::msgpack_difference(setpoint, got->back());
        check(difference.empty(), "setpoint 값 왕복: " + difference);
    }

    const gw::Bytes status = read_file(root + "/tests/protobuf/fixtures/follow_status_0.msgpack");
    check(!status.empty(), "follow_status_0 fixture");
    const gw::Bytes status_body = fx::encode("follow/status", status);
    check(!status_body.empty(), "follow/status 본문 만들기");
    const auto status_got = rig.exchange({text("follow/status"), status_body});
    check(status_got && status_got->size() == 2 && str(status_got->front()) == "follow/status",
          "follow/status 받기");
    if (status_got && status_got->size() == 2) {
        const std::string difference = fx::msgpack_difference(status, status_got->back());
        check(difference.empty(), "follow/status 값 왕복 (기록 fixture): " + difference);
    }

    // 접두만 같은 이름: 건너뛰도록 msgpack nil 본문
    const auto other = rig.exchange({text("control/setpoint_other"), setpoint_body});
    check(other && other->size() == 2 && other->back() == gw::Bytes{0xc0},
          "다른 이름은 nil 본문 (FollowBOwner 가 건너뛴다)");
    // 해석할 수 없는 본문: 빈 본문 (FollowBOwner 의 msgpack 해석이 실패한다)
    const auto bad = rig.exchange({text("control/setpoint"), gw::Bytes{0xff, 0xff, 0xff}});
    check(bad && bad->size() == 2 && bad->back().empty(), "해석 실패는 빈 본문");
    const auto empty = rig.exchange({text("follow/status"), gw::Bytes{}});
    check(empty && empty->size() == 2 && empty->back().empty(), "빈 본문은 해석 실패");
    // 세 프레임: 예전처럼 그대로 (FollowBOwner 는 두 프레임만 본다)
    const auto three = rig.exchange({text("control/setpoint"), setpoint_body, text("x")});
    check(three && three->size() == 3 && three->at(1) == setpoint_body,
          "세 프레임은 그대로 돌려준다");

    // command_results 32 행 초과: 보내는 쪽 변환은 거부, 받는 쪽 해석은 그대로 되살린다
    const gw::Bytes rows33 = cx::status_with_rows(status_body, 33);
    const auto many = rig.exchange({text("follow/status"), rows33});
    check(many && many->size() == 2 && !many->back().empty() &&
              gw::msgpack_unpack(many->back()).at("status").at("command_results").size() == 33,
          "33 행 follow/status 는 받는 쪽에서 그대로 되살린다");
    if (many && many->size() == 2 && !many->back().empty()) {
        check(fx::encode("follow/status", many->back()).empty(),
              "33 행 follow/status 는 보내는 쪽 변환이 거부한다 (32 행 한도)");
    }
    const auto rows32 =
        rig.exchange({text("follow/status"), cx::status_with_rows(status_body, 32)});
    check(rows32 && rows32->size() == 2 && !rows32->back().empty() &&
              !fx::encode("follow/status", rows32->back()).empty(),
          "32 행 follow/status 는 받고 다시 보낼 수 있다");
}

// 받을 때까지 기다린다 (보내기는 한 번)
std::optional<gw::Parts> receive_within(gw::ZmqFollowTransport& transport) {
    for (int i = 0; i < 400; ++i) {
        if (auto got = transport.command_receive()) {
            return got;
        }
        std::this_thread::sleep_for(5ms);
    }
    return std::nullopt;
}

// 2026-11-25 batch 6c: 6372 SUB 에 ZMQ_MAXMSGSIZE 를 두지 않는다 (8ca3440 의
// oversized_frame_dropped 를 바꿨다). tcp 로 본문 상한 (team_wire kMaxHeaderBytes 1 MiB) 을 넘는
// 프레임은 command_receive 가 빈 본문 (해석 실패, FollowBOwner 의 invalid_setpoint_wire) 으로
// 돌려주고 command_decode_failures 가 센다. 같은 연결로 그 뒤 setpoint 를 그대로 받는다. 8ca3440 은
// MAXMSGSIZE 로 연결이 끊겨 뒤 메시지를 받지 못했다 (drone libzmq 4.3.5 는 다시 맺지 않는다,
// ~/work/b7-probe/m.c)
void oversized_frame_rejected_connection_kept() {
    constexpr std::size_t kBodyLimit = std::size_t{1} << 20U;  // team_wire kMaxHeaderBytes
    Rig rig(20);
    const gw::Bytes setpoint_body = fx::encode("control/setpoint", setpoint_msgpack());
    const auto first = rig.exchange({text("control/setpoint"), setpoint_body});
    check(first.has_value() && first->size() == 2, "상한 시험 전 연결 확인");
    if (!first || first->size() != 2) {
        return;
    }
    const std::uint64_t before = rig.transport->command_decode_failures();
    send_parts(rig.command_pub, {text("control/setpoint"), gw::Bytes(kBodyLimit + 1, 0)});
    const auto rejected = receive_within(*rig.transport);
    check(rejected && rejected->size() == 2 && rejected->back().empty(),
          "상한을 넘는 본문은 받은 뒤 빈 본문 (해석 실패) 으로 돌려준다 (tcp)");
    check(rig.transport->command_decode_failures() == before + 1, "상한 초과를 해석 실패로 센다");
    for (int round = 0; round < 3; ++round) {
        send_parts(rig.command_pub, {text("control/setpoint"), setpoint_body});
        const auto got = receive_within(*rig.transport);
        check(got && got->size() == 2 && !got->back().empty() && got->back() == first->back(),
              "같은 연결로 뒤 setpoint 를 받는다 " + std::to_string(round));
    }
    check(rig.transport->command_decode_failures() == before + 1, "뒤 setpoint 는 해석 실패 아님");
}

// 실제 FollowBOwner 가 ZmqFollowTransport 로 받은 해석 실패를 예전 msgpack 실패처럼 남긴다
void owner_invalid_wire() {
    std::atomic<double> now{300};
    gw::FlightLinkOptions options;
    options.expected_uid = kUid;
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-session"; };
    gw::FlightLink link(std::move(options));
    const gw::FollowEndpoints endpoints{endpoint(10), endpoint(11), endpoint(12), std::nullopt};
    void* context = zmq_ctx_new();
    void* pub = zmq_socket(context, ZMQ_PUB);
    const int linger = 0;
    zmq_setsockopt(pub, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_bind(pub, endpoints.command.c_str());
    {
        auto owner = std::make_unique<gw::FollowBOwner>(
            link, std::make_unique<gw::ZmqFollowTransport>(endpoints), gw::FollowOwnerOptions{});
        gw::FollowBOwner& b = *owner;
        // 다른 이름은 건너뛴다 (해석 실패가 아니다)
        for (int i = 0; i < 40; ++i) {
            send_parts(pub, {text("control/setpoint_other"), gw::Bytes{0xff}});
            std::this_thread::sleep_for(5ms);
            b.tick();
        }
        check(b.last_reason() != "invalid_setpoint_wire", "다른 이름은 invalid_setpoint_wire 아님");
        bool invalid = false;
        for (int i = 0; i < 200 && !invalid; ++i) {
            send_parts(pub, {text("control/setpoint"), gw::Bytes{0xff, 0xff, 0xff}});
            std::this_thread::sleep_for(5ms);
            b.tick();
            invalid = b.last_reason() == "invalid_setpoint_wire";
        }
        check(invalid, "해석할 수 없는 setpoint 는 invalid_setpoint_wire");
    }
    zmq_close(pub);
    zmq_ctx_term(context);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "사용: GatewayControlProtobufTest <ARGUS root>\n";
        return 2;
    }
    try {
        transport_round_trip(argv[1]);
        owner_invalid_wire();
        oversized_frame_rejected_connection_kept();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayControlProtobufTest 통과\n";
    return 0;
}
