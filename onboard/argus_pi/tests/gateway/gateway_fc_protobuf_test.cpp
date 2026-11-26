// 게이트웨이 FC 텔레메트리 6370 시험 (2026-11-25 batch 6a, 장비 없음, 합성 입력과 기록 fixture).
// - ZmqFollowTransport 가 FollowBOwner 의 [이름][msgpack] 을 팀 [이름][protobuf] 로 보내고, 받는 쪽
//   to_json 이 같은 msgpack 값을 되살린다 (기록 fixture 와 실제 FollowBOwner 출력). 팀 seq 는
//   토픽별 발행 번호, Pi seq 는 pi_seq 그대로.
// - 변환 실패는 보내지 않고 예외도 내지 않는다 (팀 seq 는 쓴다). 6370 이 아닌 이름은 예외.
// - 정비 지상 확인 (Maintenance::confirmed_ground) 은 팀 본문을 받고 예전 키 검사를 그대로 한다:
//   status 없는 본문 (armed, in_air 없음), 이름 불일치, 해석 실패, armed 는 거부.
// - 2026-11-25 6a 리뷰: publish 는 FC 링크 잠금 안에서 대기열에 넣기만 한다 (변환과 send 는 송신
//   스레드). fc/state 발행 호출의 중앙값이 150 us 미만이어야 한다 (f934c74 는 약 670 us). 송신
//   스레드를 거쳐도 토픽 사이 순서와 토픽별 팀 seq 순서가 지켜진다. msgpack 으로 해석할 수 없는
//   본문도 팀 seq 를 쓴다 (f934c74 는 쓰지 않았다).
// 사용: GatewayFcProtobufTest <ARGUS root> [measure <count>]
#include <unistd.h>
#include <zmq.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/follow_owner.hpp"
#include "argus/gateway/follow/include/zmq_transport.hpp"
#include "argus/gateway/service/include/maintenance.hpp"
#include "tests/gateway/gateway_fc_fixture.hpp"

namespace gw = argus::gateway;
namespace fx = gateway_fc_fixture;
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

// Linux 임시 포트 범위(32768~)와 겹치지 않는 20000~29999 에서 고른다 (stage4 와 다른 자리)
std::string endpoint(int offset) {
    return "tcp://127.0.0.1:" + std::to_string(20000 + (((::getpid() * 7) + 500 + offset) % 10000));
}

gw::Bytes text(const std::string& value) { return {value.begin(), value.end()}; }

std::string str(const gw::Bytes& value) { return {value.begin(), value.end()}; }

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

void send_parts(void* socket, const gw::Parts& parts) {
    for (std::size_t i = 0; i < parts.size(); ++i) {
        zmq_send(socket, parts.at(i).data(), parts.at(i).size(),
                 i + 1 < parts.size() ? ZMQ_SNDMORE : 0);
    }
}

// 전송 하나와 그 PUB 에 붙은 raw SUB ("fc/" 접두). 늦게 붙는 구독자를 위해 처음 한 번은 받을 때까지
// 다시 보낸다
struct Rig {
    void* context{zmq_ctx_new()};
    void* command_pub{nullptr};
    void* sub{nullptr};
    std::unique_ptr<gw::ZmqFollowTransport> transport;

    explicit Rig(int offset) {
        const int linger = 0;
        command_pub = zmq_socket(context, ZMQ_PUB);
        zmq_setsockopt(command_pub, ZMQ_LINGER, &linger, sizeof(linger));
        const gw::FollowEndpoints endpoints{endpoint(offset), endpoint(offset + 1),
                                            endpoint(offset + 2), std::nullopt};
        zmq_bind(command_pub, endpoints.command.c_str());
        transport = std::make_unique<gw::ZmqFollowTransport>(endpoints);
        sub = zmq_socket(context, ZMQ_SUB);
        zmq_setsockopt(sub, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "fc/", 3);
        zmq_connect(sub, endpoints.state.c_str());
    }
    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;
    Rig(Rig&&) = delete;
    Rig& operator=(Rig&&) = delete;
    ~Rig() {
        transport.reset();
        zmq_close(sub);
        zmq_close(command_pub);
        zmq_ctx_term(context);
    }
    // 구독이 붙을 때까지 attitude 를 보내 본다 (받은 것은 버린다. 팀 seq 가 그만큼 늘어난다)
    // 반환값: 보낸 수 (그 토픽의 마지막 팀 seq). 붙지 못하면 -1
    int warm_up(const gw::Parts& parts) {
        for (int published = 1; published <= 200; ++published) {
            transport->publish(parts);
            if (!receive_parts(sub, 10).empty()) {
                while (!receive_parts(sub, 50).empty()) {
                }
                return published;
            }
        }
        return -1;
    }
};

gw::Wire fixture_wire(const std::string& root, const std::string& name) {
    return gw::msgpack_unpack(fx::read_file(root + "/tests/protobuf/fixtures/" + name));
}

// 보낸 msgpack 과 받은 protobuf 의 to_json 이 같은지, 팀 seq 가 기대값인지
void expect_round_trip(const gw::Parts& sent, const gw::Parts& got, std::uint64_t team_seq,
                       const std::string& what) {
    check(got.size() == 2 && got.at(0) == sent.at(0), what + ": [이름][본문] 두 프레임과 이름");
    if (got.size() != 2) {
        return;
    }
    gw::Bytes packed;
    std::uint64_t sequence = 0;
    std::string error;
    check(fx::decode(str(got.at(0)), got.at(1), packed, sequence, error), what + ": 해석 " + error);
    const std::string difference = fx::msgpack_difference(sent.at(1), packed);
    check(difference.empty(), what + ": 값 다름 " + difference);
    check(sequence == team_seq, what + ": 팀 seq " + std::to_string(sequence) + " (기대 " +
                                    std::to_string(team_seq) + ")");
}

void fixture_round_trip(const std::string& root) {
    Rig rig(1);
    const gw::Parts warm{text("fc/attitude"),
                         gw::msgpack_pack(fixture_wire(root, "fc_attitude_0.msgpack"))};
    int attitude_seq = rig.warm_up(warm);
    check(attitude_seq > 0, "구독 연결");
    int state_seq = 0;
    const auto send_one = [&](const char* topic, const std::string& name, int& counter) {
        const gw::Parts sent{text(topic), gw::msgpack_pack(fixture_wire(root, name))};
        rig.transport->publish(sent);
        ++counter;
        expect_round_trip(sent, receive_parts(rig.sub, 1000), static_cast<std::uint64_t>(counter),
                          "fixture " + name);
    };
    for (int i = 0; i < 3; ++i) {
        send_one("fc/state", "fc_state_" + std::to_string(i) + ".msgpack", state_seq);
        send_one("fc/attitude", "fc_attitude_" + std::to_string(i) + ".msgpack", attitude_seq);
    }
    check(fixture_wire(root, "fc_state_0.msgpack").contains("cmc_pose_history"),
          "fixture 에 cmc_pose_history 가 있다");
}

// FollowBOwner 를 거치지 않는 관측을 위해 지금 표본을 넣는다 (MAVLink 디코더처럼 실수는 double)
void seed(gw::FlightState& state, double now) {
    const auto set = [&](const char* name, gw::Json data) {
        state.samples.insert_or_assign(name, gw::Sample{std::move(data), now});
    };
    const auto numbers = gw::mode_numbers("position").value_or(std::pair<int, int>{0, 0});
    set("HEARTBEAT",
        {{"base_mode", 128}, {"custom_mode", (numbers.first << 16) | (numbers.second << 24)}});
    set("EXTENDED_SYS_STATE", {{"landed_state", 2}});
    set("LOCAL_POSITION_NED", {{"time_boot_ms", 1000},
                               {"x", 2.0},
                               {"y", 3.0},
                               {"z", -1.7},
                               {"vx", 0.0},
                               {"vy", 0.0},
                               {"vz", 0.0}});
    set("ATTITUDE", {{"time_boot_ms", 1001},
                     {"roll", 0.0},
                     {"pitch", 0.0},
                     {"yaw", .2},
                     {"rollspeed", 0.01},
                     {"pitchspeed", 0.0},
                     {"yawspeed", -0.02}});
    set("DISTANCE_SENSOR", {{"time_boot_ms", 1002},
                            {"current_distance", 170},
                            {"min_distance", 8},
                            {"max_distance", 400},
                            {"orientation", 25}});
    set("OPTICAL_FLOW_RAD", {{"quality", 50}});
}

class CaptureTransport final : public gw::IFollowTransport {
public:
    explicit CaptureTransport(std::vector<gw::Parts>& sent) : sent_(sent) {}
    void publish(const gw::Parts& parts) override { sent_.push_back(parts); }
    std::optional<gw::Parts> rpc_receive() override { return std::nullopt; }
    void rpc_send(const gw::Parts& /*parts*/) override {}
    std::optional<gw::Parts> command_receive() override { return std::nullopt; }
    bool has_tracks() const override { return false; }
    std::optional<gw::Parts> track_receive() override { return std::nullopt; }
    void close() override {}

private:
    std::vector<gw::Parts>& sent_;
};

// 실제 FollowBOwner::publish 의 fc/state, fc/attitude 와 mode_event 모양 (진단 키 포함)
void owner_round_trip() {
    std::atomic<double> now{300};
    gw::FlightLinkOptions options;
    options.expected_uid = kUid;
    options.event_metadata = "/nonexistent/fc-events.json";
    options.clock = [&] { return now.load(); };
    options.new_id = [] { return "test-session"; };
    gw::FlightLink link(std::move(options));
    std::vector<gw::Parts> sent;
    link.state().follow_b = std::make_unique<gw::FollowBOwner>(
        link, std::make_unique<CaptureTransport>(sent), gw::FollowOwnerOptions{});
    seed(link.state(), now);
    link.step();
    check(sent.size() == 2 && str(sent.at(0).at(0)) == "fc/state" &&
              str(sent.at(1).at(0)) == "fc/attitude",
          "FollowBOwner 첫 발행");
    if (sent.size() != 2) {
        return;
    }
    Rig rig(11);
    int attitude_seq = rig.warm_up(sent.at(1));
    check(attitude_seq > 0, "구독 연결 (owner)");
    rig.transport->publish(sent.at(0));
    expect_round_trip(sent.at(0), receive_parts(rig.sub, 1000), 1, "FollowBOwner fc/state");
    rig.transport->publish(sent.at(1));
    expect_round_trip(sent.at(1), receive_parts(rig.sub, 1000),
                      static_cast<std::uint64_t>(attitude_seq) + 1, "FollowBOwner fc/attitude");

    gw::Wire event = gw::Wire::object();
    for (const auto& [key, value] : std::initializer_list<std::pair<const char*, gw::Wire>>{
             {"schema", "argus.fc.mode_event.v1"},
             {"uid", kUid},
             {"onboard_session", "test-session"},
             {"requester_session_id", "controller"},
             {"fc_epoch", 3},
             {"clock_domain", "host_monotonic"},
             {"received_monotonic_ns", 123456789},
             {"command_id", "controller-7"},
             {"event", "MODE_FAILED"},
             {"actual_mode", "POSITION"},
             {"owner_token", "token"},
             {"revision", 4},
             {"command_epoch", 2},
             {"reason", "mode_rejected"},
             {"mav_result", 4},
             {"mav_result_param2", -1},
             {"source", "argus-onboard"},
             {"seq", 99},
             {"session_id", "test-session"}}) {
        gw::wire_set(event, key, value);
    }
    const gw::Parts mode{text("fc/mode_event"), gw::msgpack_pack(event)};
    rig.transport->publish(mode);
    expect_round_trip(mode, receive_parts(rig.sub, 1000), 1, "mode_event (MODE_FAILED 진단)");

    // 변환 실패: 예외 없이 보내지 않고, 팀 seq 는 쓴다 (다음 발행은 3)
    const gw::Parts broken{text("fc/state"), gw::msgpack_pack(gw::Wire{{"seq", 1}})};
    bool threw = false;
    try {
        rig.transport->publish(broken);
    } catch (const std::exception&) {
        threw = true;
    }
    check(!threw, "변환 실패는 예외가 아니다");
    check(receive_parts(rig.sub, 100).empty(), "변환 실패 본문은 보내지 않는다");
    rig.transport->publish(sent.at(0));
    expect_round_trip(sent.at(0), receive_parts(rig.sub, 1000), 3, "변환 실패 뒤 fc/state");

    // 2026-11-25 6a 리뷰: msgpack 으로 해석할 수 없는 본문도 보내지 않고 팀 seq 를 쓴다 (다음은 5)
    threw = false;
    try {
        rig.transport->publish({text("fc/state"), gw::Bytes{0xc1}});
    } catch (const std::exception&) {
        threw = true;
    }
    check(!threw, "msgpack 해석 실패는 예외가 아니다");
    check(receive_parts(rig.sub, 100).empty(), "msgpack 해석 실패 본문은 보내지 않는다");
    rig.transport->publish(sent.at(0));
    expect_round_trip(sent.at(0), receive_parts(rig.sub, 1000), 5,
                      "msgpack 해석 실패 뒤 fc/state (팀 seq 를 썼다)");

    threw = false;
    try {
        rig.transport->publish({text("fc/other"), gw::msgpack_pack(event)});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    check(threw, "6370 이 아닌 이름은 예외");
}

double micros(std::chrono::steady_clock::time_point from,
              std::chrono::steady_clock::time_point to) {
    return std::chrono::duration<double, std::micro>(to - from).count();
}

// 2026-11-25 6a 리뷰: 발행 호출 (FC 링크 잠금 안의 비용) 은 대기열 넣기뿐이고, 송신 스레드를 거쳐도
// 토픽 사이 순서와 토픽별 팀 seq 가 발행 순서대로다
void off_lock_publish(const std::string& root) {
    Rig rig(51);
    const gw::Parts attitude{text("fc/attitude"),
                             gw::msgpack_pack(fixture_wire(root, "fc_attitude_0.msgpack"))};
    const gw::Parts state{text("fc/state"),
                          gw::msgpack_pack(fixture_wire(root, "fc_state_0.msgpack"))};
    const int attitude_base = rig.warm_up(attitude);
    check(attitude_base > 0, "구독 연결 (off-lock)");
    constexpr int kTimed = 200;
    std::vector<double> call_us;
    int delivered = 0;
    for (int i = 0; i < kTimed; ++i) {
        const auto a = std::chrono::steady_clock::now();
        rig.transport->publish(state);
        const auto b = std::chrono::steady_clock::now();
        call_us.push_back(micros(a, b));
        delivered += receive_parts(rig.sub, 1000).empty() ? 0 : 1;
        std::this_thread::sleep_for(1ms);
    }
    std::sort(call_us.begin(), call_us.end());
    const double median = call_us.at(call_us.size() / 2);
    check(delivered == kTimed,
          "1 ms 간격 fc/state 는 모두 도착 (" + std::to_string(delivered) + ")");
    check(median < 150.0, "fc/state 발행 호출 중앙값 " + std::to_string(median) +
                              " us (150 us 미만이어야 한다. 변환은 송신 스레드)");

    // 몰아서 보내기: 대기열 (64) 이나 HWM 에서 버려질 수 있지만 받은 것의 순서는 발행 순서다
    constexpr int kBurst = 100;
    bool threw = false;
    try {
        for (int i = 0; i < kBurst; ++i) {
            rig.transport->publish(state);
            rig.transport->publish(attitude);
        }
    } catch (const std::exception& error) {
        threw = true;
        std::cerr << "몰아서 보내기 예외: " << error.what() << '\n';
    }
    check(!threw, "몰아서 보내도 예외가 없다 (대기열 가득 참은 세기만 한다)");
    long last_index = -1;
    int got = 0;
    bool ordered = true;
    for (;;) {
        const gw::Parts parts = receive_parts(rig.sub, 300);
        if (parts.size() != 2) {
            break;
        }
        gw::Bytes packed;
        std::uint64_t sequence = 0;
        std::string error;
        if (!fx::decode(str(parts.at(0)), parts.at(1), packed, sequence, error)) {
            ordered = false;
            break;
        }
        // 발행 순서 번호: state 는 짝수, attitude 는 홀수 자리
        const bool is_state = str(parts.at(0)) == "fc/state";
        const long round = is_state ? static_cast<long>(sequence) - kTimed - 1
                                    : static_cast<long>(sequence) - attitude_base - 1;
        const long index = (round * 2) + (is_state ? 0 : 1);
        if (round < 0 || round >= kBurst || index <= last_index) {
            ordered = false;
        }
        last_index = index;
        ++got;
    }
    check(got > 0 && ordered,
          "송신 스레드를 거친 순서와 팀 seq 는 발행 순서 (받은 " + std::to_string(got) + ")");
}

// 정비 지상 확인. 다른 스레드가 parts 를 20 ms 마다 보내는 동안 confirmed_ground 를 부른다
bool ground_with(const gw::Parts& parts, int offset) {
    void* context = zmq_ctx_new();
    void* pub = zmq_socket(context, ZMQ_PUB);
    const int linger = 0;
    zmq_setsockopt(pub, ZMQ_LINGER, &linger, sizeof(linger));
    const std::string address = endpoint(offset);
    zmq_bind(pub, address.c_str());
    std::atomic<bool> stop{false};
    std::thread publisher([&] {
        while (!stop) {
            send_parts(pub, parts);
            std::this_thread::sleep_for(20ms);
        }
    });
    gw::Wire config = gw::Wire::object();
    gw::wire_set(config, "endpoints", gw::Wire{{"fc_state", address}});
    gw::wire_set(config, "device", gw::Wire{{"expected_fc_uid", kUid}});
    bool result = false;
    try {
        result = gw::Maintenance::confirmed_ground(config);
    } catch (const std::exception& error) {
        std::cerr << "confirmed_ground 예외: " << error.what() << '\n';
    }
    stop = true;
    publisher.join();
    zmq_close(pub);
    zmq_ctx_term(context);
    return result;
}

std::int64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 지상 확인 대상 fc/state 본문. 지금 시각으로 만들어 곧바로 보내므로 500 ms 신선도 안에 받는다
gw::Bytes landed_state(const std::string& root, bool armed) {
    gw::Wire state = fixture_wire(root, "fc_state_0.msgpack");
    for (const auto& [key, value] : std::initializer_list<std::pair<const char*, gw::Wire>>{
             {"uid", kUid},
             {"clock_domain", "host_monotonic"},
             {"connected", true},
             {"armed", armed},
             {"in_air", false},
             {"ground_known", true},
             {"landed", true},
             {"offboard", false},
             {"rc_override", false},
             {"received_monotonic_ns", monotonic_ns()}}) {
        gw::wire_set(state, key, value);
    }
    return fx::encode("fc/state", gw::msgpack_pack(state));
}

void maintenance_ground(const std::string& root) {
    const gw::Bytes landed = landed_state(root, false);
    check(!landed.empty(), "지상 본문 만들기");
    check(ground_with({text("fc/state"), landed}, 21), "착지 fc/state 는 지상 확인");
    check(!ground_with({text("fc/state"), landed_state(root, true)}, 22), "armed 는 거부");
    check(!ground_with({text("fc/state"), fx::without_status(landed)}, 23),
          "status (armed, in_air) 가 없는 본문은 거부");
    check(!ground_with({text("fc/state_extra"), landed}, 24),
          "이름이 정확히 fc/state 가 아니면 거부");
    check(!ground_with({text("fc/state"), gw::Bytes{0xff, 0xff, 0xff}}, 25), "해석 실패는 거부");
    check(!ground_with(
              {text("fc/state"), gw::msgpack_pack(gw::msgpack_unpack(fx::read_file(
                                     root + "/tests/protobuf/fixtures/fc_state_0.msgpack")))},
              26),
          "예전 msgpack 본문은 거부");
}

// 합성 측정: FC 링크 잠금 안의 발행 한 번 비용과 구독자 도착까지의 시간. 예전 (msgpack 두 프레임
// send) 과 지금 (ZmqFollowTransport::publish = 대기열 넣기. 변환과 send 는 송신 스레드).
// msgpack_pack 은 예전과 같고 잠금 안이다. 도착 시간은 같은 스레드에서 막고 받아 잰다
void measure(const std::string& root, int count) {
    for (const char* name : {"fc_state_0.msgpack", "fc_attitude_0.msgpack"}) {
        const std::string topic =
            std::string(name).rfind("fc_state", 0) == 0 ? "fc/state" : "fc/attitude";
        const gw::Wire value = fixture_wire(root, name);
        Rig rig(31);
        const gw::Parts warm{text(topic), gw::msgpack_pack(value)};
        rig.warm_up(warm);
        void* raw_pub = zmq_socket(rig.context, ZMQ_PUB);
        const int hwm = 64;
        const int linger = 0;
        zmq_setsockopt(raw_pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));
        zmq_setsockopt(raw_pub, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_bind(raw_pub, endpoint(41).c_str());
        void* raw_sub = zmq_socket(rig.context, ZMQ_SUB);
        zmq_setsockopt(raw_sub, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_setsockopt(raw_sub, ZMQ_SUBSCRIBE, "fc/", 3);
        zmq_connect(raw_sub, endpoint(41).c_str());
        std::this_thread::sleep_for(200ms);
        std::vector<double> pack_us;
        std::vector<double> old_us;
        std::vector<double> old_delivery_us;
        std::vector<double> new_us;
        std::vector<double> new_delivery_us;
        std::size_t body_bytes = 0;
        int received = 0;
        for (int i = 0; i < count; ++i) {
            const auto a = std::chrono::steady_clock::now();
            const gw::Bytes packed = gw::msgpack_pack(value);
            const auto b = std::chrono::steady_clock::now();
            send_parts(raw_pub, {text(topic), packed});
            const auto c = std::chrono::steady_clock::now();
            const bool old_got = !receive_parts(raw_sub, 1000).empty();
            const auto d = std::chrono::steady_clock::now();
            rig.transport->publish({text(topic), packed});
            const auto e = std::chrono::steady_clock::now();
            const bool new_got = !receive_parts(rig.sub, 1000).empty();
            const auto f = std::chrono::steady_clock::now();
            pack_us.push_back(micros(a, b));
            old_us.push_back(micros(b, c));
            new_us.push_back(micros(d, e));
            if (old_got && new_got) {
                old_delivery_us.push_back(micros(b, d));
                new_delivery_us.push_back(micros(d, f));
                ++received;
            }
            body_bytes = packed.size();
            std::this_thread::sleep_for(1ms);  // 1 ms 간격 (HWM 측정에서 유실 없는 간격)
        }
        zmq_close(raw_sub);
        zmq_close(raw_pub);
        const auto stats = [](std::vector<double> values) {
            if (values.empty()) {
                return std::string("(없음)");
            }
            std::sort(values.begin(), values.end());
            double sum = 0;
            for (const double v : values) {
                sum += v;
            }
            const auto at = [&](double q) {
                return values.at(
                    static_cast<std::size_t>(q * static_cast<double>(values.size() - 1)));
            };
            std::array<char, 160> line{};
            std::snprintf(line.data(), line.size(), "mean %.1f p50 %.1f p99 %.1f max %.1f us",
                          sum / static_cast<double>(values.size()), at(.5), at(.99), values.back());
            return std::string(line.data());
        };
        std::cout << "FC_PUBLISH_MEASURE topic=" << topic << " msgpack_bytes=" << body_bytes
                  << " count=" << count << " received=" << received << "\n  msgpack_pack "
                  << stats(pack_us) << "\n  old_send " << stats(old_us) << "\n  new_publish "
                  << stats(new_us) << "\n  old_delivery " << stats(old_delivery_us)
                  << "\n  new_delivery " << stats(new_delivery_us) << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "사용: GatewayFcProtobufTest <ARGUS root> [measure <count>]\n";
        return 2;
    }
    const std::string root = argv[1];
    try {
        if (argc == 4 && std::string(argv[2]) == "measure") {
            measure(root, std::stoi(argv[3]));
            return 0;
        }
        fixture_round_trip(root);
        owner_round_trip();
        off_lock_publish(root);
        maintenance_ground(root);
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayFcProtobufTest 통과\n";
    return 0;
}
