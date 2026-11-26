// 게이트웨이 6373 운용자 요청-응답 시험 (2026-11-25 batch 8, 장비 없음, 합성 입력).
// - FollowProxy 가 추종 제어에 팀 [follow/command][FollowCommand] 를 보낸다 (STATUS 확인 뒤 요청,
//   세션과 시각, seq, 900 ms 기한, 대상 식별값이 예전 msgpack 값과 같다).
// - 응답은 [follow/command][FollowResult] 만 받는다. 예전 이름 follow/result, 프레임 수, 빈 본문
//   (추종 제어가 해석하지 못한 요청의 응답), 깨진 본문, 256 KiB 초과, 식별자 불일치, 전송 실패는
//   ValueError 이고 자동 재전송하지 않는다.
// - FC 없는 관측 전용 STATUS (observation-only, fc_epoch 0) 로 SELECT 는 보내고 START 는 보내지
//   않는다 (validate_observation_status, validate_follow_status 그대로).
// - 실제 REQ (zmq_request) 와 가짜 추종 제어 REP 로 한 번 주고받는다.
// 사용: GatewayFollowCommandProtobufTest
#include <unistd.h>
#include <zmq.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/follow/include/follow_proxy.hpp"
#include "tests/gateway/gateway_control_fixture.hpp"

namespace gw = argus::gateway;
namespace cx = gateway_control_fixture;

namespace {

const char* const kUid = "000600000000353733323333511700280027";
const char* const kName = "follow/command";

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

gw::Bytes bytes(const std::string& text) { return {text.begin(), text.end()}; }

// 응답 값 -> [follow/command][FollowResult]
std::vector<gw::Bytes> reply(const gw::Wire& value) {
    const auto body = cx::encode_follow_result(gw::msgpack_pack(value));
    check(!body.empty(), "응답 본문 만들기");
    return {bytes(kName), body};
}

gw::Wire status_value(const std::string& command_id, bool identity) {
    gw::Wire value = gw::Wire::object();
    value["schema"] = "argus.follow.result.v1";
    value["source"] = "argus-follow-controller";
    value["command_id"] = command_id;
    value["accepted"] = true;
    value["expected_uid"] = kUid;
    value["identity_ok"] = identity;
    value["connected"] = identity;
    value["connected_uid"] = identity ? kUid : "";
    value["observation_ready"] = true;
    value["controller_session"] = "controller-1";
    value["onboard_session"] = identity ? "onboard-2" : "observation-only";
    value["fc_epoch"] = identity ? 4 : 0;
    return value;
}

gw::Wire accepted_value(const std::string& command_id) {
    gw::Wire value = gw::Wire::object();
    value["schema"] = "argus.follow.result.v1";
    value["source"] = "argus-follow-controller";
    value["command_id"] = command_id;
    value["accepted"] = true;
    value["pending"] = false;
    value["result"] = "accepted";
    return value;
}

// 가짜 전송: 보낸 요청을 해석해 남기고, 응답은 respond 가 만든다
struct Fake {
    std::vector<gw::Wire> requests;
    std::vector<std::vector<gw::Bytes>> sent;
    std::function<std::vector<gw::Bytes>(const gw::Wire&)> respond;

    std::vector<gw::Bytes> operator()(const std::vector<gw::Bytes>& parts, int /*timeout_ms*/) {
        sent.push_back(parts);
        check(parts.size() == 2 && parts.at(0) == bytes(kName),
              "요청 프레임 [follow/command][본문]");
        gw::Bytes packed;
        std::string error;
        if (parts.size() != 2 || !cx::decode_follow_command(parts.at(1), packed, error)) {
            check(false, "요청 본문 해석: " + error);
            throw gw::TransportError("bad request");
        }
        const gw::Wire request = gw::msgpack_unpack(packed);
        requests.push_back(request);
        return respond(request);
    }
};

std::string error_of(const std::function<void()>& call) {
    try {
        call();
    } catch (const gw::ValueError& error) {
        return error.what();
    }
    return "";
}

void test_start_after_status() {
    Fake fake;
    fake.respond = [](const gw::Wire& request) {
        const std::string id = request.at("command_id");
        return reply(request.at("action") == "STATUS" ? status_value(id, true)
                                                      : accepted_value(id));
    };
    int next = 0;
    gw::FollowProxy proxy(
        "tcp://127.0.0.1:1", std::ref(fake), [] { return std::int64_t{5'000'000'000}; },
        [&next] { return "id" + std::to_string(next++); });
    gw::Wire body = gw::Wire::object();
    body["action"] = "start";
    body["expected_uid"] = kUid;
    body["sensor"] = "camera-original";
    body["generation"] = 16777215;
    body["track_id"] = 9;
    body["class_id"] = 0;
    body["label"] = "car";
    const gw::Wire result = proxy.forward(body, kUid);
    check(result.at("accepted") == true, "START 수락 응답");
    check(fake.requests.size() == 2, "STATUS 뒤 START 하나");
    if (fake.requests.size() != 2) {
        return;
    }
    const auto& status = fake.requests.at(0);
    check(status.at("action") == "STATUS" && status.at("schema") == "argus.follow.command.v1" &&
              status.at("expected_uid") == kUid && !status.contains("seq"),
          "STATUS 요청 값");
    const auto& start = fake.requests.at(1);
    check(start.at("action") == "START" && start.at("sensor") == "camera-original" &&
              start.at("generation") == 16777215 && start.at("track_id") == 9 &&
              start.at("class_id") == 0 && start.at("label") == "car",
          "START 대상 식별값");
    check(start.at("controller_session") == "controller-1" &&
              start.at("onboard_session") == "onboard-2" && start.at("fc_epoch") == 4,
          "START 세션은 STATUS 에서");
    check(start.at("seq") == 1 && start.at("clock_domain") == "host_monotonic" &&
              start.at("issued_monotonic_ns") == 5'000'000'000 &&
              start.at("valid_until_monotonic_ns") == 5'900'000'000,
          "START seq, 시각, 900 ms 기한");
    check(start.at("source") == proxy.source(), "START source");
}

void test_observation_only_context() {
    Fake fake;
    fake.respond = [](const gw::Wire& request) {
        const std::string id = request.at("command_id");
        return reply(request.at("action") == "STATUS" ? status_value(id, false)
                                                      : accepted_value(id));
    };
    gw::FollowProxy proxy("tcp://127.0.0.1:1", std::ref(fake));
    gw::Wire body = gw::Wire::object();
    body["action"] = "select";
    body["expected_uid"] = kUid;
    body["sensor"] = "camera-original";
    body["generation"] = 3;
    body["track_id"] = 9;
    body["class_id"] = 0;
    body["label"] = "car";
    check(proxy.forward(body, kUid).at("accepted") == true, "FC 없는 SELECT 수락");
    check(fake.requests.size() == 2 && fake.requests.at(1).at("action") == "SELECT" &&
              fake.requests.at(1).at("onboard_session") == "observation-only" &&
              fake.requests.at(1).at("fc_epoch") == 0,
          "SELECT 는 관측 전용 문맥으로");
    fake.requests.clear();
    body["action"] = "start";
    check(!error_of([&] { proxy.forward(body, kUid); }).empty(), "관측 전용 문맥의 START 거부");
    check(fake.requests.size() == 1 && fake.requests.at(0).at("action") == "STATUS",
          "START 거부는 STATUS 만 보낸다");
}

void test_reply_shapes() {
    const std::vector<
        std::pair<std::string, std::function<std::vector<gw::Bytes>(const gw::Wire&)>>>
        cases{
            {"B 추종 응답 multipart 형식 오류",
             [](const gw::Wire& r) {
                 auto parts = reply(status_value(r.at("command_id"), true));
                 parts.at(0) = bytes("follow/result");  // 예전 응답 이름
                 return parts;
             }},
            {"B 추종 응답 multipart 형식 오류",
             [](const gw::Wire& r) {
                 auto parts = reply(status_value(r.at("command_id"), true));
                 parts.pop_back();
                 return parts;
             }},
            {"B 추종 응답 multipart 형식 오류",
             [](const gw::Wire& r) {
                 auto parts = reply(status_value(r.at("command_id"), true));
                 parts.push_back(bytes("extra"));
                 return parts;
             }},
            {"B 추종 응답 해석 실패",
             [](const gw::Wire&) { return std::vector<gw::Bytes>{bytes(kName), {}}; }},
            {"B 추종 응답 해석 실패",
             [](const gw::Wire&) {
                 return std::vector<gw::Bytes>{bytes(kName), bytes("\xff\xff\xff")};
             }},
            {"B 추종 응답 해석 실패",
             [](const gw::Wire&) {
                 return std::vector<gw::Bytes>{bytes(kName), gw::Bytes((256 << 10) + 1, 0x0a)};
             }},
            {"B 추종 응답 식별자 불일치",
             [](const gw::Wire&) { return reply(status_value("other", true)); }},
            {"B 추종 응답 식별자 불일치",
             [](const gw::Wire& r) {
                 auto value = status_value(r.at("command_id"), true);
                 value["schema"] = "argus.follow.result.v2";
                 return reply(value);
             }},
    };
    for (const auto& [expected, respond] : cases) {
        Fake fake;
        fake.respond = respond;
        gw::FollowProxy proxy("tcp://127.0.0.1:1", std::ref(fake));
        gw::Wire body = gw::Wire::object();
        body["action"] = "stop";
        body["expected_uid"] = kUid;
        const auto error = error_of([&] { proxy.forward(body, kUid); });
        std::string message = "응답 거부 문구 ";
        message += expected;
        message += " != ";
        message += error;
        check(error == expected, message);
        check(fake.requests.size() == 1, "거부 뒤 자동 재전송 없음 (" + expected + ")");
    }
    gw::FollowProxy offline("tcp://127.0.0.1:1",
                            [](const std::vector<gw::Bytes>&, int) -> std::vector<gw::Bytes> {
                                throw gw::TransportError("timeout");
                            });
    gw::Wire body = gw::Wire::object();
    body["action"] = "status";
    body["expected_uid"] = kUid;
    check(error_of([&] { offline.forward(body, kUid); }).find("B 추종 연결부 미응답") == 0,
          "전송 실패는 미응답");
}

// 실제 REQ 와 가짜 추종 제어 REP 하나
void test_real_socket() {
    void* context = zmq_ctx_new();
    void* rep = zmq_socket(context, ZMQ_REP);
    const int linger = 0;
    zmq_setsockopt(rep, ZMQ_LINGER, &linger, sizeof(linger));
    check(zmq_bind(rep, "tcp://127.0.0.1:*") == 0, "REP bind");
    std::array<char, 256> endpoint{};
    std::size_t size = endpoint.size();
    zmq_getsockopt(rep, ZMQ_LAST_ENDPOINT, endpoint.data(), &size);
    std::atomic<bool> served{false};
    std::thread server([&] {
        std::vector<gw::Bytes> parts;
        for (bool more = true; more;) {
            zmq_msg_t message;
            zmq_msg_init(&message);
            if (zmq_msg_recv(&message, rep, 0) < 0) {
                zmq_msg_close(&message);
                return;
            }
            const auto* data = static_cast<const std::uint8_t*>(zmq_msg_data(&message));
            parts.emplace_back(data, data + zmq_msg_size(&message));
            more = zmq_msg_more(&message) != 0;
            zmq_msg_close(&message);
        }
        gw::Bytes packed;
        std::string error;
        const bool ok = parts.size() == 2 && parts.at(0) == bytes(kName) &&
                        cx::decode_follow_command(parts.at(1), packed, error);
        const auto out = reply(status_value(
            ok ? gw::msgpack_unpack(packed).at("command_id").get<std::string>() : "bad", true));
        zmq_send(rep, out.at(0).data(), out.at(0).size(), ZMQ_SNDMORE);
        zmq_send(rep, out.at(1).data(), out.at(1).size(), 0);
        served = ok;
    });
    gw::FollowProxy proxy(endpoint.data());
    gw::Wire body = gw::Wire::object();
    body["action"] = "status";
    body["expected_uid"] = kUid;
    body["request_id"] = "real-socket";
    std::string error;
    gw::Wire result;
    try {
        result = proxy.forward(body, kUid, 2000);
    } catch (const std::exception& failure) {
        error = failure.what();
    }
    server.join();
    check(error.empty() && served && result.at("command_id") == "real-socket" &&
              result.at("identity_ok") == true,
          "실제 REQ 왕복: " + error);
    zmq_close(rep);
    zmq_ctx_term(context);
}

}  // namespace

int main() {
    try {
        test_start_after_status();
        test_observation_only_context();
        test_reply_shapes();
        test_real_socket();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << "GatewayFollowCommandProtobufTest 실패 " << failures() << '\n';
        return 1;
    }
    std::cout << "GatewayFollowCommandProtobufTest 통과\n";
    return 0;
}
