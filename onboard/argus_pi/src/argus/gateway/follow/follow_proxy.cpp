#include "argus/gateway/follow/include/follow_proxy.hpp"

#include <zmq.h>

#include <array>
#include <chrono>
#include <iostream>
#include <random>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "argus/gateway/follow/include/follow_command_protobuf.hpp"

namespace argus::gateway {

namespace {

constexpr const char* kUnresponsive =
    "B 추종 연결부 미응답 · 요청 결과 미확인 · 자동 재전송하지 않음";

std::string random_hex() {
    static constexpr std::array<char, 16> kDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                                  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    static thread_local std::mt19937_64 engine{std::random_device{}()};
    std::string out;
    for (int i = 0; i < 32; ++i) {
        out += kDigits.at(engine() & 0x0fU);
    }
    return out;
}

std::int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool int_in(const Wire& value, std::int64_t low, std::int64_t high) {
    if (value.is_number_unsigned()) {
        return value.get<std::uint64_t>() <= static_cast<std::uint64_t>(high) &&
               static_cast<std::int64_t>(value.get<std::uint64_t>()) >= low;
    }
    return value.is_number_integer() && value.get<std::int64_t>() >= low &&
           value.get<std::int64_t>() <= high;
}

bool session_text(const Wire& value) {
    return value.is_string() && !value.get_ref<const std::string&>().empty() &&
           value.get_ref<const std::string&>().size() <= 128;
}

void* zmq_context() {
    static void* context = zmq_ctx_new();
    return context;
}

struct Socket {
    void* handle{nullptr};
    explicit Socket(int type) : handle(zmq_socket(zmq_context(), type)) {}
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;
    Socket(Socket&&) = delete;
    Socket& operator=(Socket&&) = delete;
    ~Socket() {
        if (handle != nullptr) {
            zmq_close(handle);
        }
    }
};

}  // namespace

Wire selection_identity(const Wire& body, bool allow_icd) {
    Wire result = Wire::object();
    for (const auto& [name, minimum, maximum] :
         {std::tuple<const char*, std::int64_t, std::int64_t>{"generation", 1, 16777215},
          std::tuple<const char*, std::int64_t, std::int64_t>{"track_id", 1, 16777215},
          std::tuple<const char*, std::int64_t, std::int64_t>{"class_id", 0, 16777215}}) {
        const Wire value = wire_get(body, name);
        if (!int_in(value, minimum, maximum)) {
            throw ValueError(std::string("대상 식별자 오류: ") + name);
        }
        wire_set(result, name, value);
    }
    for (const char* name : {"sensor", "label"}) {
        const Wire value = wire_get(body, name);
        const std::string key(name);
        if (allow_icd && key == "sensor" && value.is_number_integer() &&
            (python_equal(value, Wire(1)) || python_equal(value, Wire(2)))) {
            wire_set(result, name, value);
            continue;
        }
        if (allow_icd && key == "label" && !body.contains(name)) {
            continue;
        }
        if (!value.is_string() || value.get_ref<const std::string&>().empty()) {
            throw ValueError(std::string("대상 문자열 오류: ") + name);
        }
        for (const char32_t code : utf8_to_u32(value.get<std::string>())) {
            if (code < 32 || code == 127) {
                throw ValueError(std::string("대상 문자열 오류: ") + name);
            }
        }
        if (value.get_ref<const std::string&>().size() > 128) {
            throw ValueError(std::string("대상 문자열 길이 초과: ") + name);
        }
        wire_set(result, name, value);
    }
    return result;
}

void validate_follow_status(const Wire& status, const std::string& uid) {
    if (wire_get(status, "accepted") != true ||
        !python_equal(wire_get(status, "expected_uid"), Wire(uid)) ||
        !python_equal(wire_get(status, "connected_uid"), Wire(uid)) ||
        wire_get(status, "identity_ok") != true || wire_get(status, "connected") != true) {
        throw ValueError("B 추종 FC UID/신선한 연결 확인 실패");
    }
    for (const char* key : {"controller_session", "onboard_session"}) {
        if (!session_text(wire_get(status, key))) {
            throw ValueError(std::string("B 추종 세션 미확인: ") + key);
        }
    }
    const Wire epoch = wire_get(status, "fc_epoch");
    const bool epoch_ok =
        epoch.is_number_unsigned() || (epoch.is_number_integer() && epoch.get<std::int64_t>() >= 0);
    if (!epoch_ok) {
        throw ValueError("B 추종 FC epoch 미확인");
    }
}

void validate_observation_status(const Wire& status, const std::string& uid) {
    if (wire_get(status, "identity_ok") == true) {
        validate_follow_status(status, uid);
        return;
    }
    const Wire epoch = wire_get(status, "fc_epoch");
    const Wire controller = wire_get(status, "controller_session");
    if (wire_get(status, "accepted") != true ||
        !python_equal(wire_get(status, "expected_uid"), Wire(uid)) ||
        wire_get(status, "observation_ready") != true || wire_get(status, "identity_ok") != false ||
        wire_get(status, "onboard_session") != "observation-only" || !epoch.is_number_integer() ||
        !python_equal(epoch, Wire(0)) || !session_text(controller)) {
        throw ValueError("영상 대상 관리 세션 확인 실패");
    }
}

FollowProxy::FollowProxy(std::string endpoint, Transport transport, ClockNs clock, IdSource new_id)
    : endpoint_(std::move(endpoint)),
      transport_(std::move(transport)),
      clock_(clock ? std::move(clock) : ClockNs(steady_ns)),
      new_id_(new_id ? std::move(new_id) : IdSource(random_hex)),
      source_("realfc-gui-" + new_id_()) {}

// 원본 _exchange. 2026-11-25 batch 8: 팀 규약 [follow/command][FollowCommand] 를 보내고
// [follow/command][FollowResult] 를 받는다 (follow_command_protobuf.hpp). 요청마다 새 REQ
// (zmq_request) 와 forward 의 mutex 는 그대로다. 예전 [follow/command][msgpack] ->
// [follow/result][msgpack] 은 없앴다. 받은 응답의 식별자 검사는 예전 그대로다
Wire FollowProxy::exchange(const Wire& request, int timeout_ms) {
    const std::string& name = follow_command_name();
    Bytes body;
    std::vector<std::string> unknown;
    std::string error;
    if (!follow_command_request_body(msgpack_pack(request), body, unknown, error)) {
        // 문구는 Python 원본 follow_proxy._exchange 와 같다 (까닭은 stderr 에만)
        std::cerr << "follow 6373 요청 변환 실패: " << error << '\n';
        throw ValueError("B 추종 요청 변환 실패");
    }
    if (!unknown.empty()) {
        // 지금 forward 가 만드는 키는 모두 proto 에 있다. 새 키가 생기면 한 번씩 남긴다
        static std::mutex unknown_mutex;
        static std::set<std::string> reported;
        const std::scoped_lock lock(unknown_mutex);
        for (const auto& key : unknown) {
            if (reported.insert(key).second) {
                std::cerr << "follow 6373 요청의 proto 에 없는 키를 버림: " << key << '\n';
            }
        }
    }
    std::vector<Bytes> parts;
    try {
        const std::vector<Bytes> sent{Bytes(name.begin(), name.end()), body};
        parts =
            transport_ ? transport_(sent, timeout_ms) : zmq_request(endpoint_, sent, timeout_ms);
    } catch (const TransportError&) {
        throw ValueError(kUnresponsive);
    }
    Bytes packed;
    const FollowReplyFrame frame = follow_command_reply_msgpack(parts, packed, error);
    if (frame == FollowReplyFrame::BadShape) {
        throw ValueError("B 추종 응답 multipart 형식 오류");
    }
    if (frame != FollowReplyFrame::Ok) {
        throw ValueError("B 추종 응답 해석 실패");
    }
    const Wire response = msgpack_unpack(packed);
    if (!response.is_object() || wire_get(response, "schema") != "argus.follow.result.v1" ||
        !python_equal(wire_get(response, "command_id"), request.at("command_id")) ||
        wire_get(response, "source") != "argus-follow-controller") {
        throw ValueError("B 추종 응답 식별자 불일치");
    }
    return response;
}

Wire FollowProxy::forward(const Wire& body, const std::string& expected_uid, int timeout_ms) {
    if (!python_equal(wire_get(body, "expected_uid"), Wire(expected_uid))) {
        throw ValueError("B 추종 FC UID 불일치");
    }
    static const std::vector<std::pair<std::string, std::string>> kActions{
        {"status", "STATUS"},
        {"select", "SELECT"},
        {"start", "START"},
        {"stop", "STOP"},
        {"search", "SCAN_START"},
        {"mode", "SET_MODE"},
        {"prepare_handoff", "PREPARE_HANDOFF"},
        {"reset_input", "RESET_INPUT"}};
    const Wire action_value = wire_get(body, "action");
    std::string action;
    std::string wire_action;
    for (const auto& [name, value] : kActions) {
        if (action_value.is_string() && action_value.get<std::string>() == name) {
            action = name;
            wire_action = value;
        }
    }
    if (action.empty()) {
        throw ValueError("B 추종에서 지원하지 않는 요청: " + python_str_value(action_value));
    }
    const Wire request_id = wire_get(body, "request_id");
    Wire request = Wire::object();
    wire_set(request, "schema", "argus.follow.command.v1");
    wire_set(request, "command_id", wire_truthy(request_id) ? request_id : Wire(new_id_()));
    wire_set(request, "expected_uid", expected_uid);
    wire_set(request, "action", wire_action);
    if (action == "start" || action == "select") {
        const Wire identity = selection_identity(body, action == "start");  // items()가 가리킬 객체
        for (const auto& item : identity.items()) {
            wire_set(request, item.key(), item.value());
        }
    }
    if (action == "search") {
        for (const char* key : {"yaw_range_rad", "yaw_rate_rad_s", "timeout_s"}) {
            if (body.contains(key)) {
                if (!body.at(key).is_number()) {
                    throw ValueError(std::string("Scan 매개변수는 숫자 또는 NaN이어야 합니다: ") +
                                     key);
                }
                wire_set(request, key, body.at(key));
            }
        }
    }
    if (action == "mode") {
        const Wire mode = wire_get(body, "mode");
        if (mode != "moving" && mode != "yaw") {
            throw ValueError("추종 모드 오류");
        }
        wire_set(request, "mode", mode);
    }
    const std::scoped_lock lock(mutex_);
    if (action != "status") {
        Wire status_request = Wire::object();
        wire_set(status_request, "schema", "argus.follow.command.v1");
        wire_set(status_request, "action", "STATUS");
        wire_set(status_request, "command_id", new_id_());
        wire_set(status_request, "expected_uid", expected_uid);
        const Wire status = exchange(status_request, timeout_ms);
        if (action == "select") {
            validate_observation_status(status, expected_uid);
        } else {
            validate_follow_status(status, expected_uid);
        }
        seq_ += 1;
        const std::int64_t now = clock_();
        wire_set(request, "source", source_);
        wire_set(request, "seq", seq_);
        wire_set(request, "clock_domain", "host_monotonic");
        wire_set(request, "issued_monotonic_ns", now);
        wire_set(request, "valid_until_monotonic_ns", now + 900'000'000);
        for (const char* key : {"controller_session", "onboard_session", "fc_epoch"}) {
            wire_set(request, key, status.at(key));
        }
    }
    return exchange(request, timeout_ms);
}

std::vector<Bytes> zmq_request(const std::string& endpoint, const std::vector<Bytes>& parts,
                               int timeout_ms) {
    const Socket socket(ZMQ_REQ);
    if (socket.handle == nullptr) {
        throw TransportError("zmq socket");
    }
    const int linger = 0;
    zmq_setsockopt(socket.handle, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(socket.handle, ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));
    zmq_setsockopt(socket.handle, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
    if (zmq_connect(socket.handle, endpoint.c_str()) != 0) {
        throw TransportError("zmq connect");
    }
    for (std::size_t i = 0; i < parts.size(); ++i) {
        const int flags = i + 1 < parts.size() ? ZMQ_SNDMORE : 0;
        if (zmq_send(socket.handle, parts.at(i).data(), parts.at(i).size(), flags) < 0) {
            throw TransportError("zmq send");
        }
    }
    std::vector<Bytes> received;
    for (;;) {
        zmq_msg_t message;
        zmq_msg_init(&message);
        if (zmq_msg_recv(&message, socket.handle, 0) < 0) {
            zmq_msg_close(&message);
            throw TransportError("zmq receive");
        }
        const auto* data = static_cast<const std::uint8_t*>(zmq_msg_data(&message));
        received.emplace_back(data,
                              std::next(data, static_cast<std::ptrdiff_t>(zmq_msg_size(&message))));
        const bool more = zmq_msg_more(&message) != 0;
        zmq_msg_close(&message);
        if (!more) {
            break;
        }
    }
    return received;
}

}  // namespace argus::gateway
