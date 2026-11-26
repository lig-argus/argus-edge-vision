#include "argus/runtime/follow/include/follow_operator_gateway.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

#include "argus/common/identity/include/float_identity.hpp"
#include "argus/common/serialization/include/message_pack.hpp"
#include "argus/common/time/include/time.hpp"
#include "argus/control/contracts/include/command_outcome.hpp"
#include "argus/transport/team_wire/include/pi_topics.hpp"
#include "argus/transport/team_wire/include/team_wire.hpp"

namespace follow {
namespace {
void validate_map_keys(const std::string& bytes) {
    std::size_t offset = 0;
    const auto unpacked = msgpack::unpack(bytes.data(), bytes.size(), offset);
    const auto& root = unpacked.get();
    if (offset != bytes.size() || root.type != msgpack::type::MAP) {
        throw std::invalid_argument("command must be exactly one MessagePack map");
    }
    std::set<std::string> keys;
    for (std::uint32_t i = 0; i < root.via.map.size; ++i) {
        const auto key = msgpack_read_string(root.via.map.ptr[i].key, 128);
        if (!keys.insert(key).second) {
            throw std::invalid_argument("duplicate command field");
        }
    }
}

std::string required_text(const NativeJson& request, const char* key) {
    const auto& value = request.at(key);
    if (!value.is_string()) {
        throw std::invalid_argument(std::string(key) + " must be a string");
    }
    auto text = value.get<std::string>();
    if (text.empty() || text.size() > 128) {
        throw std::invalid_argument(std::string(key) + " is empty or too long");
    }
    return text;
}

std::uint64_t required_u64(const NativeJson& request, const char* key,
                           std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max()) {
    const auto& value = request.at(key);
    if ((!value.is_number_unsigned() && !value.is_number_integer()) ||
        (value.is_number_integer() && !value.is_number_unsigned() &&
         value.get<std::int64_t>() < 0)) {
        throw std::invalid_argument(std::string(key) + " must use nonnegative integer storage");
    }
    const auto result = value.get<std::uint64_t>();
    if (result > maximum) {
        throw std::invalid_argument(std::string(key) + " exceeds integer range");
    }
    return result;
}

std::string command_id_text(const NativeJson& request) {
    if (request.at("command_id").is_string()) {
        return required_text(request, "command_id");
    }
    return std::to_string(required_u64(request, "command_id"));
}

NativeJson normalize_follow(const NativeJson& wire,
                            const std::map<std::uint32_t, std::string>& sensor_sources) {
    const bool v2 = wire.at("schema") == "argus.follow.command.v2";
    auto input = wire;
    if (!v2 && input.at("schema") != "argus.follow.command.v1") {
        throw std::invalid_argument("invalid follow command schema");
    }
    const std::set<std::string> allowed{"schema",
                                        "source",
                                        "seq",
                                        "command_id",
                                        "action",
                                        "expected_uid",
                                        "sensor",
                                        "generation",
                                        "class_id",
                                        "label",
                                        "track_id",
                                        "controller_session",
                                        "onboard_session",
                                        "fc_epoch",
                                        "issued_monotonic_ns",
                                        "valid_until_monotonic_ns",
                                        "yaw_range_rad",
                                        "yaw_rate_rad_s",
                                        "timeout_s",
                                        "mode",
                                        "reason",
                                        "clock_domain",
                                        "session_id"};
    for (auto it = input.begin(); it != input.end();) {
        if (!allowed.count(it.key())) {
            if (v2) {
                it = input.erase(it);
                continue;
            }
            throw std::invalid_argument("unknown follow command field: " + it.key());
        }
        ++it;
    }
    if (v2 && input.value("action", "") != "STATUS") {
        const auto session = required_text(input, "session_id");
        if (input.contains("onboard_session") && input.at("onboard_session") != session) {
            throw std::invalid_argument("onboard session alias mismatch");
        }
        input["onboard_session"] = session;
    }
    NativeJson result = input;
    result["schema"] = "argus.vmy.request.v1";
    result["request_id"] = command_id_text(input);
    const auto action = required_text(input, "action");
    required_text(input, "expected_uid");
    if (action == "STATUS") {
        result["action"] = "status";
        return result;
    }
    required_text(input, "source");
    required_text(input, "controller_session");
    required_text(input, "onboard_session");
    required_u64(input, "fc_epoch");
    required_u64(input, "seq");
    if (input.contains("clock_domain") && input.at("clock_domain") != "host_monotonic") {
        throw std::invalid_argument("follow command requires host_monotonic");
    }
    if (action == "START" || action == "SELECT") {
        result["action"] = action == "SELECT" ? "select" : "track_start";
        if (action == "SELECT") {
            required_text(input, "label");
        }
        if (input.at("sensor").is_string()) {
            required_text(input, "sensor");
        } else {
            const auto sensor = required_u64(input, "sensor", 255);
            const auto found = sensor_sources.find(static_cast<std::uint32_t>(sensor));
            if ((sensor != 1 && sensor != 2) || found == sensor_sources.end()) {
                throw std::invalid_argument("sensor number has no configured source mapping");
            }
            result["sensor"] = found->second;
        }
        // ICD의 숫자 식별자는 label을 싣지 않는다. 최신 트랙 조회와 재검증은 core에서 한다.
        if (input.contains("label")) {
            required_text(input, "label");
        }
        if (required_u64(input, "generation", kFloatIdentityMax) == 0) {
            throw std::invalid_argument("generation must be positive");
        }
        required_u64(input, "class_id", kFloatIdentityMax);
        if (required_u64(input, "track_id", kFloatIdentityMax) == 0) {
            throw std::invalid_argument("track_id must be positive");
        }
    } else if (action == "STOP") {
        result["action"] = "stop";
    } else if (action == "SCAN_START") {
        result["action"] = "search";
        for (const char* key : {"yaw_range_rad", "yaw_rate_rad_s", "timeout_s"}) {
            if (input.contains(key) && !input.at(key).is_number()) {
                throw std::invalid_argument(std::string(key) + " must be numeric or NaN");
            }
        }
    } else if (action == "SET_MODE") {
        result["action"] = "mode";
        const auto mode = required_text(input, "mode");
        if (mode != "moving" && mode != "yaw") {
            throw std::invalid_argument("mode must be moving or yaw");
        }
    } else if (action == "PREPARE_HANDOFF") {
        result["action"] = "prepare_handoff";
    } else if (action == "RESET_INPUT") {
        result["action"] = "reset_input";
    } else {
        throw std::invalid_argument("unsupported follow command action");
    }
    return result;
}

void check_admission(const NativeJson& request) {
    const auto issued =
        required_u64(request, "issued_monotonic_ns", std::numeric_limits<std::int64_t>::max());
    const auto until =
        required_u64(request, "valid_until_monotonic_ns", std::numeric_limits<std::int64_t>::max());
    const auto now = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
            .count());
    if (issued == 0 || issued > now || until <= now || until <= issued ||
        until - issued > 1000000000ULL) {
        throw std::invalid_argument("follow command admission expired, future, or exceeds 1000ms");
    }
}

void merge_result(NativeJson& response, const NativeJson& result) {
    if (!result.is_object()) {
        throw std::invalid_argument("follow command handler result must be a map");
    }
    for (auto it = result.begin(); it != result.end(); ++it) {
        response[it.key()] = it.value();
    }
}
}  // namespace

VMyOperatorGateway::VMyOperatorGateway(ZeroMqContext& context, const std::string& endpoint,
                                       bool allow_follow_commands,
                                       std::function<bool(const NativeJson&)> permit,
                                       std::map<std::uint32_t, std::string> sensor_sources)
    : socket_(zmq_open_socket(context, ZMQ_REP, "vehicle gateway REP")),
      allow_follow_commands_(allow_follow_commands),
      permit_(std::move(permit)),
      sensor_sources_(std::move(sensor_sources)) {
    for (const auto& item : sensor_sources_) {
        if ((item.first != 1 && item.first != 2) || item.second.empty() ||
            item.second.size() > 128) {
            throw std::invalid_argument("invalid configured sensor source");
        }
    }
    if (allow_follow_commands_) {
        team_wire_ = std::make_unique<argus::team_wire::TeamWire>();
    }
    const int linger = 0;
    zmq_set_option(socket_.get(), ZMQ_LINGER, &linger, sizeof(linger), "vehicle gateway linger");
    zmq_apply_endpoint(socket_.get(), endpoint, BusSocketEndpoint::Bind, "vehicle gateway bind");
}

VMyOperatorGateway::~VMyOperatorGateway() = default;

void VMyOperatorGateway::poll(const std::function<NativeJson(const NativeJson&)>& handler) {
    FunctionalFollowCommand command{handler};
    poll(command);
}

void VMyOperatorGateway::poll(IFollowCommand& handler) {
    zmq_pollitem_t item{socket_.get(), 0, ZMQ_POLLIN, 0};
    if (zmq_poll(&item, 1, 0) < 0) {
        throw std::runtime_error("vehicle gateway poll failed");
    }
    if ((item.revents & ZMQ_POLLIN) == 0) {
        return;
    }
    if (allow_follow_commands_) {
        serve_team(handler);
        return;
    }
    // B 가 아닌 6363 경로: 예전 msgpack (한 프레임 argus.vmy.request.v1, follow/command 는 거부)
    std::string bytes, error;
    bool more = false;
    auto received = zmq_receive_part(socket_.get(), &bytes, 4096, more, error, "vehicle init",
                                     "vehicle receive", "vehicle request too large");
    const bool follow_wire = more && bytes == "follow/command";
    if (follow_wire) {
        received = zmq_receive_part(socket_.get(), &bytes, 4096, more, error, "follow init",
                                    "follow receive", "follow request too large");
    }
    while (more) {
        std::string ignored;
        zmq_receive_part(socket_.get(), nullptr, 4096, more, ignored, "vehicle drain init",
                         "vehicle drain", "vehicle drain size");
        if (error.empty()) {
            error = "invalid command multipart shape";
        }
    }
    NativeJson input;
    std::string failure;
    try {
        if (received != ZmqPartReceive::Consumed || !error.empty()) {
            throw std::runtime_error(error.empty() ? "vehicle receive failed" : error);
        }
        validate_map_keys(bytes);
        input = NativeJson::from_msgpack(bytes.begin(), bytes.end());
        if (!input.is_object()) {
            throw std::invalid_argument("command must be a map");
        }
    } catch (const std::exception& e) {
        failure = e.what();
        if (failure.empty()) {
            failure = "vehicle request rejected";
        }
    }
    const auto response = respond(input, follow_wire, handler, failure);
    if (follow_wire) {
        if (zmq_send(socket_.get(), "follow/result", 13, ZMQ_SNDMORE) < 0) {
            throw std::runtime_error("follow result topic send failed");
        }
    }
    auto packed = NativeJson::to_msgpack(response);
    if (zmq_send(socket_.get(), packed.data(), packed.size(), 0) < 0) {
        throw std::runtime_error("vehicle result send failed");
    }
}

// B: 팀 [follow/command][FollowCommand] 하나를 받아 [follow/command][FollowResult] 로 답한다.
// 팀 ZmqReplier 대신 team_wire serve_one 을 쓰는 까닭: 이 REP 는 추종 제어 thread 의 tick 에서
// dontwait 로 읽고 (팀 ZmqPoller 는 따로 돈다), 받은 뒤의 검사가 모두 respond 안에 있다. 해석하지
// 못한 요청과 respond 밖으로 나온 예외는 serve_one 이 [follow/command][빈 본문] 으로 답한다.
// respond 밖으로 나온 예외는 handler 가 이미 실행됐을 수 있으므로 stderr 에 남긴다 (2026-11-25
// batch 8 리뷰). 응답 송신 실패는 예전처럼 예외로 알린다
void VMyOperatorGateway::serve_team(IFollowCommand& handler) {
    const zmq::socket_ref socket(zmq::from_handle, socket_.get());
    const auto encode_before = team_wire_->stats().encode_failures;
    const auto send_before = team_wire_->stats().send_failures;
    const auto handler_before = team_wire_->stats().handler_failures;
    const auto status = team_wire_->serve_one(
        socket, argus::team_wire::kFollowCommand,
        [&](const NativeJson& input) { return respond(input, true, handler, {}); },
        zmq::recv_flags::dontwait);
    if (status != argus::team_wire::ReceiveStatus::Ok &&
        status != argus::team_wire::ReceiveStatus::NoMessage) {
        const std::uint64_t count = ++team_rejects_;
        if ((count & (count - 1)) == 0) {
            std::cerr << "follow 6373 요청 거부 (누적 " << count
                      << "): " << argus::team_wire::to_string(status) << ' '
                      << team_wire_->last_error() << '\n';
        }
    }
    if (team_wire_->stats().encode_failures != encode_before) {
        const std::uint64_t count = ++team_encode_failures_;
        if ((count & (count - 1)) == 0) {
            std::cerr << "follow 6373 응답 변환 실패, 빈 본문으로 답함 (누적 " << count
                      << "): " << team_wire_->last_error() << '\n';
        }
    }
    if (team_wire_->stats().handler_failures != handler_before) {
        const std::uint64_t count = ++team_handler_failures_;
        if ((count & (count - 1)) == 0) {
            std::cerr << "follow 6373 응답 구성 예외, 빈 본문으로 답함 (누적 " << count
                      << "): " << team_wire_->last_error() << '\n';
        }
    }
    if (team_wire_->stats().send_failures != send_before) {
        throw std::runtime_error("follow result send failed");
    }
}

NativeJson VMyOperatorGateway::respond(const NativeJson& input, bool follow_wire,
                                       IFollowCommand& handler, const std::string& failure) {
    NativeJson response = {
        {"schema", follow_wire ? "argus.follow.result.v1" : "argus.vmy.result.v1"},
        {"accepted", false},
        {"request_id", ""},
        {"reason", ""}};
    try {
        if (!failure.empty()) {
            throw std::invalid_argument(failure);
        }
        if (!input.is_object()) {
            throw std::invalid_argument("command must be a map");
        }
        if (follow_wire && !allow_follow_commands_) {
            throw std::invalid_argument("follow command transport disabled");
        }
        auto request = follow_wire ? normalize_follow(input, sensor_sources_) : input;
        if (request.at("schema") != "argus.vmy.request.v1") {
            throw std::invalid_argument("invalid vehicle request schema");
        }
        const auto id = required_text(request, "request_id");
        response["request_id"] = id;
        const bool status = request.value("action", "") == "status";
        // In B mode the legacy single-part envelope must not bypass the same
        // admission/session boundary. SDK/native two-argument mode stays compatible.
        // (2026-11-25 batch 8: B 는 한 프레임 요청을 받지 않으므로 B 에서는 follow_wire 뿐이다)
        if (!status && (follow_wire || allow_follow_commands_)) {
            check_admission(request);
            if (!permit_ || !permit_(request)) {
                throw std::invalid_argument(
                    "current onboard/controller session or FC epoch rejected");
            }
        } else if (!status && permit_ && !permit_(request)) {
            throw std::invalid_argument("current backend command context rejected");
        }
        // Canonical MessagePack preserves NaN versus null, unlike JSON dump().
        auto semantic_request =
            input.value("schema", "") == "argus.follow.command.v2" ? request : input;
        semantic_request["schema"] = input.at("schema");
        if (follow_wire) {
            // 재전송은 onboard가 새 내부 기한/seq를 붙여도 같은 작업이다.
            // 사용자 인수, 세션, NaN/null의 구분은 fingerprint에 그대로 남긴다.
            for (const auto* key : {"seq", "issued_monotonic_ns", "valid_until_monotonic_ns"}) {
                semantic_request.erase(key);
            }
        }
        const auto packed_request = NativeJson::to_msgpack(semantic_request);
        const std::string encoded(packed_request.begin(), packed_request.end());
        const auto cache_id = std::string(follow_wire ? "b:" : "legacy:") + id;
        auto old = previous_.find(cache_id);
        if (old != previous_.end() && old->second.first != encoded) {
            throw std::invalid_argument("vehicle request ID reused with different payload");
        }
        if ((follow_wire || allow_follow_commands_) && !status) {
            const SequenceKey key{
                required_text(request, "source"), required_text(request, "controller_session"),
                required_text(request, "onboard_session"), required_u64(request, "fc_epoch")};
            const auto seq = required_u64(request, "seq");
            auto sequence = sequences_.find(key);
            if (sequence != sequences_.end()) {
                if (old != previous_.end() && sequence->second.second != cache_id) {
                    throw std::invalid_argument("command superseded; late retry rejected");
                }
                if (seq < sequence->second.first ||
                    (seq == sequence->second.first &&
                     (sequence->second.second != cache_id || old == previous_.end()))) {
                    throw std::invalid_argument("follow command sequence is stale or reused");
                }
                sequence->second = {seq, cache_id};
            } else {
                if (sequences_.size() >= 256) {
                    throw std::invalid_argument("follow command session limit reached");
                }
                sequences_.emplace(key, std::make_pair(seq, cache_id));
            }
        }
        // STATUS is freshly evaluated so bootstrap reports current sessions.
        if (!status && old != previous_.end()) {
            response = old->second.second;
            response["request_accepted"] = response.value("accepted", false) ||
                                           response.value("pending", false) ||
                                           response.value("execution_complete", false);
            if (follow_wire && response.value("pending", false)) {
                auto query = request;
                query["action"] = "status";
                const auto current = handler.execute(query);
                if (current.contains("command_results") &&
                    current.at("command_results").is_array()) {
                    for (const auto& row : current.at("command_results")) {
                        if (row.at("command_id") == input.at("command_id") &&
                            !row.at("pending").get<bool>()) {
                            response["accepted"] = row.at("accepted");
                            response["pending"] = false;
                            response["execution_complete"] = true;
                            response["request_accepted"] = true;
                            response["reason"] = row.at("reason");
                            old->second.second = response;
                            break;
                        }
                    }
                }
            }
        } else {
            merge_result(response, handler.execute(request));
            if (!status) {
                if (previous_.size() >= 256) {
                    previous_.erase(previous_.begin());
                }
                previous_[cache_id] = std::make_pair(encoded, response);
            }
        }
    } catch (const std::exception& e) {
        response["accepted"] = false;
        if (follow_wire) {
            response["pending"] = false;
        }
        response["reason"] = e.what();
    }
    if (follow_wire) {
        response["schema"] = "argus.follow.result.v1";
        response["source"] = "argus-follow-controller";
        response["seq"] = ++reply_sequence_;
        response["clock_domain"] = "host_monotonic";
        response["published_monotonic_ns"] =
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count();
        for (const char* key :
             {"command_id", "onboard_session", "fc_epoch", "controller_session"}) {
            // STATUS가 반환한 현재 세션을 요청의 빈/이전 세션으로 덮어쓰지 않는다.
            if (!response.contains(key) && input.is_object() && input.contains(key)) {
                response[key] = input.at(key);
            }
        }
        const bool pending = response.value("pending", false);
        const bool accepted = response.value("accepted", false);
        response["pending"] = pending;
        response["failed"] = !accepted && !pending;
        add_command_outcome(response);
        response["result"] = pending ? "pending" : accepted ? "accepted" : "failed";
        if (input.is_object() && input.contains("schema") &&
            input.at("schema") == "argus.follow.command.v2") {
            response["schema"] = "argus.follow.result.v2";
            response["requester_session_id"] =
                input.contains("session_id") && input.at("session_id").is_string()
                    ? input.at("session_id")
                    : NativeJson("");
            response["session_id"] = accepted ? response.value("controller_session", "") : "";
            response["execution_result"] = pending    ? "IN_PROGRESS"
                                           : accepted ? "ACCEPTED"
                                                      : "FAILED";
            response["result"] = accepted                                  ? "ACCEPTED"
                                 : response.contains("execution_complete") ? "FAILED"
                                                                           : "REJECTED";
        }
    }
    return response;
}
}  // namespace follow
