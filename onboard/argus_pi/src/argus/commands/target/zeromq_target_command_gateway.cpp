// REQ/REP로 요청 하나당 결과 하나를 돌려주고 selector의 request_id dedupe를 사용한다.
#include "argus/commands/target/include/zeromq_target_command_gateway.hpp"

#include <algorithm>
#include <cerrno>
#include <exception>
#include <stdexcept>
#include <utility>

#include "argus/commands/target/include/target_command_codec.hpp"
#include "argus/transport/zeromq/include/zeromq_operations.hpp"

namespace follow {
namespace {
constexpr std::size_t kMaxCommandBytes = 4096;
constexpr char kMultipartRequestError[] = "command request must contain exactly one frame";
ZmqPartReceive receive_request(void* socket, std::string& request, std::string& error) {
    bool more = false;
    const auto received =
        zmq_receive_part(socket, &request, kMaxCommandBytes, more, error, "command message init",
                         "command receive", "command request exceeds message size limit");
    if (received == ZmqPartReceive::TransportError || !more) {
        return received;
    }
    if (error.empty()) {
        error = kMultipartRequestError;
    }
    while (more) {
        std::string drain_error;
        if (zmq_receive_part(socket, nullptr, kMaxCommandBytes, more, drain_error,
                             "command message init", "command receive",
                             "command request exceeds message size limit") ==
            ZmqPartReceive::TransportError) {
            error = std::move(drain_error);
            return ZmqPartReceive::TransportError;
        }
    }
    return ZmqPartReceive::Invalid;
}
bool send_result(void* socket, const TargetCommandResult& result, std::string& error) {
    try {
        const auto packed = TargetCommandCodec::encode_result(result);
        if (zmq_send(socket, packed.data(), packed.size(), 0) < 0) {
            error = zmq_error("command result send");
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}
}  // namespace

ZeroMqTargetCommandGateway::ZeroMqTargetCommandGateway(
    ZeroMqContext& context, TargetSelector& selector, std::string endpoint,
    BusSocketEndpoint action, std::chrono::milliseconds poll_interval, std::string expected_uid,
    std::function<bool()> identity_verified,
    std::function<TargetCommandResult(const TargetCommand&)> executor)
    : context_(context),
      selector_(selector),
      endpoint_(std::move(endpoint)),
      action_(action),
      poll_interval_(poll_interval),
      expected_uid_(std::move(expected_uid)),
      identity_verified_(std::move(identity_verified)),
      executor_(std::move(executor)) {
    if (endpoint_.empty() || poll_interval_.count() <= 0 ||
        poll_interval_ > std::chrono::seconds(1)) {
        throw std::invalid_argument("invalid target command gateway configuration");
    }
    if (!expected_uid_.empty() && !identity_verified_) {
        throw std::invalid_argument("FC-bound gateway requires live identity verifier");
    }
}
ZeroMqTargetCommandGateway::~ZeroMqTargetCommandGateway() { stop(); }
void ZeroMqTargetCommandGateway::start() {
    std::promise<void> initialized;
    auto ready = initialized.get_future();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (thread_.joinable() || stats_.running) {
            throw std::logic_error("command gateway already started");
        }
        stop_requested_.store(false);
        thread_ = std::thread([this, initialized = std::move(initialized)]() mutable {
            run(std::move(initialized));
        });
    }
    try {
        ready.get();
    } catch (...) {
        stop();
        throw;
    }
}
void ZeroMqTargetCommandGateway::stop() {
    stop_requested_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}
TargetCommandGatewayStats ZeroMqTargetCommandGateway::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

TargetCommandResult ZeroMqTargetCommandGateway::process_request(void* socket) {
    std::string packed, receive_error;
    const auto received = receive_request(socket, packed, receive_error);
    if (received == ZmqPartReceive::TransportError) {
        throw std::runtime_error(receive_error);
    }
    if (received == ZmqPartReceive::Invalid) {
        TargetCommandResult response{"", false, "REJECTED", receive_error};
        if (receive_error == kMultipartRequestError) {
            response.request_id = TargetCommandCodec::decode_request(packed).command.request_id;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.rejected_malformed;
        }
        return response;
    }
    const auto decoded = TargetCommandCodec::decode_request(packed);
    if (decoded.valid) {
        if (!expected_uid_.empty() &&
            (decoded.command.expected_uid != expected_uid_ || !identity_verified_())) {
            return {decoded.command.request_id, false, "REJECTED",
                    "expected_fc_uid_or_live_identity_mismatch"};
        }
        return executor_ ? executor_(decoded.command)
                         : selector_.execute(decoded.command, Clock::now());
    }
    TargetCommandResult response{decoded.command.request_id, false, "REJECTED", decoded.error};
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.rejected_malformed;
    return response;
}

void ZeroMqTargetCommandGateway::run(std::promise<void> initialized) {
    try {
        // REP 소켓은 이 worker thread에서만 사용한다.
        auto socket = zmq_open_socket(context_, ZMQ_REP, "command REP socket");
        const int linger = 0;
        const int hwm = 8;
        zmq_set_option(socket.get(), ZMQ_LINGER, &linger, sizeof(linger), "command REP linger");
        zmq_set_option(socket.get(), ZMQ_RCVHWM, &hwm, sizeof(hwm), "command REP receive HWM");
        zmq_set_option(socket.get(), ZMQ_SNDHWM, &hwm, sizeof(hwm), "command REP send HWM");
        zmq_apply_endpoint(socket.get(), endpoint_, action_, "command endpoint");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.running = true;
        }
        initialized.set_value();
        while (!stop_requested_.load()) {
            zmq_pollitem_t item{socket.get(), 0, ZMQ_POLLIN, 0};
            const auto poll_ms = std::clamp<std::int64_t>(poll_interval_.count(), 0, 1000);
            if (zmq_poll(&item, 1, static_cast<long>(poll_ms)) < 0) {
                if (zmq_errno() == EINTR) {
                    continue;
                }
                throw std::runtime_error(zmq_error("command REP poll"));
            }
            if (!(item.revents & ZMQ_POLLIN)) {
                continue;
            }
            const auto response = process_request(socket.get());
            std::string send_error;
            if (!send_result(socket.get(), response, send_error)) {
                throw std::runtime_error(send_error);
            }
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.requests;
        }
    } catch (const std::exception& exception) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.transport_errors;
        stats_.last_error = exception.what();
        if (!stats_.running) {
            initialized.set_exception(std::current_exception());
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.running = false;
}

bool ZeroMqTargetCommandClient::request(const std::string& endpoint, BusSocketEndpoint action,
                                        const TargetCommand& command, TargetCommandResult& result,
                                        std::string& error, std::chrono::milliseconds timeout) {
    if (endpoint.empty() || timeout.count() <= 0 || timeout > std::chrono::seconds(30)) {
        error = "invalid command client settings";
        return false;
    }
    try {
        auto socket = zmq_open_socket(context_, ZMQ_REQ, "command REQ socket");
        const int linger = 0;
        const auto timeout_ms = static_cast<int>(timeout.count());
        zmq_set_option(socket.get(), ZMQ_LINGER, &linger, sizeof(linger), "command REQ linger");
        zmq_set_option(socket.get(), ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms),
                       "command REQ send timeout");
        zmq_set_option(socket.get(), ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms),
                       "command REQ receive timeout");
        zmq_apply_endpoint(socket.get(), endpoint, action, "command endpoint");
        const auto packed = TargetCommandCodec::encode_request(command);
        if (zmq_send(socket.get(), packed.data(), packed.size(), 0) < 0) {
            throw std::runtime_error(zmq_error("command REQ send"));
        }
        zmq_msg_t message;
        if (zmq_msg_init(&message) != 0) {
            throw std::runtime_error(zmq_error("command result init"));
        }
        const auto count = zmq_msg_recv(&message, socket.get(), 0);
        if (count < 0) {
            const auto message_error = zmq_error("command result receive");
            zmq_msg_close(&message);
            throw std::runtime_error(message_error);
        }
        if (zmq_msg_size(&message) > kMaxCommandBytes) {
            zmq_msg_close(&message);
            throw std::runtime_error("command result exceeds message size limit");
        }
        const auto* data = static_cast<const char*>(zmq_msg_data(&message));
        std::string packed_result(data, zmq_msg_size(&message));
        zmq_msg_close(&message);
        if (!TargetCommandCodec::decode_result(packed_result, result, error)) {
            throw std::runtime_error(error);
        }
        if (result.request_id != command.request_id) {
            throw std::runtime_error("command result request_id mismatch");
        }
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}
}  // namespace follow
