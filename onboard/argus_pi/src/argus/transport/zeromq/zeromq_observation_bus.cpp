// XSUB/XPUB bus의 연결·multipart·HWM 경계를 맡는다. 제어·추적 코드는 ZeroMQ API를 직접 보지 않는다.
#include "argus/transport/zeromq/include/zeromq_observation_bus.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "argus/perception/observations/include/observation_v2_codec.hpp"
#include "argus/transport/zeromq/include/zeromq_operations.hpp"

namespace follow {
namespace {

constexpr std::int64_t kMaxPartBytes = 4 * 1024 * 1024;
constexpr std::size_t kMaxMultipartParts = 16;

ZmqPartReceive receive_part(void* socket, std::string* stored, std::size_t maximum, bool& more,
                            std::string& error) {
    return zmq_receive_part(socket, stored, maximum, more, error, "zmq_msg_init", "zmq_msg_recv",
                            "multipart frame exceeds configured size limit");
}

}  // namespace

ZeroMqContext::ZeroMqContext() : context_(zmq_ctx_new()) {
    if (!context_) {
        throw std::runtime_error(zmq_error("zmq_ctx_new"));
    }
}

ZeroMqContext::~ZeroMqContext() {
    if (context_) {
        zmq_ctx_term(context_);
    }
}

ZeroMqObservationPublisher::ZeroMqObservationPublisher(ZeroMqContext& context,
                                                       const std::string& endpoint,
                                                       BusSocketEndpoint action, int send_hwm) {
    if (send_hwm < 1) {
        throw std::invalid_argument("PUB send HWM must be positive");
    }
    auto socket = zmq_open_socket(context, ZMQ_PUB, "zmq_socket(PUB)");
    const int linger = 0;
    zmq_set_option(socket.get(), ZMQ_LINGER, &linger, sizeof(linger), "set PUB linger");
    zmq_set_option(socket.get(), ZMQ_SNDHWM, &send_hwm, sizeof(send_hwm), "set PUB send HWM");
    zmq_apply_endpoint(socket.get(), endpoint, action, "ZeroMQ endpoint setup");
    socket_ = socket.release();
}

ZeroMqObservationPublisher::~ZeroMqObservationPublisher() {
    if (socket_) {
        zmq_close(socket_);
    }
}

bool ZeroMqObservationPublisher::publish(const ObservationV2& message, std::string& error) {
    try {
        const auto header = ObservationV2Codec::encode(message);
        if (zmq_send(socket_, kObservationsTopic, sizeof(kObservationsTopic) - 1, ZMQ_SNDMORE) <
            0) {
            error = zmq_error("send observation topic");
            return false;
        }
        if (zmq_send(socket_, header.data(), header.size(), 0) < 0) {
            error = zmq_error("send observation MessagePack header");
            return false;
        }
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

ZeroMqObservationSubscriber::ZeroMqObservationSubscriber(ZeroMqContext& context,
                                                         const std::string& endpoint,
                                                         BusSocketEndpoint action,
                                                         int receive_hwm) {
    if (receive_hwm < 1) {
        throw std::invalid_argument("SUB receive HWM must be positive");
    }
    auto socket = zmq_open_socket(context, ZMQ_SUB, "zmq_socket(SUB)");
    const int linger = 0;
    zmq_set_option(socket.get(), ZMQ_LINGER, &linger, sizeof(linger), "set SUB linger");
    zmq_set_option(socket.get(), ZMQ_RCVHWM, &receive_hwm, sizeof(receive_hwm),
                   "set SUB receive HWM");
    // 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 없음 (libzmq 4.3.5 는 넘는 프레임을 보낸 연결을 끊고
    // 다시 맺지 않는다). 부분마다의 상한은 receive 의 receive_part 가 보고 그 메시지만 Invalid
    zmq_set_option(socket.get(), ZMQ_SUBSCRIBE, kObservationsTopic, sizeof(kObservationsTopic) - 1,
                   "subscribe observations topic");
    zmq_apply_endpoint(socket.get(), endpoint, action, "ZeroMQ endpoint setup");
    socket_ = socket.release();
}

ZeroMqObservationSubscriber::~ZeroMqObservationSubscriber() {
    if (socket_) {
        zmq_close(socket_);
    }
}

ObservationBusReceive ZeroMqObservationSubscriber::receive(std::chrono::milliseconds timeout) {
    ObservationBusReceive result;
    const auto bounded_timeout = std::clamp<std::int64_t>(timeout.count(), 0, 60000);
    zmq_pollitem_t item{socket_, 0, ZMQ_POLLIN, 0};
    if (zmq_poll(&item, 1, static_cast<long>(bounded_timeout)) < 0) {
        if (zmq_errno() == EINTR) {
            return result;
        }
        result.state = ObservationBusReceive::State::TransportError;
        result.error = zmq_error("zmq_poll");
        return result;
    }
    if ((item.revents & ZMQ_POLLIN) == 0) {
        return result;
    }

    std::array<std::string, 2> stored_parts;
    std::size_t part_count = 0;
    std::string part_error;
    bool more = true;
    while (more) {
        std::string* stored =
            part_count < stored_parts.size() ? &stored_parts[part_count] : nullptr;
        const auto max_size = part_count == 0   ? std::size_t{128}
                              : part_count == 1 ? ObservationV2Codec::kMaxHeaderBytes
                                                : static_cast<std::size_t>(kMaxPartBytes);
        const auto received = receive_part(socket_, stored, max_size, more, part_error);
        if (received == ZmqPartReceive::TransportError) {
            result.state = ObservationBusReceive::State::TransportError;
            result.error = std::move(part_error);
            return result;
        }
        if (received == ZmqPartReceive::Invalid) {
            // Consume remaining parts of the same multipart message before returning to poll().
            while (more) {
                std::string drain_error;
                if (receive_part(socket_, nullptr, static_cast<std::size_t>(kMaxPartBytes), more,
                                 drain_error) == ZmqPartReceive::TransportError) {
                    result.state = ObservationBusReceive::State::TransportError;
                    result.error = std::move(drain_error);
                    return result;
                }
            }
            result.state = ObservationBusReceive::State::Invalid;
            result.error = std::move(part_error);
            return result;
        }
        ++part_count;
        if (part_count > kMaxMultipartParts) {
            part_error = "multipart message has too many frames";
        }
    }
    if (!part_error.empty() || part_count < 2 || part_count > 3) {
        result.state = ObservationBusReceive::State::Invalid;
        result.error =
            part_error.empty() ? "unexpected multipart frame count" : std::move(part_error);
        return result;
    }
    if (stored_parts[0] != kObservationsTopic) {
        result.state = ObservationBusReceive::State::Invalid;
        result.error = "unexpected bus topic";
        return result;
    }

    const auto decoded = ObservationV2Codec::decode(stored_parts[1]);
    if (!decoded.valid) {
        result.state = ObservationBusReceive::State::Invalid;
        result.error = decoded.error;
        return result;
    }
    result.state = ObservationBusReceive::State::Message;
    result.message = decoded.message;
    return result;
}

}  // namespace follow
