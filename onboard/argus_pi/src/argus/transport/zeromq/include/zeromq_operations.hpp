// ZeroMQ adapter의 오류·옵션·endpoint 연산을 공유한다. socket 수명과 수신 계약은 adapter가 맡는다.
#pragma once

#include <zmq.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include "argus/transport/zeromq/include/zeromq_observation_bus.hpp"

namespace follow {

inline std::string zmq_error(const char* operation) {
    return std::string(operation) + ": " + zmq_strerror(zmq_errno());
}

enum class ZmqPartReceive { Consumed, Invalid, TransportError };

inline ZmqPartReceive zmq_receive_part(void* socket, std::string* stored, std::size_t maximum,
                                       bool& more, std::string& error, const char* init_operation,
                                       const char* receive_operation, const char* size_error) {
    more = false;
    zmq_msg_t part;
    if (zmq_msg_init(&part) != 0) {
        error = zmq_error(init_operation);
        return ZmqPartReceive::TransportError;
    }
    const int received = zmq_msg_recv(&part, socket, 0);
    if (received < 0) {
        error = zmq_error(receive_operation);
        zmq_msg_close(&part);
        return ZmqPartReceive::TransportError;
    }
    const auto size = zmq_msg_size(&part);
    more = zmq_msg_more(&part) != 0;
    if (size > maximum) {
        error = size_error;
    } else if (stored) {
        stored->assign(static_cast<const char*>(zmq_msg_data(&part)), size);
    }
    zmq_msg_close(&part);
    return error.empty() ? ZmqPartReceive::Consumed : ZmqPartReceive::Invalid;
}

// 생성/설정 실패와 worker 종료 때 표준 unique_ptr가 socket을 닫는다.
using ZmqSocket = std::unique_ptr<void, decltype(&zmq_close)>;
inline ZmqSocket zmq_open_socket(ZeroMqContext& context, int type, const char* operation) {
    ZmqSocket socket(zmq_socket(context.native_handle(), type), &zmq_close);
    if (!socket) {
        throw std::runtime_error(zmq_error(operation));
    }
    return socket;
}

inline void zmq_set_option(void* socket, int option, const void* value, std::size_t size,
                           const char* operation) {
    if (zmq_setsockopt(socket, option, value, size) != 0) {
        throw std::runtime_error(zmq_error(operation));
    }
}

inline void zmq_apply_endpoint(void* socket, const std::string& address, BusSocketEndpoint action,
                               const char* operation) {
    const auto result = action == BusSocketEndpoint::Connect ? zmq_connect(socket, address.c_str())
                                                             : zmq_bind(socket, address.c_str());
    if (result != 0) {
        throw std::runtime_error(zmq_error(operation));
    }
}

}  // namespace follow
