// 상태는 관찰 전용이며 PX4 명령을 대신하거나 전달 성공을 주장하지 않는다.
#include "argus/diagnostics/status/include/zeromq_system_status_publisher.hpp"

#include <stdexcept>

#include "argus/diagnostics/status/include/system_status_v1_codec.hpp"
#include "argus/transport/zeromq/include/zeromq_operations.hpp"

namespace follow {
ZeroMqSystemStatusPublisher::ZeroMqSystemStatusPublisher(ZeroMqContext& context,
                                                         const std::string& address,
                                                         BusSocketEndpoint action, int send_hwm) {
    if (address.empty() || send_hwm < 1) {
        throw std::invalid_argument("invalid status publisher config");
    }
    auto socket = zmq_open_socket(context, ZMQ_PUB, "status PUB socket");
    const int linger = 0;
    zmq_set_option(socket.get(), ZMQ_LINGER, &linger, sizeof(linger), "status PUB linger");
    zmq_set_option(socket.get(), ZMQ_SNDHWM, &send_hwm, sizeof(send_hwm), "status PUB HWM");
    zmq_apply_endpoint(socket.get(), address, action, "status endpoint");
    socket_ = socket.release();
}

ZeroMqSystemStatusPublisher::~ZeroMqSystemStatusPublisher() {
    if (socket_) {
        zmq_close(socket_);
    }
}

bool ZeroMqSystemStatusPublisher::publish(const SystemStatusV1& message, std::string& error) {
    try {
        const auto packed = SystemStatusV1Codec::encode(message);
        if (zmq_send(socket_, kSystemStatusTopic, sizeof(kSystemStatusTopic) - 1, ZMQ_SNDMORE) <
                0 ||
            zmq_send(socket_, packed.data(), packed.size(), 0) < 0) {
            error = zmq_error("status PUB send");
            return false;
        }
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}
}  // namespace follow
