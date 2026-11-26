// system/status PUB의 topic과 MessagePack 전송 경계를 정의한다.
#pragma once

#include <string>

#include "argus/diagnostics/status/include/system_status_v1.hpp"
#include "argus/transport/zeromq/include/zeromq_observation_bus.hpp"

namespace follow {

class ZeroMqSystemStatusPublisher final {
public:
    ZeroMqSystemStatusPublisher(ZeroMqContext&, const std::string& endpoint,
                                BusSocketEndpoint action = BusSocketEndpoint::Connect,
                                int send_hwm = 10);
    ~ZeroMqSystemStatusPublisher();
    ZeroMqSystemStatusPublisher(const ZeroMqSystemStatusPublisher&) = delete;
    ZeroMqSystemStatusPublisher& operator=(const ZeroMqSystemStatusPublisher&) = delete;
    bool publish(const SystemStatusV1&, std::string& error);

private:
    void* socket_{nullptr};
};

inline constexpr char kSystemStatusTopic[] = "system/status";

}  // namespace follow
