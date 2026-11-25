// mavlink_gateway 실행 파일 (argus_mavlink_gateway)
// 규약: docs/design/uml/class_mavlink_gateway.md 2절, 4절
// 통신 구현(ZeroMQ, UDP)과 실제 시계를 생성하는 유일한 곳이다. 그 밖의 코드는 인터페이스로만
// 사용한다. 지금 구성은 FC 쪽(FcGateway)만 포함한다. 통제기 쪽(ARGUS_STATUS 송신, 통제기 명령
// 처리)은 다음 작업에서 추가한다. 설정 값은 코드의 기본값(UdpMavlinkConfig,
// protocol/zmq_endpoints.hpp)을 사용한다
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <zmq.hpp>

#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/event_loop.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/mavlink_gateway/fc_gateway.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/udp_mavlink_connection.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/protocol/zmq_endpoints.hpp"
#include "argus/vehicle_api_bus/zmq_codecs.hpp"

namespace {

namespace bus_zmq = argus::bus_zmq;
namespace core = argus::core;
namespace mavlink_gateway = argus::mavlink_gateway;
namespace protocol = argus::protocol;

// core::EventLoop 가 process_events() 한 번에 handler 를 호출하는 최대 시간. 가장 짧은 주기
// 작업(50 ms)이 제때 실행되도록 그보다 짧게 정한다
constexpr std::chrono::milliseconds kHandlerBudget{10};

constexpr const char* kLogPrefix = "argus_mavlink_gateway: ";

// mavlink_gateway 가 쓰는 이름의 ZeroMQ 엔드포인트 (process_interfaces.md 5.1, 5.2).
// fc/* 와 control/mode_request 는 bind, control/setpoint 는 connect 한다 (ZmqBus 가 역할로 정함)
bus_zmq::ZmqEndpointTable endpoints() {
    const std::string fc_state{protocol::kDefaultFcStateEndpoint};
    return {
        {std::string(protocol::kFcAttitude), fc_state},
        {std::string(protocol::kFcState), fc_state},
        {std::string(protocol::kFcModeEvent), fc_state},
        {std::string(protocol::kControlModeRequest),
         std::string(protocol::kDefaultControlModeRequestEndpoint)},
        {std::string(protocol::kControlSetpoint),
         std::string(protocol::kDefaultControlStateEndpoint)},
    };
}

void log_configuration(const mavlink_gateway::UdpMavlinkConnection& connection,
                       const mavlink_gateway::UdpMavlinkConfig& udp,
                       const bus_zmq::ZmqEndpointTable& table) {
    std::cout << kLogPrefix << "FC UDP " << udp.local_address << ":" << connection.local_port()
              << "\n";
    for (const auto& [name, endpoint] : table) {
        std::cout << kLogPrefix << name << " " << endpoint << "\n";
    }
}

std::int32_t run() {
    const core::SteadyClock clock;
    zmq::context_t context;
    bus_zmq::ZmqPoller poller{clock};
    const bus_zmq::ZmqEndpointTable table = endpoints();
    bus_zmq::ZmqBus bus{context, poller, clock, table, argus::vehicle_api_bus::zmq_codecs()};

    const mavlink_gateway::UdpMavlinkConfig udp_config;
    const std::unique_ptr<mavlink_gateway::UdpMavlinkConnection> connection =
        mavlink_gateway::UdpMavlinkConnection::create(poller, udp_config);
    if (!connection) {
        return EXIT_FAILURE;  // 원인은 UdpMavlinkConnection 이 기록한다
    }

    mavlink_gateway::HealthState health;
    core::EventLoop loop{poller, clock, kHandlerBudget};
    core::stop_on_signals(&loop);

    // loop, connection, bus 보다 뒤에 선언하여 먼저 소멸시킨다
    const std::unique_ptr<mavlink_gateway::FcGateway> gateway =
        mavlink_gateway::FcGateway::create(bus, *connection, clock, loop, health);
    if (!gateway) {
        return EXIT_FAILURE;  // 원인은 ZmqBus 가 기록한다
    }

    log_configuration(*connection, udp_config, table);
    loop.run();
    return EXIT_SUCCESS;
}

}  // namespace

int main() {
    try {
        return run();
    } catch (const std::exception& error) {
        std::cerr << kLogPrefix << error.what() << "\n";
    } catch (...) {
        std::cerr << kLogPrefix << "알 수 없는 예외\n";
    }
    return EXIT_FAILURE;
}
