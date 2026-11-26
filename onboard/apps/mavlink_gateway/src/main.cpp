// mavlink_gateway 실행 파일 (argus_mavlink_gateway)
// 규약: docs/design/uml/class_mavlink_gateway.md 2절, 4절
// 통신 구현(DDS 또는 ZeroMQ, UDP)과 실제 시계를 생성하는 유일한 곳이다. 그 밖의 코드는
// 인터페이스로만 사용한다. 현재 구성은 FC 쪽(FcGateway)만 포함한다. 통제기 쪽(ARGUS_STATUS 송신,
// 통제기 명령 처리)은 다음 작업에서 추가한다. 설정 값은 코드의 기본값(UdpMavlinkConfig,
// protocol/dds_settings.hpp, protocol/zmq_endpoints.hpp)을 사용한다
// 인자
// - --bus dds|zmq: 통신 구현. 기본값 dds. ARGUS_BUILD_BUS_DDS 가 OFF 인 빌드는 zmq 만 사용할 수
// 있다
// - --dds-domain N: DDS domain. 기본값 protocol::kDefaultDdsDomain
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <zmq.hpp>

#include "argus/bus/bus.hpp"
#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/event_loop.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/mavlink_gateway/fc_gateway.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/udp_mavlink_connection.hpp"
#include "argus/protocol/dds_settings.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/protocol/zmq_endpoints.hpp"
#include "argus/vehicle_api_bus/zmq_codecs.hpp"

#ifdef ARGUS_HAS_BUS_DDS
#include "argus/bus_dds/dds_bus.hpp"
#include "argus/vehicle_api_bus/dds_types.hpp"
#endif

namespace {

namespace bus_zmq = argus::bus_zmq;
namespace core = argus::core;
namespace mavlink_gateway = argus::mavlink_gateway;
namespace protocol = argus::protocol;

// core::EventLoop 가 process_events() 한 번에 handler 를 호출하는 최대 시간. 가장 짧은 주기
// 작업(50 ms)이 제때 실행되도록 그보다 짧게 정한다
constexpr std::chrono::milliseconds kHandlerBudget{10};

constexpr const char* kLogPrefix = "argus_mavlink_gateway: ";

constexpr std::string_view kBusOption = "--bus";
constexpr std::string_view kDdsDomainOption = "--dds-domain";
constexpr std::string_view kBusDds = "dds";
constexpr std::string_view kBusZmq = "zmq";

enum class BusKind : std::uint8_t {
    Dds,
    Zmq,
};

struct Options {
    BusKind bus = BusKind::Dds;
    std::uint32_t dds_domain = protocol::kDefaultDdsDomain;
};

// domain 은 0~232 (DDS 의 포트 규칙에서 쓸 수 있는 범위)
std::optional<std::uint32_t> dds_domain_of(const char* text) {
    constexpr std::uint32_t kMaxDomain = 232;
    std::uint32_t value = 0;
    const char* const end = text + std::strlen(text);
    const auto [last, error] = std::from_chars(text, end, value);
    if (error != std::errc{} || last != end || last == text || value > kMaxDomain) {
        return std::nullopt;
    }
    return value;
}

// 명령행 인자를 해석한다. 정의되지 않은 인자나 값이 있으면 값이 없는 optional 을 반환한다
std::optional<Options> parse_options(std::int32_t argc, char** argv) {
    Options options;
    for (std::int32_t i = 1; i < argc; ++i) {
        const std::string_view argument{argv[i]};
        if (argument == kBusOption && i + 1 < argc) {
            const std::string_view value{argv[++i]};
            if (value == kBusDds) {
                options.bus = BusKind::Dds;
            } else if (value == kBusZmq) {
                options.bus = BusKind::Zmq;
            } else {
                return std::nullopt;
            }
        } else if (argument == kDdsDomainOption && i + 1 < argc) {
            const std::optional<std::uint32_t> domain = dds_domain_of(argv[++i]);
            if (!domain.has_value()) {
                return std::nullopt;
            }
            options.dds_domain = domain.value();
        } else {
            return std::nullopt;
        }
    }
    return options;
}

void print_usage() {
    std::cerr << "사용법: argus_mavlink_gateway [" << kBusOption << " " << kBusDds << "|" << kBusZmq
              << "] [" << kDdsDomainOption << " N]\n"
              << "  " << kBusOption << "  통신 구현. 지정하지 않으면 " << kBusDds << "\n"
              << "  " << kDdsDomainOption << "  DDS domain (0~232). 지정하지 않으면 "
              << protocol::kDefaultDdsDomain << "\n";
}

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
                       const mavlink_gateway::UdpMavlinkConfig& udp, const Options& options,
                       const bus_zmq::ZmqEndpointTable& table) {
    std::cout << kLogPrefix << "FC UDP " << udp.local_address << ":" << connection.local_port()
              << "\n";
    if (options.bus == BusKind::Dds) {
        std::cout << kLogPrefix << "bus DDS domain " << options.dds_domain << "\n";
        return;
    }
    for (const auto& [name, endpoint] : table) {
        std::cout << kLogPrefix << name << " " << endpoint << "\n";
    }
}

// options 의 통신 구현으로 bus 를 만든다. 만들지 못하면 원인을 기록하고 nullptr 를 반환한다
std::unique_ptr<argus::bus::IBus> make_bus(const Options& options, zmq::context_t& context,
                                           bus_zmq::ZmqPoller& poller, const core::IClock& clock,
                                           const bus_zmq::ZmqEndpointTable& table) {
    if (options.bus == BusKind::Zmq) {
        return std::make_unique<bus_zmq::ZmqBus>(context, poller, clock, table,
                                                 argus::vehicle_api_bus::zmq_codecs());
    }
#ifdef ARGUS_HAS_BUS_DDS
    // 원인은 DdsBus 가 기록한다
    return argus::bus_dds::DdsBus::create(poller, poller, clock, options.dds_domain,
                                          argus::vehicle_api_bus::dds_types());
#else
    std::cerr << kLogPrefix << "DDS 없이 빌드됨 (ARGUS_BUILD_BUS_DDS=OFF). " << kBusOption << " "
              << kBusZmq << " 로 실행해야 한다\n";
    return nullptr;
#endif
}

std::int32_t run(const Options& options) {
    const core::SteadyClock clock;
    zmq::context_t context;
    bus_zmq::ZmqPoller poller{clock};
    const bus_zmq::ZmqEndpointTable table = endpoints();
    const std::unique_ptr<argus::bus::IBus> bus = make_bus(options, context, poller, clock, table);
    if (!bus) {
        return EXIT_FAILURE;
    }

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
        mavlink_gateway::FcGateway::create(*bus, *connection, clock, loop, health);
    if (!gateway) {
        return EXIT_FAILURE;  // 원인은 bus 구현이 기록한다
    }

    log_configuration(*connection, udp_config, options, table);
    loop.run();
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::optional<Options> options = parse_options(argc, argv);
        if (!options.has_value()) {
            print_usage();
            return EXIT_FAILURE;
        }
        return run(options.value());
    } catch (const std::exception& error) {
        std::cerr << kLogPrefix << error.what() << "\n";
    } catch (...) {
        std::cerr << kLogPrefix << "알 수 없는 예외\n";
    }
    return EXIT_FAILURE;
}
