// mavlink_gateway 실행 파일 (argus_mavlink_gateway)
// 규약: docs/design/uml/class_mavlink_gateway.md 2절, 4절
// 통신 구현(DDS 또는 ZeroMQ, UDP)과 실제 시계를 생성하는 유일한 곳이다. 그 밖의 코드는
// 인터페이스로만 사용한다. 구성은 FC 쪽(FcGateway)과 통제기 쪽(통제기 연결, 제어권, 명령 수신, 두절
// 감시, 요청 메시지 응답, FC 전달, 상태 송신, 추종 명령)이다. 설정 값은 코드의 기본값
// (UdpMavlinkConfig, UdpGcsConfig, protocol/dds_settings.hpp, protocol/zmq_endpoints.hpp)을
// 사용한다.
// 인자
// - --bus dds|zmq: 통신 구현. 기본값 dds. ARGUS_BUILD_BUS_DDS 가 OFF 인 빌드는 zmq 만 사용할 수
// 있다
// - --dds-domain N: DDS domain. 기본값 protocol::kDefaultDdsDomain
// - --gcs-port N: 통제기 UDP 포트. 기본값 14550 (icd 1.2). 같은 컴퓨터에서 다른 프로그램이 14550 을
//   사용할 때 다른 포트를 지정한다
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <zmq.hpp>

#include "argus/bus/bus.hpp"
#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/event_loop.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/mavlink_gateway/argus_sensor_status_reporter.hpp"
#include "argus/mavlink_gateway/argus_status_reporter.hpp"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/fc_gateway.hpp"
#include "argus/mavlink_gateway/fc_telemetry_relay.hpp"
#include "argus/mavlink_gateway/follow_command_tracker.hpp"
#include "argus/mavlink_gateway/gcs_fc_forwarder.hpp"
#include "argus/mavlink_gateway/gcs_link_monitor.hpp"
#include "argus/mavlink_gateway/gcs_registry.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/heartbeat_sender.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/latest_tracks.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/mavlink_gateway/request_message_handler.hpp"
#include "argus/mavlink_gateway/request_operator_control_handler.hpp"
#include "argus/mavlink_gateway/scan_start_handler.hpp"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/mavlink_gateway/sysfs_cpu_temperature.hpp"
#include "argus/mavlink_gateway/track_start_handler.hpp"
#include "argus/mavlink_gateway/track_stop_handler.hpp"
#include "argus/mavlink_gateway/udp_gcs_link.hpp"
#include "argus/mavlink_gateway/udp_mavlink_connection.hpp"
#include "argus/mavlink_gateway/video_stream_information.hpp"
#include "argus/perception_api_bus/bus_tracking_status_subscriber.hpp"
#include "argus/perception_api_bus/bus_vision_status_subscriber.hpp"
#include "argus/perception_api_bus/zmq_codecs.hpp"
#include "argus/protocol/dds_settings.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/protocol/zmq_endpoints.hpp"
#include "argus/vehicle_api_bus/bus_follow_control_client.hpp"
#include "argus/vehicle_api_bus/bus_follow_status_subscriber.hpp"
#include "argus/vehicle_api_bus/zmq_codecs.hpp"
#include "argus/video_api_bus/bus_video_stream_info_subscriber.hpp"
#include "argus/video_api_bus/zmq_codecs.hpp"

#ifdef ARGUS_HAS_BUS_DDS
#include "argus/bus_dds/dds_bus.hpp"
#include "argus/perception_api_bus/dds_types.hpp"
#include "argus/vehicle_api_bus/dds_types.hpp"
#include "argus/video_api_bus/dds_types.hpp"
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
constexpr std::string_view kGcsPortOption = "--gcs-port";
constexpr std::string_view kBusDds = "dds";
constexpr std::string_view kBusZmq = "zmq";

enum class BusKind : std::uint8_t {
    Dds,
    Zmq,
};

struct Options {
    BusKind bus = BusKind::Dds;
    std::uint32_t dds_domain = protocol::kDefaultDdsDomain;
    std::uint16_t gcs_port = mavlink_gateway::kGcsUdpPort;
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

// 1~65535
std::optional<std::uint16_t> port_of(const char* text) {
    std::uint16_t value = 0;
    const char* const end = text + std::strlen(text);
    const auto [last, error] = std::from_chars(text, end, value);
    if (error != std::errc{} || last != end || last == text || value == 0) {
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
        } else if (argument == kGcsPortOption && i + 1 < argc) {
            const std::optional<std::uint16_t> port = port_of(argv[++i]);
            if (!port.has_value()) {
                return std::nullopt;
            }
            options.gcs_port = port.value();
        } else {
            return std::nullopt;
        }
    }
    return options;
}

void print_usage() {
    std::cerr << "사용법: argus_mavlink_gateway [" << kBusOption << " " << kBusDds << "|" << kBusZmq
              << "] [" << kDdsDomainOption << " N] [" << kGcsPortOption << " N]\n"
              << "  " << kBusOption << "  통신 구현. 지정하지 않으면 " << kBusDds << "\n"
              << "  " << kDdsDomainOption << "  DDS domain (0~232). 지정하지 않으면 "
              << protocol::kDefaultDdsDomain << "\n"
              << "  " << kGcsPortOption << "  통제기 UDP 포트 (1~65535). 지정하지 않으면 "
              << mavlink_gateway::kGcsUdpPort << "\n";
}

// mavlink_gateway 의 세션. 시작할 때마다 새로 만든다 (process_interfaces.md 3장 session_id).
// 다시 시작한 mavlink_gateway 를 추종 제어가 구분하는 데만 사용하므로 난수 64 비트의 16 진수로 한다
std::string make_session_id() {
    std::random_device device;
    const std::uint64_t value = (static_cast<std::uint64_t>(device()) << 32U) | device();
    std::ostringstream text;
    text << "mavlink_gateway-" << std::hex << value;
    return text.str();
}

// mavlink_gateway 가 쓰는 이름의 ZeroMQ 엔드포인트 (process_interfaces.md 5.1, 5.2).
// fc/* 와 control/mode_request 는 bind, control/setpoint, follow/status, follow/command,
// video/stream_info 는 connect 한다 (ZmqBus 가 역할로 정함)
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
        {std::string(protocol::kFollowStatus), std::string(protocol::kDefaultControlStateEndpoint)},
        {std::string(protocol::kFollowCommand),
         std::string(protocol::kDefaultFollowCommandEndpoint)},
        {std::string(protocol::kVideoStreamInfo),
         std::string(protocol::kDefaultVideoStreamInfoEndpoint)},
        {std::string(protocol::kTrackingStatus), std::string(protocol::kDefaultTrackingEndpoint)},
        {std::string(protocol::kTrackingTracks), std::string(protocol::kDefaultTrackingEndpoint)},
        {std::string(protocol::kVisionStatus), std::string(protocol::kDefaultVisionEndpoint)},
    };
}

void log_configuration(const mavlink_gateway::UdpMavlinkConnection& connection,
                       const mavlink_gateway::UdpMavlinkConfig& udp,
                       const mavlink_gateway::UdpGcsLink& gcs_link,
                       const mavlink_gateway::UdpGcsConfig& gcs, const Options& options,
                       const bus_zmq::ZmqEndpointTable& table) {
    std::cout << kLogPrefix << "FC UDP " << udp.local_address << ":" << connection.local_port()
              << "\n";
    std::cout << kLogPrefix << "통제기 UDP " << gcs.local_address << ":" << gcs_link.local_port()
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
        bus_zmq::ZmqCodecTable codecs = argus::vehicle_api_bus::zmq_codecs();
        bus_zmq::ZmqCodecTable video_codecs = argus::video_api_bus::zmq_codecs();
        bus_zmq::ZmqCodecTable perception_codecs = argus::perception_api_bus::zmq_codecs();
        codecs.merge(video_codecs);
        codecs.merge(perception_codecs);
        return std::make_unique<bus_zmq::ZmqBus>(context, poller, clock, table, std::move(codecs));
    }
#ifdef ARGUS_HAS_BUS_DDS
    argus::bus_dds::DdsTypeTable types = argus::vehicle_api_bus::dds_types();
    argus::bus_dds::DdsTypeTable video_types = argus::video_api_bus::dds_types();
    argus::bus_dds::DdsTypeTable perception_types = argus::perception_api_bus::dds_types();
    types.merge(video_types);
    types.merge(perception_types);
    // 원인은 DdsBus 가 기록한다
    return argus::bus_dds::DdsBus::create(poller, poller, clock, options.dds_domain,
                                          std::move(types));
#else
    std::cerr << kLogPrefix << "DDS 없이 빌드됨 (ARGUS_BUILD_BUS_DDS=OFF). " << kBusOption << " "
              << kBusZmq << " 로 실행해야 한다\n";
    return nullptr;
#endif
}

// 통제기 쪽 객체. 멤버는 선언의 역순으로 소멸하므로, 참조되는 객체를 참조하는 객체보다 앞에 둔다
struct GcsSide {
    std::unique_ptr<mavlink_gateway::UdpGcsLink> link;
    std::unique_ptr<mavlink_gateway::GcsRegistry> registry;
    mavlink_gateway::CommandActivity activity;
    mavlink_gateway::InactiveLandingLock landing_lock;
    mavlink_gateway::InactiveRcOverride rc_override;
    std::unique_ptr<argus::vehicle_api_bus::BusFollowStatusSubscriber> follow_status;
    std::unique_ptr<argus::vehicle_api_bus::BusFollowControlClient> follow_control;
    std::unique_ptr<argus::video_api_bus::BusVideoStreamInfoSubscriber> video_stream_info;
    std::unique_ptr<mavlink_gateway::RequestHeaderFactory> request_headers;
    std::unique_ptr<mavlink_gateway::ControlAuthority> authority;
    std::unique_ptr<mavlink_gateway::RequestOperatorControlHandler> request_control_handler;
    std::unique_ptr<mavlink_gateway::RequestMessageHandler> request_message_handler;
    std::unique_ptr<mavlink_gateway::MavlinkCommandReceiver> receiver;
    std::unique_ptr<mavlink_gateway::GcsFcForwarder> forwarder;
    std::unique_ptr<mavlink_gateway::FcTelemetryRelay> telemetry_relay;
    std::unique_ptr<mavlink_gateway::GcsLinkMonitor> monitor;
    mavlink_gateway::SysfsCpuTemperature cpu_temperature;
    std::unique_ptr<mavlink_gateway::ArgusStatusReporter> status_reporter;
    std::unique_ptr<argus::perception_api_bus::BusTrackingStatusSubscriber> tracking_status;
    std::unique_ptr<argus::perception_api_bus::BusVisionStatusSubscriber> vision_status;
    mavlink_gateway::SensorSourceMap sensors = mavlink_gateway::SensorSourceMap::default_map();
    std::unique_ptr<mavlink_gateway::ArgusSensorStatusReporter> sensor_status_reporter;
    std::unique_ptr<mavlink_gateway::LatestTracks> latest_tracks;
    std::unique_ptr<mavlink_gateway::FollowCommandTracker> follow_tracker;
    std::unique_ptr<mavlink_gateway::TrackStartHandler> track_start_handler;
    std::unique_ptr<mavlink_gateway::TrackStopHandler> track_stop_handler;
    std::unique_ptr<mavlink_gateway::ScanStartHandler> scan_start_handler;
    core::PeriodicTimer receiver_timer;
    core::PeriodicTimer monitor_timer;
    core::PeriodicTimer status_timer;
    core::PeriodicTimer sensor_status_timer;
    core::PeriodicTimer follow_tracker_timer;
};

// 추종 명령(TRACK_START, TRACK_STOP, SCAN_START)의 handler 를 만들어 등록한다. create_gcs_side 가
// 통제기 쪽의 나머지 객체를 만든 뒤 부른다. tracking/tracks 구독자를 만들지 못하면 false
bool add_follow_commands(GcsSide& gcs, argus::bus::IBus& bus, const core::IClock& clock,
                         core::EventLoop& loop, mavlink_gateway::FcGateway& fc) {
    gcs.latest_tracks = mavlink_gateway::LatestTracks::create(bus);
    if (!gcs.latest_tracks) {
        return false;
    }
    gcs.follow_tracker = std::make_unique<mavlink_gateway::FollowCommandTracker>(
        clock, *gcs.follow_status, *gcs.follow_status, gcs.request_headers->session_id());
    gcs.track_start_handler = std::make_unique<mavlink_gateway::TrackStartHandler>(
        mavlink_gateway::TrackStartHandlerInputs{
            clock, gcs.sensors, *gcs.tracking_status, *gcs.vision_status, *gcs.latest_tracks,
            *gcs.request_headers, *gcs.follow_control, *gcs.follow_tracker});
    gcs.track_stop_handler =
        std::make_unique<mavlink_gateway::TrackStopHandler>(mavlink_gateway::TrackStopHandlerInputs{
            clock, fc.lock_release(), *gcs.follow_status, *gcs.request_headers, *gcs.follow_control,
            *gcs.follow_tracker});
    gcs.scan_start_handler =
        std::make_unique<mavlink_gateway::ScanStartHandler>(mavlink_gateway::ScanStartHandlerInputs{
            *gcs.request_headers, *gcs.follow_control, *gcs.follow_tracker});
    gcs.receiver->add_handler(*gcs.track_start_handler, mavlink_gateway::TrackStartHandler::kRules);
    gcs.receiver->add_handler(*gcs.track_stop_handler, mavlink_gateway::TrackStopHandler::kRules);
    gcs.receiver->add_handler(*gcs.scan_start_handler, mavlink_gateway::ScanStartHandler::kRules);
    mavlink_gateway::FollowCommandTracker* const follow_tracker = gcs.follow_tracker.get();
    gcs.follow_tracker_timer = loop.add_periodic(mavlink_gateway::kFollowCommandTrackerTickPeriod,
                                                 [follow_tracker] { follow_tracker->tick(); });
    return true;
}

// 통제기 쪽 객체를 만들어 연결하고 주기 작업을 등록한다. 하나라도 만들지 못하면 false (원인은
// UdpGcsLink 와 bus 구현이 기록한다)
bool create_gcs_side(GcsSide& gcs, const mavlink_gateway::UdpGcsConfig& config,
                     argus::bus::IBus& bus, bus_zmq::ZmqPoller& poller, const core::IClock& clock,
                     core::EventLoop& loop, mavlink_gateway::IMavlinkConnection& connection,
                     mavlink_gateway::FcGateway& fc, mavlink_gateway::HealthState& health) {
    gcs.link = mavlink_gateway::UdpGcsLink::create(poller, config);
    gcs.follow_status = argus::vehicle_api_bus::BusFollowStatusSubscriber::create(bus, clock);
    gcs.follow_control = argus::vehicle_api_bus::BusFollowControlClient::create(bus);
    gcs.video_stream_info = argus::video_api_bus::BusVideoStreamInfoSubscriber::create(bus, clock);
    if (!gcs.link || !gcs.follow_status || !gcs.follow_control || !gcs.video_stream_info) {
        return false;
    }
    gcs.registry = std::make_unique<mavlink_gateway::GcsRegistry>(*gcs.link, clock);
    gcs.request_headers = std::make_unique<mavlink_gateway::RequestHeaderFactory>(
        make_session_id(), clock, fc.fc_epoch_source());
    gcs.authority =
        std::make_unique<mavlink_gateway::ControlAuthority>(mavlink_gateway::ControlAuthorityInputs{
            *gcs.registry, *gcs.registry, clock, fc.fc_state(), fc.command_status(), gcs.activity,
            *gcs.follow_status, gcs.landing_lock});
    gcs.request_control_handler =
        std::make_unique<mavlink_gateway::RequestOperatorControlHandler>(*gcs.authority);
    gcs.receiver = std::make_unique<mavlink_gateway::MavlinkCommandReceiver>(
        *gcs.registry, *gcs.link, clock, *gcs.authority, gcs.activity);
    gcs.receiver->add_handler(*gcs.request_control_handler,
                              mavlink_gateway::RequestOperatorControlHandler::kRules);
    gcs.request_message_handler =
        std::make_unique<mavlink_gateway::RequestMessageHandler>(*gcs.link);
    const argus::video_api::IVideoStreamInfoSource* const video_source =
        gcs.video_stream_info.get();
    const mavlink_gateway::IGcsLink* const link = gcs.link.get();
    gcs.request_message_handler->add_message(
        MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
        [video_source, link](const mavlink_gateway::CommandRequest& request) {
            return mavlink_gateway::build_video_stream_information(request, *video_source, *link);
        });
    gcs.receiver->add_handler(*gcs.request_message_handler,
                              mavlink_gateway::RequestMessageHandler::kRules);
    // ControlAuthority 보다 뒤에 만들어, 제어권을 부여한 HEARTBEAT 부터 FC 로 전달한다
    gcs.forwarder =
        std::make_unique<mavlink_gateway::GcsFcForwarder>(mavlink_gateway::GcsFcForwarderInputs{
            *gcs.registry, *gcs.link, *gcs.registry, connection, clock, *gcs.authority,
            fc.fc_state(), fc.command_status(), gcs.activity, *gcs.follow_status, gcs.landing_lock,
            gcs.rc_override, fc.stop_requester()});
    gcs.telemetry_relay =
        std::make_unique<mavlink_gateway::FcTelemetryRelay>(connection, *gcs.link);
    gcs.monitor =
        std::make_unique<mavlink_gateway::GcsLinkMonitor>(mavlink_gateway::GcsLinkMonitorInputs{
            *gcs.registry, *gcs.authority, clock, *gcs.follow_status, *gcs.follow_control,
            *gcs.request_headers, health});
    gcs.status_reporter = std::make_unique<mavlink_gateway::ArgusStatusReporter>(
        mavlink_gateway::ArgusStatusReporterInputs{
            *gcs.link, clock, *gcs.follow_status, *gcs.follow_status, *gcs.authority, health,
            fc.command_status(), fc.stop_requester(), gcs.cpu_temperature});
    mavlink_gateway::MavlinkCommandReceiver* const receiver = gcs.receiver.get();
    mavlink_gateway::GcsLinkMonitor* const monitor = gcs.monitor.get();
    gcs.receiver_timer = loop.add_periodic(mavlink_gateway::kCommandReceiverTickPeriod,
                                           [receiver] { receiver->tick(); });
    gcs.monitor_timer =
        loop.add_periodic(mavlink_gateway::kGcsLinkCheckPeriod, [monitor] { monitor->tick(); });
    mavlink_gateway::ArgusStatusReporter* const status_reporter = gcs.status_reporter.get();
    gcs.status_timer = loop.add_periodic(mavlink_gateway::kArgusStatusCheckPeriod,
                                         [status_reporter] { status_reporter->tick(); });

    gcs.tracking_status =
        argus::perception_api_bus::BusTrackingStatusSubscriber::create(bus, clock);
    gcs.vision_status = argus::perception_api_bus::BusVisionStatusSubscriber::create(bus, clock);
    if (!gcs.tracking_status || !gcs.vision_status) {
        return false;
    }
    gcs.sensor_status_reporter = std::make_unique<mavlink_gateway::ArgusSensorStatusReporter>(
        mavlink_gateway::ArgusSensorStatusReporterInputs{*gcs.link, clock, *gcs.tracking_status,
                                                         *gcs.vision_status, gcs.sensors});
    mavlink_gateway::ArgusSensorStatusReporter* const sensor_status_reporter =
        gcs.sensor_status_reporter.get();
    gcs.sensor_status_timer =
        loop.add_periodic(mavlink_gateway::kSensorStatusCheckPeriod,
                          [sensor_status_reporter] { sensor_status_reporter->tick(); });
    return add_follow_commands(gcs, bus, clock, loop, fc);
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

    // gateway 보다 뒤에 선언하여 먼저 소멸시킨다 (통제기 쪽이 FC 쪽 객체를 참조)
    mavlink_gateway::UdpGcsConfig gcs_config;
    gcs_config.local_port = options.gcs_port;
    GcsSide gcs;
    if (!create_gcs_side(gcs, gcs_config, *bus, poller, clock, loop, *connection, *gateway,
                         health)) {
        return EXIT_FAILURE;
    }

    // gcs 보다 뒤에 선언하여 먼저 소멸시킨다 (통제기 연결을 참조)
    mavlink_gateway::HeartbeatSender heartbeat{*gcs.link, *connection, health};
    const core::PeriodicTimer heartbeat_timer =
        loop.add_periodic(mavlink_gateway::kHeartbeatPeriod, [&heartbeat] { heartbeat.tick(); });

    log_configuration(*connection, udp_config, *gcs.link, gcs_config, options, table);
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
