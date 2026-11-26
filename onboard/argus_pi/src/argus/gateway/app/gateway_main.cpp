// C++ FC 게이트웨이 실행 진입점 (Pi tools/real_fc/gateway.py main()과 같은 인수·조립·종료 규칙).
// 운영 실행: argus_fc_gateway --router --rc-reclaim --main-gcs (run_pi_fc.sh가 고른다).
// 종료 신호를 받아도 FC가 지상·무장 해제가 아니면 원본처럼 끝내지 않는다.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "argus/gateway/bridge/include/follow_preferences.hpp"
#include "argus/gateway/bridge/include/main_gcs_bridge.hpp"
#include "argus/gateway/bridge/include/profile_coordinator.hpp"
#include "argus/gateway/bridge/include/udp_sockets.hpp"
#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/follow/include/follow_owner.hpp"
#include "argus/gateway/follow/include/follow_proxy.hpp"
#include "argus/gateway/follow/include/zmq_transport.hpp"
#include "argus/gateway/http/include/gateway_api.hpp"
#include "argus/gateway/http/include/http_server.hpp"
#include "argus/gateway/ownership/include/operator_authority.hpp"
#include "argus/gateway/service/include/follow_service.hpp"
#include "argus/gateway/service/include/maintenance.hpp"
#include "argus/gateway/settings/include/deployment_settings.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"
#include "argus/gateway/vehicle/include/rc_reclaim.hpp"

namespace gw = argus::gateway;

namespace {

struct Arguments {
    int port{8768};
    std::optional<std::string> expected_uid;
    bool follow_b{true};
    bool router{false};
    bool rc_reclaim{false};
    bool main_gcs{false};
    std::optional<std::filesystem::path> root;
};

constexpr const char* kUsage =
    "usage: argus_fc_gateway [-h] [--port PORT] [--expected-uid EXPECTED_UID]\n"
    "                        [--follow-b | --no-follow-b] [--router] [--rc-reclaim]\n"
    "                        [--main-gcs] [--root ROOT]\n";

[[noreturn]] void usage_error(const std::string& message) {
    std::cerr << kUsage << "argus_fc_gateway: error: " << message << '\n';
    std::exit(2);  // NOLINT(concurrency-mt-unsafe): 시작 전 단일 스레드
}

Arguments parse(const std::vector<std::string>& args) {
    Arguments out;
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args.at(i);
        const auto value = [&](const char* name) {
            if (i + 1 >= args.size()) {
                usage_error(std::string("argument ") + name + ": expected one argument");
            }
            return args.at(++i);
        };
        if (arg == "-h" || arg == "--help") {
            std::cout << kUsage;
            std::exit(0);  // NOLINT(concurrency-mt-unsafe): 시작 전 단일 스레드
        } else if (arg == "--port") {
            const std::string text = value("--port");
            long long port = 0;
            if (!gw::python_parse_int(text, port)) {
                usage_error("argument --port: invalid int value: '" + text + "'");
            }
            out.port = static_cast<int>(port);
        } else if (arg == "--expected-uid") {
            out.expected_uid = value("--expected-uid");
        } else if (arg == "--follow-b") {
            out.follow_b = true;
        } else if (arg == "--no-follow-b") {
            out.follow_b = false;
        } else if (arg == "--router") {
            out.router = true;
        } else if (arg == "--rc-reclaim") {
            out.rc_reclaim = true;
        } else if (arg == "--main-gcs") {
            out.main_gcs = true;
        } else if (arg == "--root") {
            out.root = value("--root");
        } else {
            usage_error("unrecognized arguments: " + arg);
        }
    }
    if (out.rc_reclaim && !out.router) {
        usage_error("RC 복귀는 Pi router 경로에서만 사용할 수 있습니다");
    }
    if (out.main_gcs && !(out.rc_reclaim && out.router && out.follow_b)) {
        usage_error("메인 GCS 중재는 router·RC 복귀·ARGUS B 연결부가 필요합니다");
    }
    if (out.port < 0 || out.port > 65535) {
        usage_error("포트 범위 오류");
    }
    return out;
}

// 원본 ROOT: 설치 폴더 (실행 파일은 ROOT/bin 아래에 있다)
std::filesystem::path install_root() {
    std::error_code error;
    const auto exe = std::filesystem::read_symlink("/proc/self/exe", error);
    return error ? std::filesystem::current_path() : exe.parent_path().parent_path();
}

std::string random_hex() {
    static thread_local std::mt19937_64 engine{std::random_device{}()};
    static constexpr std::array<char, 16> kDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                                  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
    std::string out;
    for (int i = 0; i < 32; ++i) {
        out += kDigits.at(engine() % 16);
    }
    return out;
}

double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::int64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// 원본 RC 복귀의 추종기 확인: operator 포트에 0.3초 안에 TCP 연결되는지
bool port_present(int port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        return false;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    // POSIX 소켓 API는 sockaddr*를 받는다
    bool connected = ::connect(fd, reinterpret_cast<const sockaddr*>(&address),  // NOLINT
                               sizeof(address)) == 0;
    if (!connected && errno == EINPROGRESS) {
        pollfd item{fd, POLLOUT, 0};
        if (::poll(&item, 1, 300) > 0) {
            int failure = 0;
            socklen_t length = sizeof(failure);
            ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &failure, &length);
            connected = failure == 0;
        }
    }
    ::close(fd);
    return connected;
}

int endpoint_port(const std::string& endpoint) {
    return std::stoi(endpoint.substr(endpoint.rfind(':') + 1));
}

std::string host_name() {
    std::array<char, 256> name{};
    return ::gethostname(name.data(), name.size()) == 0 ? std::string(name.data()) : "";
}

int run(const Arguments& args) {
    const std::filesystem::path root = args.root ? *args.root : install_root();
    const gw::OrderedJson settings = gw::load_deployment_settings(root, gw::current_environment());
    const std::string uid = args.expected_uid
                                ? *args.expected_uid
                                : settings.at("device").at("expected_fc_uid").get<std::string>();
    const std::string router_address =
        "udpout:127.0.0.1:" +
        std::to_string(settings.at("device").at("router_gateway_port").get<int>());
    // 원본: os.environ.setdefault("ARGUS_PROFILE_REBOOT_FILE",
    // ~/.local/state/TheSSEN/fc-profile/uid.json)
    {
        const char* home = std::getenv("HOME");           // NOLINT(concurrency-mt-unsafe): 시작 전
        const char* local = std::getenv("LOCALAPPDATA");  // NOLINT(concurrency-mt-unsafe)
        const std::filesystem::path state_root =
            local != nullptr ? std::filesystem::path(local)
                             : std::filesystem::path(home != nullptr ? home : "") / ".local/state";
        const auto marker = state_root / "TheSSEN/fc-profile" / (uid + ".json");
        ::setenv("ARGUS_PROFILE_REBOOT_FILE", marker.c_str(), 0);  // NOLINT(concurrency-mt-unsafe)
    }
    const std::string operator_endpoint = settings.at("endpoints").at("operator");
    gw::FollowProxy proxy(operator_endpoint);
    const auto forward = [&proxy](const gw::Wire& body, const std::string& who, int timeout_ms) {
        return proxy.forward(body, who, timeout_ms);
    };
    std::unique_ptr<gw::OperatorAuthority> authority;
    if (args.rc_reclaim) {
        authority = std::make_unique<gw::OperatorAuthority>();
    }
    gw::FlightLinkOptions options =
        gw::FlightLink::default_options(uid, root / "apps/real_fc/tools/real_fc/fc-events.json");
    std::unique_ptr<gw::FlightLink> vehicle;
    if (args.rc_reclaim) {
        gw::ReclaimController controller;
        const int operator_port = endpoint_port(operator_endpoint);
        controller.present = [operator_port] { return port_present(operator_port); };
        controller.request = [&forward, uid](const gw::Json& body, int timeout_ms) {
            return gw::Json::parse(forward(gw::to_wire(body), uid, timeout_ms).dump());
        };
        auto reclaim = std::make_unique<gw::ReclaimFlightLink>(std::move(options), controller,
                                                               authority.get());
        // 2026-11-10 실내 RC→GUI 복귀는 HOLD 대신 Offboard 제자리 유지(GPS 없는 PX4 HOLD 거부).
        reclaim->set_indoor_offboard_reclaim(true);
        vehicle = std::move(reclaim);
    } else {
        vehicle = std::make_unique<gw::FlightLink>(std::move(options));
    }
    vehicle->start();
    if (args.follow_b) {
        const gw::OrderedJson& endpoints = settings.at("endpoints");
        const gw::OrderedJson v21 =
            settings.contains("v2_1") ? settings.at("v2_1") : gw::OrderedJson::object();
        const bool cv_gap = gw::wire_truthy(gw::wire_get(v21, "enabled")) &&
                            gw::wire_truthy(gw::wire_get(v21, "cv_gap"));
        gw::FollowEndpoints follow_endpoints{endpoints.at("fc_state"), endpoints.at("mode_rpc"),
                                             endpoints.at("control"), std::nullopt};
        if (cv_gap && endpoints.contains("tracks")) {
            follow_endpoints.tracks = endpoints.at("tracks").get<std::string>();
        }
        gw::FollowOwnerOptions owner_options;
        owner_options.offboard_hover = true;
        owner_options.cv_gap_enabled = cv_gap;
        owner_options.journal_directory = root / "run/follow_b";
        owner_options.authority = authority.get();
        const std::scoped_lock lock(vehicle->mutex());
        vehicle->state().follow_b = std::make_unique<gw::FollowBOwner>(
            *vehicle, std::make_unique<gw::ZmqFollowTransport>(follow_endpoints), owner_options);
    }
    gw::FollowPreferences preferences(root / "config/operator/follow-preferences.json");
    gw::FollowService follow_service(gw::FollowServiceOptions{root, forward, {}, 45});
    gw::FollowProfileCoordinator profiles(*vehicle, authority.get(), preferences,
                                          follow_service.services());
    const char* fc_python = std::getenv("THESSEN_FC_PYTHON");  // NOLINT(concurrency-mt-unsafe)
    gw::Maintenance maintenance(gw::MaintenanceOptions{
        root, forward, fc_python != nullptr ? fc_python : (root / ".venv-fc/bin/python").string(),
        args.port, settings.at("device").at("uart_device").get<std::string>()});
    std::unique_ptr<gw::MainGcsBridge> bridge;
    if (args.main_gcs) {
        gw::MainGcsBridgeOptions bridge_options;
        bridge_options.monotonic = monotonic;
        bridge_options.monotonic_ns = monotonic_ns;
        bridge_options.new_id = random_hex;
        bridge_options.forward = forward;
        bridge_options.log_path = root / "run/main-gcs-bridge.jsonl";
        bridge_options.router_gateway_port = settings.at("device").at("router_gateway_port");
        bridge_options.uart_baud = settings.at("device").at("uart_baud");
        bridge = std::make_unique<gw::MainGcsBridge>(
            *vehicle, *authority, profiles,
            std::make_unique<gw::UdpBridgeSockets>(14550, 14561,
                                                   settings.at("endpoints").at("tracks")),
            std::move(bridge_options));
    }
    // 2026-11-18 데모 모드: 실내 demo가 선택되었거나 실행 중이면 이륙/고도 변경은 1.0~2.0 m만.
    // 원천은 FC 링크 잠금 안에서 불린다. 파일 읽기와 연결부 상태 복사만 하고 다른 잠금은 잡지
    // 않는다.
    vehicle->set_height_band_source(
        [&preferences, bridge_ptr = bridge.get()]() -> std::optional<gw::FlightLink::HeightBand> {
            const std::optional<gw::Wire> status =
                bridge_ptr != nullptr ? bridge_ptr->fresh_status() : std::nullopt;
            if (!gw::demo_selected_or_active(preferences, status)) {
                return std::nullopt;
            }
            return gw::FlightLink::HeightBand{gw::kDemoMinHeightM, gw::kDemoMaxHeightM,
                                              gw::kDemoHeightMessage};
        });
    // 2026-11-18 데모 위치 고정: 실행 중 추종기가 실내 demo인가. TRACK_START 회전 모드 판정
    // (track_adapter)과 같은 active_profile, demo_profile을 쓴다. 1초 안 상태가 있으면 그 값,
    // 없으면 마지막 TRACK_START 명령이 확인한 값(30초 안)을 쓴다. 인계 대기 중 bridge 스레드가
    // 상태 조회를 멈춰도 첫 송신이 판정을 잃지 않게 하려는 것이다. 둘 다 없으면 모름(비어 있음).
    // 메인 GUI 연결부가 없으면 실내 demo 시작 경로(track_adapter)가 없으므로 기존 송신(거짓)이다.
    // 연결부 상태 복사만 하고 다른 잠금은 잡지 않는다.
    vehicle->set_demo_follow_source([bridge_ptr = bridge.get()]() -> std::optional<bool> {
        constexpr double kTrackStartDemoMaxAgeS = 30.0;
        if (bridge_ptr == nullptr) {
            return false;
        }
        const std::optional<gw::Wire> status = bridge_ptr->fresh_status();
        if (status && status->is_object()) {
            try {
                const gw::Wire profile = gw::active_profile(*status);
                if (!profile.is_null()) {
                    return gw::demo_profile(profile);
                }
            } catch (const std::exception&) {
                // 아래 TRACK_START 기록으로 판정한다
            }
        }
        return bridge_ptr->track_start_demo(kTrackStartDemoMaxAgeS);
    });
    // 먼저 해제되는 preferences, bridge를 FC 스레드가 보지 않도록 끝날 때 원천을 지운다.
    struct HeightBandReset {
        gw::FlightLink& link;
        HeightBandReset(const HeightBandReset&) = delete;
        HeightBandReset& operator=(const HeightBandReset&) = delete;
        HeightBandReset(HeightBandReset&&) = delete;
        HeightBandReset& operator=(HeightBandReset&&) = delete;
        ~HeightBandReset() {
            link.set_height_band_source({});
            link.set_demo_follow_source({});
        }
    } const height_band_reset{*vehicle};
    std::unique_ptr<gw::HttpServer> server;
    gw::GatewayApiOptions api_options;
    if (args.router) {
        api_options.router_address = router_address;
    }
    api_options.uart_device = settings.at("device").at("uart_device");
    api_options.forward = forward;
    api_options.new_id = random_hex;
    api_options.shutdown = [&server] { std::thread([&server] { server->shutdown(); }).detach(); };
    api_options.host = host_name();
    api_options.serial_ports = gw::linux_serial_ports;
    gw::GatewayApi api(*vehicle, authority.get(), bridge.get(), profiles, maintenance,
                       std::move(api_options));
    server = std::make_unique<gw::HttpServer>(
        args.port, [&api](const gw::HttpRequest& request) { return api.handle(request); });
    if (bridge) {
        bridge->start();
    }
    std::cout << "실기체 게이트웨이 http://127.0.0.1:" << server->port() << " · FC 미연결\n"
              << std::flush;
    // 원본 signal(SIGTERM/SIGINT, shutdown): FC가 멈출 수 없으면 알리고 계속 돈다
    std::thread signals([&vehicle, &server] {
        sigset_t set;
        sigemptyset(&set);
        sigaddset(&set, SIGTERM);
        sigaddset(&set, SIGINT);
        sigaddset(&set, SIGUSR1);
        for (;;) {
            int signal_number = 0;
            if (sigwait(&set, &signal_number) != 0) {
                continue;
            }
            if (signal_number == SIGUSR1) {
                return;
            }
            try {
                vehicle->stop();
            } catch (const gw::ValueError& error) {
                std::cout << error.what() << '\n' << std::flush;
                continue;
            }
            server->shutdown();
            return;
        }
    });
    server->serve_forever();
    if (bridge) {
        bridge->close();
    }
    server->close();
    {
        const std::scoped_lock lock(vehicle->mutex());
        if (auto* owner = dynamic_cast<gw::FollowBOwner*>(vehicle->state().follow_b.get())) {
            owner->close();
        }
        vehicle->state().follow_b.reset();
    }
    if (signals.joinable()) {
        ::pthread_kill(signals.native_handle(), SIGUSR1);  // 신호 스레드를 깨워 끝낸다
        signals.join();
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    // 종료 신호는 신호 전용 스레드만 받는다 (모든 스레드를 만들기 전에 막는다)
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGUSR1);
    ::pthread_sigmask(SIG_BLOCK, &set, nullptr);
    ::signal(SIGPIPE, SIG_IGN);
    const std::vector<std::string> args(argv, std::next(argv, argc));
    try {
        return run(parse(args));
    } catch (const std::exception& error) {
        std::cerr << "게이트웨이 시작 실패: " << error.what() << '\n';
        return 1;
    }
}
