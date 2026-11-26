#include "argus/gateway/service/include/maintenance.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <zmq.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iterator>
#include <set>
#include <thread>

#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/follow/include/fc_protobuf.hpp"
#include "argus/gateway/follow/include/follow_proxy.hpp"
#include "argus/gateway/service/include/process_supervisor.hpp"
#include "argus/gateway/settings/include/deployment_settings.hpp"

namespace argus::gateway {

namespace {

using Clock = std::chrono::steady_clock;

constexpr std::array<const char*, 7> kIrNames{"argus-onboard",           "argus-frame-source",
                                              "i3_managed_capture",      "argus_preview_bus",
                                              "argus_preview_udp_relay", "argus_follow_tracker",
                                              "argus_follow_controller"};
constexpr std::array<const char*, 3> kRgbNames{"argus_follow_tracker", "argus_follow_controller",
                                               "argus_target_diagnostic"};

// 2026-11-24 사용자 지시: FC 장치가 없을 때 끌 수 있는 카메라/탐지 프로세스 (제어기, 추적기, FC
// 연결부 제외)
constexpr std::array<const char*, 5> kPerceptionNames{"argus-onboard", "argus-frame-source",
                                                      "i3_managed_capture", "argus_preview_bus",
                                                      "argus_preview_udp_relay"};

template <std::size_t N>
bool named(const std::array<const char*, N>& names, const std::string& name) {
    return std::any_of(names.begin(), names.end(),
                       [&name](const char* candidate) { return name == candidate; });
}

double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

bool is_true(const Wire& value) { return value.is_boolean() && value.get<bool>(); }

bool is_false(const Wire& value) { return value.is_boolean() && !value.get<bool>(); }

std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw os_error(errno, path.string());
    }
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

int endpoint_port(const std::string& endpoint) {
    return std::stoi(endpoint.substr(endpoint.rfind(':') + 1));
}

// 원본 int((proc/'stat').read_text()의 ')' 뒤 20번째 값) (프로세스 시작 시각)
long start_ticks(const std::filesystem::path& proc) {
    const std::string data = read_file(proc / "stat");
    const std::size_t close = data.rfind(')');
    if (close == std::string::npos || close + 2 > data.size()) {
        throw ValueError("stat");
    }
    std::istringstream rest(data.substr(close + 2));
    std::string field;
    for (int i = 0; i <= 19; ++i) {
        if (!(rest >> field)) {
            throw ValueError("stat");
        }
    }
    long long value = 0;
    if (!python_parse_int(field, value)) {
        throw ValueError("stat");
    }
    return static_cast<long>(value);
}

bool contains(const std::vector<std::string>& arguments, const std::string& value) {
    return std::find(arguments.begin(), arguments.end(), value) != arguments.end();
}

// 원본 urllib.request.urlopen('http://127.0.0.1:<port>/state', timeout=.6)
Wire local_state(int port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        throw os_error(errno);
    }
    struct Closer {
        int fd;
        Closer(const Closer&) = delete;
        Closer& operator=(const Closer&) = delete;
        Closer(Closer&&) = delete;
        Closer& operator=(Closer&&) = delete;
        ~Closer() { ::close(fd); }
    } const closer{fd};
    timeval timeout{0, 600000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    // POSIX 소켓 API는 sockaddr*를 받는다
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&address),  // NOLINT(*-reinterpret-cast)
                  sizeof(address)) != 0) {
        throw os_error(errno);
    }
    const std::string request = "GET /state HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n";
    if (::send(fd, request.data(), request.size(), 0) < 0) {
        throw os_error(errno);
    }
    std::string response;
    std::array<char, 8192> buffer{};
    for (;;) {
        const ssize_t got = ::recv(fd, buffer.data(), buffer.size(), 0);
        if (got < 0) {
            throw os_error(errno);
        }
        if (got == 0) {
            break;
        }
        response.append(buffer.data(), static_cast<std::size_t>(got));
    }
    const std::size_t body = response.find("\r\n\r\n");
    if (body == std::string::npos) {
        throw ValueError("gateway state response");
    }
    return Wire::parse(response.substr(body + 4));
}

}  // namespace

Maintenance::Maintenance(MaintenanceOptions options) : options_(std::move(options)) {}

bool Maintenance::processes_installed() const {
    std::error_code error;
    return std::filesystem::is_regular_file(
        options_.root / "src/argus/diagnostics/tracking/maintenance_processes.py", error);
}

bool Maintenance::diagnostic_installed() const {
    std::error_code error;
    return std::filesystem::is_regular_file(
        options_.root / "src/argus/diagnostics/tracking/ground_diagnostic_service.py", error);
}

Wire Maintenance::inventory() const {
    const std::filesystem::path& root = options_.root;
    const std::string tracking = (root / "src/argus/diagnostics/tracking").string();
    const std::string observer = tracking + "/maintenance_observer.py";
    const std::array<std::string, 3> scripts{tracking + "/ground_recorder.py",
                                             tracking + "/native_ground_recorder.py", observer};
    struct Item {
        int pid;
        long ticks;
        std::string name;
        bool protected_process;
        std::string reason;
    };
    std::vector<Item> items;
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator("/proc", error)) {
        const std::string name = entry.path().filename().string();
        if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos ||
            std::stoi(name) == ::getpid()) {
            continue;
        }
        try {
            std::error_code link_error;
            const std::filesystem::path exe =
                std::filesystem::read_symlink(entry.path() / "exe", link_error);
            if (link_error) {
                continue;
            }
            const auto arguments = process_arguments(std::stoi(name));
            if (!arguments) {
                continue;
            }
            std::string kind;
            bool protected_process = false;
            std::string reason;
            std::error_code cwd_error;
            const std::string cwd =
                resolved(std::filesystem::read_symlink(entry.path() / "cwd", cwd_error)).string();
            const auto cwd_is = [&](const std::filesystem::path& path) {
                return !cwd_error && cwd == path.string();
            };
            if (exe.parent_path() == root / "bin" && named(kIrNames, exe.filename().string())) {
                kind = exe.filename().string();
            } else if (exe.parent_path() == "/home/user/ARGUS_RGB/ucmc/bin" &&
                       named(kRgbNames, exe.filename().string())) {
                kind = "RGB/" + exe.filename().string();
                protected_process = true;
                reason = "RGB 독립 제어 경로는 여기서 종료하지 않습니다";
            } else if (exe == "/opt/argus/bin/argus_onboard") {
                kind = "onboard(메인 관제 연결)";
                protected_process = true;
                reason = "메인 관제 연결을 보존합니다";
            } else if (exe == "/opt/argus/bin/ir_video_sender") {
                kind = "IR 영상 송신기";
            } else if (std::any_of(scripts.begin(), scripts.end(), [&](const std::string& path) {
                           return contains(*arguments, path);
                       })) {
                kind = contains(*arguments, observer) ? "IR 대상 상태 보기" : "IR 지상 진단 기록기";
            } else if (contains(*arguments, "/home/user/ARGUS_RGB/live/producer.py")) {
                kind = "RGB 영상·검출";
                protected_process = true;
                reason = "RGB 독립 실행 경로는 여기서 종료하지 않습니다";
            } else if (contains(
                           *arguments,
                           "/home/user/ARGUS_RGB/ucmc/apps/real_fc/tools/real_fc/gateway.py") ||
                       (contains(*arguments, "tools.real_fc.gateway") && !cwd_error &&
                        cwd.rfind("/home/user/ARGUS_RGB/", 0) == 0)) {
                kind = "RGB FC 연결부";
                protected_process = true;
                reason = "RGB 독립 FC 연결을 보존합니다";
            } else if ((contains(*arguments, "tools.real_fc.gateway") ||
                        exe == root / "bin" / kGatewayBinaryName) &&
                       cwd_is(root)) {
                kind = "IR FC 연결부";
                protected_process = true;
                reason = "현재 FC 연결 경로를 보존합니다";
            }
            if (kind.empty()) {
                continue;
            }
            struct stat info{};
            if (::stat(entry.path().c_str(), &info) != 0) {
                continue;
            }
            if (info.st_uid != ::getuid()) {
                protected_process = true;
                reason = "현재 사용자 소유 프로세스가 아닙니다";
            }
            items.push_back(
                Item{std::stoi(name), start_ticks(entry.path()), kind, protected_process, reason});
        } catch (const std::exception&) {
            continue;  // 원본 (OSError, ValueError, IndexError)
        }
    }
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
        return std::tie(a.name, a.pid) < std::tie(b.name, b.pid);
    });
    Wire out = Wire::array();
    for (const Item& item : items) {
        Wire row = Wire::object();
        wire_set(row, "pid", item.pid);
        wire_set(row, "start_ticks", item.ticks);
        wire_set(row, "name", item.name);
        wire_set(row, "protected", item.protected_process);
        wire_set(row, "reason", item.reason);
        out.push_back(std::move(row));
    }
    return out;
}

bool Maintenance::confirmed_ground(const Wire& config) {
    void* context = zmq_ctx_new();
    void* socket = zmq_socket(context, ZMQ_SUB);
    const auto finish = [&] {
        zmq_close(socket);
        zmq_ctx_term(context);
    };
    try {
        const int linger = 0;
        const int timeout = 1200;
        zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
        zmq_setsockopt(socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
        const std::string endpoint = config.at("endpoints").at("fc_state");
        zmq_connect(socket, endpoint.c_str());
        zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "fc/state", 8);
        const auto start = Clock::now();
        const std::string uid = config.at("device").at("expected_fc_uid");
        while (elapsed(start) < 1.2) {
            std::vector<Bytes> parts;
            bool received = true;
            for (;;) {
                zmq_msg_t message;
                zmq_msg_init(&message);
                if (zmq_msg_recv(&message, socket, 0) < 0) {
                    zmq_msg_close(&message);
                    received = false;
                    break;
                }
                const auto* data = static_cast<const std::uint8_t*>(zmq_msg_data(&message));
                parts.emplace_back(
                    data, std::next(data, static_cast<std::ptrdiff_t>(zmq_msg_size(&message))));
                const bool more = zmq_msg_more(&message) != 0;
                zmq_msg_close(&message);
                if (!more) {
                    break;
                }
            }
            if (!received) {
                break;  // 원본 zmq.Again
            }
            if (parts.size() != 2 || parts.at(1).size() > std::size_t{1024} * 1024) {
                continue;
            }
            // 2026-11-25 batch 6a: 팀 6370 [fc/state][protobuf]. 이름이 정확히 fc/state 가 아니거나
            // 되살리지 못한 본문은 건너뛴다. 되살린 msgpack 에 예전 키 검사를 그대로 한다 (status
            // 가 없는 본문은 되살리지 못해 armed, in_air 가 없는 본문처럼 지상 확인이 되지 않는다)
            Bytes packed;
            std::string error;
            if (fc_state_msgpack(parts, packed, error) != FcStateFrame::Ok) {
                continue;
            }
            const Wire data = msgpack_unpack(packed);
            if (!data.is_object()) {
                continue;
            }
            const Wire stamp = wire_get(data, "received_monotonic_ns");
            const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 Clock::now().time_since_epoch())
                                 .count();
            const bool fresh = stamp.is_number_integer() && 0 <= now - stamp.get<std::int64_t>() &&
                               now - stamp.get<std::int64_t>() <= 500'000'000;
            if (python_equal(wire_get(data, "schema"), Wire("argus.fc.state.v1")) &&
                python_equal(wire_get(data, "clock_domain"), Wire("host_monotonic")) &&
                python_equal(wire_get(data, "uid"), Wire(uid)) &&
                is_true(wire_get(data, "connected")) && is_false(wire_get(data, "armed")) &&
                is_false(wire_get(data, "in_air")) && is_true(wire_get(data, "ground_known")) &&
                is_true(wire_get(data, "landed")) && is_false(wire_get(data, "offboard")) &&
                is_false(wire_get(data, "rc_override")) && fresh) {
                finish();
                return true;
            }
        }
        finish();
        return false;
    } catch (...) {
        finish();
        throw;
    }
}

bool Maintenance::perception_only(const std::string& name) { return named(kPerceptionNames, name); }

bool Maintenance::fc_link_absent() const {
    if (options_.uart_device.empty()) {
        return false;
    }
    std::error_code error;
    const bool present = std::filesystem::exists(options_.uart_device, error);
    return !error && !present;
}

Wire Maintenance::restart_status() const {
    std::ifstream stream(options_.root / "run/connection/restart-all.json", std::ios::binary);
    if (!stream) {
        return Wire();
    }
    const std::string text{std::istreambuf_iterator<char>(stream),
                           std::istreambuf_iterator<char>()};
    try {
        const Wire value = Wire::parse(text);
        return value.is_object() ? value : Wire();
    } catch (const nlohmann::json::exception&) {
        return Wire();
    }
}

Wire Maintenance::restart() const {
    const std::filesystem::path& root = options_.root;
    const auto script = root / "src/argus/diagnostics/tracking/maintenance_processes.py";
    std::error_code error;
    if (!std::filesystem::is_regular_file(script, error)) {
        throw ValueError("ARGUS 프로세스 관리 모듈 미설치");
    }
    const auto run = root / "run/connection";
    std::filesystem::create_directories(run);
    // 요청 스크립트는 작업자를 새 세션으로 띄우고 바로 끝난다. 작업자가 이 게이트웨이도 내린다.
    ChildProcess request = spawn_logged(root, run / "restart-request.log",
                                        {options_.python, "-B", script.string(), "restart"});
    if (!request.wait(5)) {
        throw ValueError(
            "전체 재시작 요청 응답 시간 초과, run/connection/restart-request.log 확인");
    }
    const Wire status = restart_status();
    if (status.is_null()) {
        throw ValueError("전체 재시작 요청 실패, run/connection/restart-request.log 확인");
    }
    return Wire{{"accepted", true},
                {"reason", "전체 재시작 요청 접수, 게이트웨이 재연결까지 약 10~30초"},
                {"restart", status}};
}

Wire Maintenance::stop(long pid, long ticks, bool vehicle_ground) const {
    const std::filesystem::path& root = options_.root;
    const OrderedJson config = load_deployment_settings(root, current_environment());
    const std::filesystem::path run = root / "run/follow_b";
    std::error_code error;
    if (!std::filesystem::is_directory(run, error)) {
        throw ValueError("추종 준비 잠금 경로 없음");
    }
    const int lock = open_for_append(run / "prepare.lock");
    struct Closer {
        int fd;
        Closer(const Closer&) = delete;
        Closer& operator=(const Closer&) = delete;
        Closer(Closer&&) = delete;
        Closer& operator=(Closer&&) = delete;
        ~Closer() { ::close(fd); }
    } const closer{lock};
    if (::flock(lock, LOCK_EX | LOCK_NB) != 0) {
        throw os_error(errno);  // 원본 BlockingIOError
    }
    const auto find = [pid, ticks](const Wire& entries) {
        Wire matches = Wire::array();
        for (const Wire& item : entries) {
            if (item.at("pid") == pid && item.at("start_ticks") == ticks) {
                matches.push_back(item);
            }
        }
        return matches;
    };
    const Wire entries = inventory();
    const Wire match = find(entries);
    if (match.size() != 1) {
        throw ValueError("프로세스가 종료됐거나 변경됐습니다");
    }
    const Wire& item = match.at(0);
    if (item.at("protected").get<bool>()) {
        throw ValueError(item.at("reason").get<std::string>());
    }
    // 2026-11-24 사용자 지시: FC 장치가 아예 없으면 기체에 명령이 갈 수 없으므로 카메라/탐지만
    // 끈다. 제어기, 추적기, FC 연결부는 지금처럼 FC 지상 확인이 필요하다.
    const bool without_fc = !vehicle_ground;
    if (without_fc && !(fc_link_absent() && perception_only(item.at("name").get<std::string>()))) {
        throw ValueError("FC 미연결 때는 카메라/탐지 프로세스만 종료할 수 있습니다");
    }
    const auto any_named = [&entries](std::initializer_list<const char*> names) {
        return std::any_of(entries.begin(), entries.end(), [&names](const Wire& entry) {
            return std::any_of(names.begin(), names.end(),
                               [&entry](const char* name) { return entry.at("name") == name; });
        });
    };
    const bool control_present =
        any_named({"argus_follow_controller", "onboard(메인 관제 연결)", "IR FC 연결부",
                   "RGB/argus_follow_controller", "RGB FC 연결부"});
    if (!without_fc && control_present && !confirmed_ground(config)) {
        throw ValueError("FC 신원·신선한 지상·무장 해제 상태를 확인할 수 없습니다");
    }
    if (!without_fc && any_named({"RGB/argus_follow_controller", "RGB FC 연결부"})) {
        throw ValueError("RGB 추종 프로세스의 상태를 이 경로에서 확인할 수 없습니다");
    }
    const std::string uid = config.at("device").at("expected_fc_uid");
    if (!without_fc && any_named({"argus_follow_controller"})) {
        const Wire status =
            options_.forward(Wire{{"action", "status"}, {"expected_uid", uid}}, uid, 400);
        validate_follow_status(status, uid);
        if (!is_false(wire_get(status, "active")) || !is_false(wire_get(status, "bench_active"))) {
            throw ValueError("추종 또는 지상 yaw 시험 중에는 종료할 수 없습니다");
        }
    }
    if (item.at("name") == "IR FC 연결부") {
        const Wire gateway = local_state(options_.gateway_port);
        const Wire owner =
            wire_truthy(wire_get(gateway, "follow_b")) ? gateway.at("follow_b") : Wire::object();
        const bool idle_gateway =
            python_equal(wire_get(gateway, "app"), Wire("argus-real-fc-gui")) &&
            python_equal(wire_get(gateway, "expected_uid"), Wire(uid)) &&
            is_true(wire_get(gateway, "fresh")) && is_true(wire_get(gateway, "identity_ok")) &&
            is_false(wire_get(gateway, "armed")) && is_true(wire_get(gateway, "ground")) &&
            !wire_truthy(wire_get(gateway, "busy")) &&
            !wire_truthy(wire_get(gateway, "external")) &&
            !wire_truthy(wire_get(gateway, "owned_flight")) &&
            !wire_truthy(wire_get(owner, "owner_active")) &&
            !wire_truthy(wire_get(owner, "bench_active"));
        if (!idle_gateway) {
            throw ValueError("FC 연결부의 지상·무장 해제·송신권 상태 미확인");
        }
    }
    // SIGTERM 직전에 신원을 다시 읽는다 (PID 재사용 방지).
    const Wire again = find(inventory());
    if (again.size() != 1 || again.at(0).at("protected").get<bool>()) {
        throw ValueError("프로세스 식별이 변경됐습니다");
    }
    if (without_fc && !fc_link_absent()) {
        throw ValueError("FC 장치가 다시 연결됐습니다. 지상 상태 확인 뒤 다시 시도하세요");
    }
    if (::kill(static_cast<pid_t>(pid), SIGTERM) != 0) {
        throw os_error(errno);
    }
    const auto start = Clock::now();
    while (elapsed(start) < 3) {
        if (find(inventory()).empty()) {
            return Wire{{"accepted", true}, {"reason", "프로세스 종료 확인"}, {"process", item}};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return Wire{
        {"accepted", true}, {"reason", "종료 요청됨 · 아직 종료 확인 전"}, {"process", item}};
}

std::optional<int> Maintenance::diagnostic_pid() const {
    try {
        const auto run = options_.root / "run" / "target_diagnostic";
        long long pid = 0;
        if (!python_parse_int(read_file(run / "diagnostic.pid"), pid)) {
            return std::nullopt;
        }
        const auto arguments = process_arguments(static_cast<int>(pid));
        const std::string recorder =
            (options_.root / "src/argus/diagnostics/tracking/ground_recorder.py").string();
        if (arguments && contains(*arguments, recorder)) {
            return static_cast<int>(pid);
        }
        return std::nullopt;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

Wire Maintenance::diagnostic_status() const {
    const auto run = options_.root / "run" / "target_diagnostic";
    const auto base = options_.root / "diagnostics" / "target_manager";
    Wire result;
    try {
        const auto folder = resolved(read_file(run / "diagnostic-session.txt"));
        const std::string base_text = resolved(base).string();
        const std::string folder_text = folder.string();
        const bool inside = folder_text == base_text || folder_text.rfind(base_text + "/", 0) == 0;
        if (!inside) {
            throw ValueError("diagnostic output outside base");
        }
        result = Wire::parse(read_file(folder / "status.json"));
    } catch (const std::exception&) {
        result = Wire{{"reason", "not_started"}};
    }
    wire_set(result, "active", diagnostic_pid().has_value());
    wire_set(result, "available", true);
    wire_set(result, "flight_commands", false);
    return result;
}

Wire Maintenance::diagnostic_request(const Wire& action, const Wire& state) const {
    const std::filesystem::path& root = options_.root;
    const auto run = root / "run" / "target_diagnostic";
    const auto base = root / "diagnostics" / "target_manager";
    std::filesystem::create_directories(run);
    const int lock = open_for_append(run / "control.lock");
    struct Closer {
        int fd;
        Closer(const Closer&) = delete;
        Closer& operator=(const Closer&) = delete;
        Closer(Closer&&) = delete;
        Closer& operator=(Closer&&) = delete;
        ~Closer() { ::close(fd); }
    } const closer{lock};
    ::flock(lock, LOCK_EX);
    if (python_equal(action, Wire("status"))) {
        Wire out{{"accepted", true}};
        for (const auto& item : diagnostic_status().items()) {
            wire_set(out, item.key(), item.value());
        }
        return out;
    }
    if (python_equal(action, Wire("stop"))) {
        if (const auto pid = diagnostic_pid()) {
            if (::kill(*pid, SIGTERM) != 0) {
                throw os_error(errno);
            }
        }
        return Wire{{"accepted", true},
                    {"reason", "지상 진단 기록 종료 요청"},
                    {"diagnostic", diagnostic_status()}};
    }
    if (!python_equal(action, Wire("start"))) {
        throw ValueError("unknown diagnostic action");
    }
    const Wire owner =
        wire_truthy(wire_get(state, "follow_b")) ? state.at("follow_b") : Wire::object();
    const Wire manual = wire_get(state, "manual_yaw");
    const bool manual_active = wire_truthy(manual) && wire_truthy(wire_get(manual, "active"));
    const bool ground_idle =
        is_true(wire_get(state, "fresh")) && is_true(wire_get(state, "identity_ok")) &&
        is_false(wire_get(state, "armed")) && is_true(wire_get(state, "ground")) &&
        !wire_truthy(wire_get(state, "busy")) && !wire_truthy(wire_get(state, "owned_flight")) &&
        !wire_truthy(wire_get(state, "external")) &&
        !wire_truthy(wire_get(owner, "owner_active")) &&
        !wire_truthy(wire_get(owner, "bench_active")) && !manual_active;
    if (!ground_idle) {
        throw ValueError(
            "진단 기록은 신원 확인된 지상·무장 해제·비행 제어 중단 상태에서만 가능합니다");
    }
    if (diagnostic_pid()) {
        return Wire{{"accepted", true},
                    {"reason", "진단 기록 이미 실행 중"},
                    {"diagnostic", diagnostic_status()}};
    }
    const OrderedJson config = load_deployment_settings(root, current_environment());
    const Wire expected = wire_get(state, "expected_uid");
    if (!python_equal(expected, config.at("device").at("expected_fc_uid"))) {
        throw ValueError("진단 대상 FC UID 불일치");
    }
    const std::string uid = expected.get<std::string>();
    const Wire current =
        options_.forward(Wire{{"action", "status"}, {"expected_uid", uid}}, uid, 1200);
    validate_follow_status(current, uid);
    if (!is_false(wire_get(current, "active")) || !is_false(wire_get(current, "bench_active")) ||
        !python_equal(wire_get(current, "fc_epoch"), wire_get(owner, "fc_epoch"))) {
        throw ValueError("진단 중 추종 비활성 및 FC epoch 확인 필요");
    }
    const Wire mode = wire_get(current, "mode");
    const bool moving = python_equal(mode, Wire("moving"));
    if (!moving && !python_equal(mode, Wire("yaw"))) {
        throw ValueError("진단할 현재 추종 모드 미확인");
    }
    const auto binaries = resolved(config.at("binary_dir").get<std::string>());
    const auto controller = resolved(binaries / "argus_follow_controller");
    const auto diagnostic = resolved(binaries / "argus_target_diagnostic");
    std::error_code error;
    if (!std::filesystem::is_regular_file(diagnostic, error)) {
        throw ValueError("IR 진단 실행 파일이 없습니다. 해당 runtime을 빌드하세요");
    }
    const auto pid = matching_process(controller);
    const int port = endpoint_port(config.at("endpoints").at("operator"));
    if (!pid || !owns_listeners(*pid, {port})) {
        throw ValueError("실행 중인 IR 추종기와 포트 소유권을 확인할 수 없습니다");
    }
    const auto arguments = process_arguments(*pid);
    const std::string flag = moving ? "--moving-profile" : "--yaw-profile";
    std::filesystem::path profile;
    {
        const auto found = arguments ? std::find(arguments->begin(), arguments->end(), flag)
                                     : std::vector<std::string>::const_iterator();
        if (!arguments || found == arguments->end() || std::next(found) == arguments->end()) {
            throw ValueError("실행 중인 추종 프로필을 확인할 수 없습니다");
        }
        std::error_code canonical_error;
        profile = std::filesystem::canonical(*std::next(found), canonical_error);
        if (canonical_error) {
            throw ValueError("실행 중인 추종 프로필을 확인할 수 없습니다");
        }
    }
    const std::string config_root = resolved(root / "run/config").string();
    if (profile.string().rfind(config_root + "/", 0) != 0 && profile.string() != config_root) {
        throw ValueError("진단 프로필이 IR runtime 범위 밖입니다");
    }
    std::array<char, 32> stamp{};
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    ::localtime_r(&now, &local);
    std::strftime(stamp.data(), stamp.size(), "%Y%m%d_%H%M%S", &local);
    const auto monotonic =
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
            .count();
    const auto folder = base / (std::string(stamp.data()) + "_" + std::to_string(monotonic));
    std::filesystem::create_directories(base);
    const auto recorder = root / "src/argus/diagnostics/tracking/ground_recorder.py";
    const std::vector<std::string> command{options_.python,
                                           "-B",
                                           recorder.string(),
                                           "--output",
                                           folder.string(),
                                           "--profile",
                                           profile.string(),
                                           "--binary",
                                           diagnostic.string(),
                                           "--mode",
                                           mode.get<std::string>(),
                                           "--tracks",
                                           config.at("endpoints").at("tracks"),
                                           "--images",
                                           config.at("endpoints").at("vision")};
    const ChildProcess child = spawn_logged(root, run / "diagnostic.log", command);
    std::ofstream(run / "diagnostic.pid", std::ios::binary | std::ios::trunc) << child.pid();
    std::ofstream(run / "diagnostic-session.txt", std::ios::binary | std::ios::trunc)
        << folder.string();
    return Wire{{"accepted", true},
                {"reason", "지상 진단 준비 · 대상만 선택하세요"},
                {"output", folder.string()},
                {"flight_commands", false}};
}

}  // namespace argus::gateway
