#include "argus/gateway/service/include/preview_supervisor.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <zmq.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <sstream>
#include <thread>

#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/service/include/preview_ready.hpp"
#include "argus/gateway/service/include/process_supervisor.hpp"
#include "argus/gateway/service/include/vision_protobuf.hpp"
#include "argus/gateway/settings/include/deployment_settings.hpp"

namespace argus::gateway {

namespace {

using Clock = std::chrono::steady_clock;

constexpr int kVideoPort = 25002;
// 2026-11-25 batch 6c: 영상 바이너리 상한 16 MiB (team_wire kMaxFrameBinaryBytes 와 같은 값)
constexpr std::size_t kMaxPreviewBinaryBytes = std::size_t{16} << 20U;

double seconds_since(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

void sleep_s(double seconds) {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

std::string read_file(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw os_error(errno, path.string());
    }
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::vector<std::string> fields_of(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> out;
    std::string field;
    while (stream >> field) {
        out.push_back(field);
    }
    return out;
}

int endpoint_port(const std::string& endpoint) {
    return std::stoi(endpoint.substr(endpoint.rfind(':') + 1));
}

// 원본 bytes.decode('utf8', 'replace') 후 끝 n글자
std::string decoded_tail(const std::string& bytes, std::size_t characters) {
    std::u32string text;
    std::size_t i = 0;
    while (i < bytes.size()) {
        const auto lead = static_cast<unsigned char>(bytes.at(i));
        std::size_t extra = 0;
        char32_t code = lead;
        if (lead >= 0xc2 && lead <= 0xdf) {
            extra = 1;
            code = lead & 0x1fU;
        } else if (lead >= 0xe0 && lead <= 0xef) {
            extra = 2;
            code = lead & 0x0fU;
        } else if (lead >= 0xf0 && lead <= 0xf4) {
            extra = 3;
            code = lead & 0x07U;
        } else if (lead >= 0x80) {
            text.push_back(0xfffd);
            ++i;
            continue;
        }
        bool valid = i + extra < bytes.size();
        for (std::size_t k = 1; valid && k <= extra; ++k) {
            const auto next = static_cast<unsigned char>(bytes.at(i + k));
            valid = (next & 0xc0U) == 0x80;
            code = (code << 6U) | (next & 0x3fU);
        }
        if (!valid) {
            text.push_back(0xfffd);
            ++i;
            continue;
        }
        text.push_back(code);
        i += extra + 1;
    }
    if (text.size() > characters) {
        text.erase(0, text.size() - characters);
    }
    return u32_to_utf8(text);
}

// 원본 open('a') + fcntl.flock(LOCK_EX | LOCK_NB)를 시간 안에 되풀이
class FileLock {
public:
    explicit FileLock(const std::filesystem::path& path) : fd_(open_for_append(path)) {}
    FileLock(const FileLock&) = delete;
    FileLock& operator=(const FileLock&) = delete;
    FileLock(FileLock&&) = delete;
    FileLock& operator=(FileLock&&) = delete;
    ~FileLock() { ::close(fd_); }
    bool try_lock() const { return ::flock(fd_, LOCK_EX | LOCK_NB) == 0; }

private:
    int fd_{-1};
};

}  // namespace

std::set<std::string> udp_inodes(int port) {
    std::set<std::string> result;
    for (const char* name : {"udp", "udp6"}) {
        const std::filesystem::path table = std::filesystem::path("/proc/net") / name;
        std::error_code error;
        if (!std::filesystem::exists(table, error)) {
            continue;
        }
        std::istringstream lines(read_file(table));
        std::string line;
        std::getline(lines, line);
        while (std::getline(lines, line)) {
            const auto fields = fields_of(line);
            if (fields.size() > 9 &&
                std::stoi(fields.at(1).substr(fields.at(1).rfind(':') + 1), nullptr, 16) == port) {
                result.insert(fields.at(9));
            }
        }
    }
    return result;
}

std::vector<std::pair<int, std::string>> port_owners(const std::set<std::string>& inodes) {
    std::vector<std::pair<int, std::string>> found;
    std::error_code error;
    for (const auto& process : std::filesystem::directory_iterator("/proc", error)) {
        const std::string name = process.path().filename().string();
        if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) {
            continue;
        }
        std::error_code fd_error;
        bool owns = false;
        for (const auto& fd :
             std::filesystem::directory_iterator(process.path() / "fd", fd_error)) {
            std::error_code link_error;
            const std::string target =
                std::filesystem::read_symlink(fd.path(), link_error).string();
            if (!link_error && target.rfind("socket:[", 0) == 0 && target.size() > 9 &&
                inodes.count(target.substr(8, target.size() - 9)) != 0) {
                owns = true;
                break;
            }
        }
        if (fd_error || !owns) {
            continue;
        }
        const auto arguments = process_arguments(std::stoi(name));
        if (!arguments) {
            continue;
        }
        std::string command;
        for (std::size_t i = 0; i < arguments->size(); ++i) {
            command += (i == 0 ? "" : " ") + arguments->at(i);
        }
        found.emplace_back(std::stoi(name), decoded_tail(command, command.size()));
    }
    return found;
}

Wire ensure_main_gcs_video(const std::filesystem::path& root_in, const std::string& bus,
                           const std::string& tracks, const std::string& python) {
    const std::filesystem::path root = resolved(root_in);
    static const std::regex kLocal(R"(tcp://127\.0\.0\.1:[0-9]{1,5})");
    for (const std::string& endpoint : {bus, tracks}) {
        if (!std::regex_match(endpoint, kLocal)) {
            throw ValueError("영상·추적 endpoint는 로컬 TCP 주소여야 합니다");
        }
        const int port = endpoint_port(endpoint);
        if (port < 1 || port > 65535) {
            throw ValueError("영상·추적 포트 범위 오류");
        }
    }
    const std::filesystem::path run = root / "run/preview";
    std::filesystem::create_directories(run);
    const FileLock lock(run / "main-gcs-video.prepare.lock");
    const auto lock_start = Clock::now();
    while (!lock.try_lock()) {
        if (seconds_since(lock_start) >= 15.0) {
            throw PreviewError("다른 영상 중계 준비가 진행 중입니다");
        }
        sleep_s(.1);
    }
    const auto inodes = udp_inodes(kVideoPort);
    if (!inodes.empty()) {
        const auto owners = port_owners(inodes);
        std::vector<int> known;
        for (const auto& [pid, command] : owners) {
            const bool sender = command.find("main_gcs_video_entry.py") != std::string::npos ||
                                command.find("vision_bus_bridge.py") != std::string::npos ||
                                command.find("argus_main_gcs_video") != std::string::npos;
            if (sender && command.find(bus) != std::string::npos &&
                command.find(tracks) != std::string::npos) {
                known.push_back(pid);
            }
        }
        if (known.size() != 1 || owners.size() != 1) {
            throw PreviewError(
                "UDP 25002를 다른 프로세스가 사용 중이거나 소유자를 확인할 수 없습니다");
        }
        return Wire{
            {"ready", true}, {"reused", true}, {"pid", known.front()}, {"port", kVideoPort}};
    }
    // C++ 송신기(영상+박스 v2)가 설치돼 있으면 그것을, 없으면 기존 Python 송신기를 쓴다.
    const std::filesystem::path native = root / "bin/argus_main_gcs_video";
    std::vector<std::string> command;
    std::error_code error;
    if (std::filesystem::is_regular_file(native, error) && ::access(native.c_str(), X_OK) == 0) {
        command = {native.string(), "--bus", bus, "--tracks", tracks, "--video-port", "25002"};
    } else {
        const auto entry = root / "src/argus/runtime/streaming/main_gcs_video_entry.py";
        const auto bridge = root / "src/argus/runtime/streaming/main_gcs_legacy_bridge.py";
        if (!std::filesystem::is_regular_file(entry, error) ||
            !std::filesystem::is_regular_file(bridge, error)) {
            throw PreviewError("메인 GUI 영상 중계 파일이 없습니다");
        }
        command = {python,        "-B",    "-u", entry.string(), "--root",
                   root.string(), "--bus", bus,  "--tracks",     tracks};
    }
    const std::filesystem::path log = run / "main-gcs-video.log";
    ChildProcess process = spawn_logged(root, log, command);
    const auto start = Clock::now();
    while (seconds_since(start) < 10.0) {
        if (process.poll()) {
            break;
        }
        const auto current = udp_inodes(kVideoPort);
        if (!current.empty()) {
            for (const auto& [pid, owner_command] : port_owners(current)) {
                (void)owner_command;
                if (pid == process.pid()) {
                    return Wire{
                        {"ready", true}, {"reused", false}, {"pid", pid}, {"port", kVideoPort}};
                }
            }
        }
        sleep_s(.1);
    }
    if (!process.poll()) {
        process.terminate();
        if (!process.wait(2)) {
            process.kill();
            process.wait(2);
        }
    }
    const std::string all = read_file(log);
    const std::string tail =
        all.size() > process.log_start() ? all.substr(process.log_start()) : std::string();
    throw PreviewError("메인 GUI 영상 중계 시작 실패: " + decoded_tail(tail, 1200));
}

// 2026-11-25 batch 4: 팀 vision 6374 전환. argus_preview_bus (5555 -> 5558 프록시) 는 없앴고 6374
// 는 argus-onboard 가 bind 한다. Python ensure() (preview_producer_supervisor.py) 와 같은 규칙:
//   1. 실행 파일은 argus-onboard 와 run_pi_producer.sh 만 필요하다.
//   2. 생산자가 없는데 6374 가 이미 listening 이면 다른 프로그램이 쓰는 것으로 보고 거부한다.
//   3. 생산자를 재사용할 때는 바로, 새로 시작했을 때는 첫 실제 프레임 뒤에 그 pid 가 6374 listener
//   를
//      가지는지 본다. 예전 argus-onboard (5555 에 connect, 6374 를 bind 하지 않음) 는 시간 초과를
//      기다리지 않고 바로 거부한다.
//   4. 첫 실제 프레임: [이름][protobuf][binary] 를 이름 정확 일치와 3 부분으로 받고 Pi 값으로
//   되살린 뒤
//      preview_ready.hpp 판정을 그대로 쓴다 (pi_ 출처가 없는 팀 전용 본문은 준비로 보지 않는다).
Wire ensure_preview(const std::filesystem::path& root, double timeout_s) {
    const OrderedJson settings = load_deployment_settings(root, current_environment());
    const std::string vision = settings.at("endpoints").at("vision");
    const int vision_port = endpoint_port(vision);
    const std::filesystem::path run = root / "run/preview";
    std::filesystem::create_directories(run);
    const FileLock lock(run / "prepare.lock");
    const auto start = Clock::now();
    while (!lock.try_lock()) {
        if (seconds_since(start) >= timeout_s) {
            throw PreviewError("다른 영상 연결의 준비 시간이 초과됐습니다.");
        }
        sleep_s(.1);
    }
    const std::filesystem::path binaries = settings.at("binary_dir").get<std::string>();
    const auto producer = binaries / "argus-onboard";
    for (const auto& required : {producer, root / "run_pi_producer.sh"}) {
        std::error_code error;
        if (!std::filesystem::is_regular_file(required, error)) {
            throw PreviewError("Pi 영상 실행 파일이 없습니다: " + required.string());
        }
    }
    const std::string port_text = std::to_string(vision_port);
    auto producer_pid = matching_process(producer);
    const bool reused = producer_pid.has_value();
    std::optional<ChildProcess> process;
    if (!producer_pid) {
        if (tcp_listening(vision_port)) {
            throw PreviewError("영상 포트 " + port_text + "를 다른 프로그램이 사용 중입니다.");
        }
        std::cerr << "Pi 카메라와 YOLOX 시작 중\n" << std::flush;
        process =
            start_process(root, run, "producer", {"bash", (root / "run_pi_producer.sh").string()});
        producer_pid = process->pid();
    } else {
        if (!owns_listeners(*producer_pid, {vision_port})) {
            throw PreviewError("실행 중인 Pi 영상 생산자가 영상 포트 " + port_text +
                               "를 소유하지 않습니다 (예전 버전일 수 있음). 생산자를 멈춘 뒤 다시 "
                               "연결하세요.");
        }
        std::cerr << "실행 중인 Pi 영상 생산자를 재사용합니다.\n" << std::flush;
    }
    void* context = zmq_ctx_new();
    void* subscriber = zmq_socket(context, ZMQ_SUB);
    const auto finish = [&] {
        zmq_close(subscriber);
        zmq_ctx_term(context);
    };
    try {
        const int linger = 0;
        zmq_setsockopt(subscriber, ZMQ_LINGER, &linger, sizeof(linger));
        // 2026-11-25 batch 6c: ZMQ_MAXMSGSIZE 는 두지 않는다 (libzmq 4.3.5 는 넘는 프레임을 보낸
        // 연결을 끊고 다시 맺지 않는다). 바이너리 상한 16 MiB 는 받은 뒤 보고 그 메시지만
        // 건너뛴다. 본문 상한 (1 MiB) 은 vision_msgpack (team_wire vision_body) 이 본다
        // 2026-11-23: Python ensure()처럼 JPEG 미리보기와 흑백 영상을 함께 구독한다
        // (preview_ready.hpp).
        zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, kPreviewOverlayTopic.data(),
                       kPreviewOverlayTopic.size());
        zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, kPreviewGray8Topic.data(),
                       kPreviewGray8Topic.size());
        zmq_connect(subscriber, vision.c_str());
        while (seconds_since(start) < timeout_s) {
            if (process && process->poll()) {
                const std::string all = read_file(run / "producer.log");
                const std::string tail = decoded_tail(
                    all.size() > process->log_start() ? all.substr(process->log_start()) : "",
                    2000);
                if (tail.find("NO_CAMERA") != std::string::npos) {
                    throw PreviewError(
                        "Pi에서 카메라를 찾지 못했습니다. 카메라 USB 연결, 케이블, 전원을 "
                        "확인하세요.");
                }
                if (tail.find("Managed SDK NUC failed") != std::string::npos) {
                    throw PreviewError(
                        "카메라 보정(NUC)에 실패했습니다. 카메라 연결과 전원을 확인하세요.");
                }
                if (tail.find("vision bus ownership failed") != std::string::npos) {
                    std::string message = "Pi 영상 생산자가 영상 포트 " + port_text;
                    message += "를 열지 못했습니다:\n";
                    message += tail;
                    throw PreviewError(message);
                }
                throw PreviewError("Pi 영상 생산자 시작 실패:\n" + tail);
            }
            zmq_pollitem_t item{subscriber, 0, ZMQ_POLLIN, 0};
            if (zmq_poll(&item, 1, 200) <= 0) {
                continue;
            }
            std::vector<Bytes> parts;
            for (;;) {
                zmq_msg_t message;
                zmq_msg_init(&message);
                zmq_msg_recv(&message, subscriber, 0);
                const auto* data = static_cast<const std::uint8_t*>(zmq_msg_data(&message));
                parts.emplace_back(
                    data, std::next(data, static_cast<std::ptrdiff_t>(zmq_msg_size(&message))));
                const bool more = zmq_msg_more(&message) != 0;
                zmq_msg_close(&message);
                if (!more) {
                    break;
                }
            }
            const std::string topic(parts.at(0).begin(), parts.at(0).end());
            if (topic != kPreviewOverlayTopic && topic != kPreviewGray8Topic) {
                continue;
            }
            if (parts.size() == 3 && parts.at(2).size() > kMaxPreviewBinaryBytes) {
                continue;  // 2026-11-25 batch 6c: 바이너리 상한 초과 (연결은 그대로)
            }
            Bytes packed;
            std::string body_error;
            if (vision_msgpack(parts, packed, body_error) != VisionFrame::Ok) {
                continue;  // 형식이 틀리거나 팀 전용 본문 (수와 로그는 team_wire vision_body)
            }
            const Wire metadata = msgpack_unpack(packed);
            if (preview_frame_ready(topic, metadata, parts.at(2))) {
                if (!owns_listeners(*producer_pid, {vision_port})) {
                    throw PreviewError("영상 포트 " + port_text +
                                       "를 Pi 영상 생산자가 아닌 프로세스가 소유하고 있습니다.");
                }
                // 실제 프레임을 본 뒤 영상 중계를 확인한다 (카메라 준비 시간을 쓰지 않게).
                Wire video = Wire::object();
                std::string video_error;
                try {
                    const char* configured = std::getenv(  // NOLINT(concurrency-mt-unsafe)
                        "THESSEN_PERCEPTION_PYTHON");
                    const std::string interpreter = configured != nullptr
                                                        ? configured
                                                        : (root / ".venv-rpi/bin/python").string();
                    video = ensure_main_gcs_video(
                        root, vision, settings.at("endpoints").at("tracks"), interpreter);
                } catch (const std::exception& failure) {
                    video_error = failure.what();
                    std::cerr << "메인 GUI 영상 중계 준비 실패: " << video_error << '\n'
                              << std::flush;
                }
                Wire status = Wire::object();
                wire_set(status, "producer_pid", *producer_pid);
                wire_set(status, "reused", reused);
                wire_set(status, "frame_id", wire_get(metadata, "frame_id"));
                wire_set(status, "source", wire_get(metadata, "source"));
                wire_set(status, "synthetic", false);
                wire_set(status, "fc_started", false);
                wire_set(status, "main_gcs_video", video);
                wire_set(status, "main_gcs_video_error", video_error);
                std::ofstream(run / "last-ready.json", std::ios::binary | std::ios::trunc)
                    << python_json_dumps_indent(status, 2, false);
                std::cerr << "Pi 실제 영상 준비 완료, UDP 중계 연결 중\n" << std::flush;
                finish();
                return status;
            }
        }
        throw PreviewError(
            "Pi 실제 영상 수신 대기 시간 초과, 카메라와 run/preview/producer.log 확인");
    } catch (...) {
        finish();
        throw;
    }
}

}  // namespace argus::gateway
