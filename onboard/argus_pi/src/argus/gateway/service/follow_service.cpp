#include "argus/gateway/service/include/follow_service.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <thread>
#include <utility>

#include "argus/gateway/follow/include/follow_proxy.hpp"
#include "argus/gateway/service/include/preview_supervisor.hpp"
#include "argus/gateway/service/include/process_supervisor.hpp"
#include "argus/gateway/settings/include/deployment_settings.hpp"

namespace argus::gateway {

namespace {

using Clock = std::chrono::steady_clock;

double elapsed(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

void sleep_s(double seconds) {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

int endpoint_port(const std::string& endpoint) {
    return std::stoi(endpoint.substr(endpoint.rfind(':') + 1));
}

bool is_file(const std::filesystem::path& path) {
    std::error_code error;
    return std::filesystem::is_regular_file(path, error);
}

bool is_true(const Wire& value) { return value.is_boolean() && value.get<bool>(); }

bool is_false(const Wire& value) { return value.is_boolean() && !value.get<bool>(); }

// 원본 _stop_verified_process: PID를 고정(pidfd)한 뒤 같은 실행 파일일 때만 SIGTERM.
// 강제 종료하지 않는다.
void stop_verified_process(pid_t pid, const std::filesystem::path& binary,
                           const std::function<void()>& admission) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): Linux syscall은 가변 인수 함수다
    const auto descriptor = static_cast<int>(::syscall(SYS_pidfd_open, pid, 0));
    if (descriptor < 0) {
        if (errno == ENOSYS) {
            throw ValueError("안전한 프로세스 교체 기능 미지원 · 지상에서 명시 종료 필요");
        }
        throw os_error(errno);
    }
    struct Closer {
        int fd;
        Closer(const Closer&) = delete;
        Closer& operator=(const Closer&) = delete;
        Closer(Closer&&) = delete;
        Closer& operator=(Closer&&) = delete;
        ~Closer() { ::close(fd); }
    } const closer{descriptor};
    std::error_code error;
    const auto exe = std::filesystem::read_symlink(
        std::filesystem::path("/proc") / std::to_string(pid) / "exe", error);
    if (error || resolved(exe) != resolved(binary)) {
        throw ValueError("교체 대상 실행 파일 불일치");
    }
    admission();
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): Linux syscall
    if (::syscall(SYS_pidfd_send_signal, descriptor, SIGTERM, nullptr, 0) != 0) {
        throw os_error(errno);
    }
    const auto start = Clock::now();
    while (elapsed(start) < 3) {
        pollfd item{descriptor, POLLIN, 0};
        if (::poll(&item, 1, 50) > 0) {
            reap_children();
            return;
        }
    }
    throw ValueError("추종 프로세스 정상 종료 확인 시간 초과 · 강제 종료 안 함");
}

}  // namespace

bool uses_profile(int pid, const std::string& flag, const std::string& expected) {
    const auto arguments = process_arguments(pid);
    if (!arguments) {
        return false;
    }
    std::optional<std::size_t> last;
    for (std::size_t i = 0; i < arguments->size(); ++i) {
        if (arguments->at(i) == flag) {
            last = i;
        }
    }
    return last && *last + 1 < arguments->size() && arguments->at(*last + 1) == expected;
}

FollowService::FollowService(FollowServiceOptions options) : options_(std::move(options)) {
    if (!options_.preview) {
        options_.preview = ensure_preview;
    }
}

Wire FollowService::status(const std::string& uid) const {
    const Wire result =
        options_.forward(Wire{{"action", "status"}, {"expected_uid", uid}}, uid, 350);
    if (!is_true(wire_get(result, "accepted")) ||
        !python_equal(wire_get(result, "expected_uid"), Wire(uid))) {
        throw ValueError("vMy 준비 응답 UID/승인 불일치");
    }
    validate_observation_status(result, uid);
    return result;
}

ProfileServices FollowService::services() {
    ProfileServices services;
    services.prepare = [this](const std::string& uid, const Wire& selection, bool reconfigure,
                              const std::function<void()>& admission) {
        return prepare(uid, selection.at("environment").get<std::string>(),
                       selection.at("controller").get<std::string>(), reconfigure, admission);
    };
    services.status = [this](const std::string& uid) { return status(uid); };
    return services;
}

Wire FollowService::prepare(const std::string& uid, const std::string& environment,
                            const std::string& controller, bool reconfigure,
                            const std::function<void()>& admission) {
    if (environment != "indoor" && environment != "outdoor") {
        throw ValueError("추종 환경은 indoor 또는 outdoor여야 합니다");
    }
    if (reconfigure && !admission) {
        throw ValueError("프로필 교체에는 지상 상태 재검사가 필요합니다");
    }
    EnvironmentMap environment_values = current_environment();
    environment_values["ARGUS_FOLLOW_ENVIRONMENT"] = environment;
    environment_values["ARGUS_FOLLOW_CONTROLLER"] = controller;
    OrderedJson config;
    try {
        config = load_deployment_settings(resolved(options_.root), environment_values);
    } catch (const SettingsError& error) {
        throw ValueError(error.what());
    }
    const std::unique_lock<std::mutex> lock(lock_, std::try_to_lock);
    if (!lock.owns_lock()) {
        throw ValueError("다른 vMy 준비 요청 진행 중");
    }
    try {
        return prepare_locked(std::move(config), uid, environment, controller, reconfigure,
                              admission);
    } catch (const OsError& error) {
        throw ValueError(error.what());
    } catch (const PreviewError& error) {
        throw ValueError(error.what());
    } catch (const std::filesystem::filesystem_error& error) {
        throw ValueError(error.what());
    }
}

Wire FollowService::prepare_locked(OrderedJson config, const std::string& uid,
                                   const std::string& environment, const std::string& controller,
                                   bool reconfigure, const std::function<void()>& admission) const {
    const std::filesystem::path root = resolved(options_.root);
    const int operator_port = endpoint_port(config.at("endpoints").at("operator"));
    const int tracks_port = endpoint_port(config.at("endpoints").at("tracks"));
    const std::filesystem::path run = root / "run/follow_b";
    std::filesystem::create_directories(run);
    const int lock_fd = open_for_append(run / "prepare.lock");
    struct LockFile {
        int fd;
        LockFile(const LockFile&) = delete;
        LockFile& operator=(const LockFile&) = delete;
        LockFile(LockFile&&) = delete;
        LockFile& operator=(LockFile&&) = delete;
        ~LockFile() { ::close(fd); }
    } const lock_file{lock_fd};
    if (::flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
        throw ValueError("다른 vMy 준비 요청 진행 중");
    }
    config = materialize_deployment_settings(config);
    const std::filesystem::path binaries = config.at("binary_dir").get<std::string>();
    const auto binary = binaries / "argus_follow_controller";
    const auto tracker = binaries / "argus_follow_tracker";
    const std::filesystem::path camera = config.at("paths").at("camera").get<std::string>();
    for (const auto& path : {binary, tracker, camera, root / "run_follow_b_controller.sh",
                             root / "run_follow_b_tracker.sh"}) {
        if (!is_file(path)) {
            throw ValueError("Pi vMy 설치 파일 없음: " + path.string());
        }
    }
    const auto profile_path = [&config](const char* key) {
        return config.at("paths").at(key).get<std::string>();
    };
    // 응답 없는 기존 소유자는 거부한다. 죽이거나 다시 띄우지 않는다.
    auto runtime_pid = matching_process(binary);
    std::optional<pid_t> tracker_pid = reconfigure ? matching_process(tracker) : std::nullopt;
    bool mismatch = false;
    if (runtime_pid) {
        for (const char* mode : {"moving", "yaw"}) {
            mismatch =
                mismatch || !uses_profile(*runtime_pid, std::string("--") + mode + "-profile",
                                          profile_path(mode));
        }
    }
    // 2026-11-12: 장착각 모드가 바뀌면 카메라 보정 파일도 바뀌므로 함께 비교한다.
    const auto tracker_uses_paths = [&](int pid) {
        return uses_profile(pid, "--measurement", profile_path("measurement")) &&
               uses_profile(pid, "--calibration", camera.string());
    };
    const bool tracker_mismatch = tracker_pid && !tracker_uses_paths(*tracker_pid);
    if (reconfigure && (mismatch || tracker_mismatch)) {
        admission();
        if (runtime_pid) {
            const Wire state = status(uid);
            const Wire mission = wire_get(state, "mission_state");
            const bool idle = python_equal(mission, Wire("IDLE")) ||
                              python_equal(mission, Wire("HOLD_WAIT")) ||
                              python_equal(mission, Wire("MANUAL"));
            if (!is_true(wire_get(state, "profile_replace_ready")) ||
                !is_false(wire_get(state, "active")) ||
                !is_false(wire_get(state, "bench_active")) || !idle) {
                throw ValueError("기존 추종기 중단 상태 확인 필요 · 교체하지 않습니다");
            }
            // 바뀌지 않은 추종기도 추적기 신원을 쓴다. 측정 설정이 바뀌면 둘을 함께 바꾼다.
            stop_verified_process(*runtime_pid, binary, admission);
            runtime_pid.reset();
        }
        if (tracker_pid && tracker_mismatch) {
            stop_verified_process(*tracker_pid, tracker, admission);
        }
        admission();
    }
    if (runtime_pid) {
        for (const char* mode : {"moving", "yaw"}) {
            if (!uses_profile(*runtime_pid, std::string("--") + mode + "-profile",
                              profile_path(mode))) {
                throw ValueError(
                    "기존 추종기 환경이 다릅니다. 지상에서 명시 종료 후 환경을 선택하세요");
            }
        }
        status(uid);
        if (!owns_listeners(*runtime_pid, {operator_port})) {
            throw ValueError("B 추종 포트 소유 프로세스 불일치");
        }
    } else if (tcp_listening(operator_port)) {
        throw ValueError("vMy 6373 포트를 다른 프로세스가 사용 중");
    }
    const auto start = Clock::now();
    tracker_pid = matching_process(tracker);
    if (tracker_pid && !tracker_uses_paths(*tracker_pid)) {
        throw ValueError("기존 추적기 환경이 다릅니다. 지상에서 명시 종료 후 환경을 선택하세요");
    }
    const std::vector<std::string> launch_environment{
        "env", "ARGUS_FOLLOW_ENVIRONMENT=" + environment, "ARGUS_FOLLOW_CONTROLLER=" + controller};
    if (!tracker_pid) {
        if (tcp_listening(tracks_port)) {
            throw ValueError("추적기 " + std::to_string(tracks_port) +
                             " 포트를 다른 프로세스가 사용 중");
        }
        if (admission) {
            admission();
        }
        std::vector<std::string> command = launch_environment;
        command.insert(command.end(), {"bash", (root / "run_follow_b_tracker.sh").string()});
        ChildProcess child = start_process(root, run, "tracker", command);
        while (!tcp_listening(tracks_port)) {
            if (child.poll() || elapsed(start) >= options_.timeout_s) {
                throw ValueError("Pi 추적기 시작 실패 · run/follow_b/tracker.log 확인");
            }
            sleep_s(.1);
        }
        tracker_pid = matching_process(tracker);
        if (!tracker_pid) {
            throw ValueError("B 추적기 실행 파일 소유 프로세스 미확인");
        }
    } else if (!tcp_listening(tracks_port)) {
        throw ValueError("기존 추적기가 미응답 · 자동 재시작하지 않음");
    }
    if (!owns_listeners(*tracker_pid, {tracks_port})) {
        throw ValueError("B 추적기 포트 소유 프로세스 불일치");
    }
    const bool reused = runtime_pid.has_value();
    std::optional<ChildProcess> child;
    if (!runtime_pid) {
        if (admission) {
            admission();
        }
        std::vector<std::string> command = launch_environment;
        command.insert(command.end(), {"bash", (root / "run_follow_b_controller.sh").string(),
                                       "--camera-profile", camera.string(), "--mode", "yaw"});
        child = start_process(root, run, "runtime", command);
        runtime_pid = child->pid();
    }
    Wire current_status;
    for (;;) {
        try {
            current_status = status(uid);
            runtime_pid = matching_process(binary);
            if (!runtime_pid || !owns_listeners(*runtime_pid, {operator_port})) {
                throw ValueError("B 추종 실행 파일/포트 소유 프로세스 미확인");
            }
            break;
        } catch (const ValueError&) {
            if (!child || child->poll() || elapsed(start) >= options_.timeout_s) {
                throw ValueError("vMy C++ 준비 미확인 · run/follow_b/runtime.log 확인");
            }
            sleep_s(.15);
        }
    }
    // 카메라가 빠져 있어도 대상 관리는 쓸 수 있다.
    Wire preview = Wire::object();
    bool preview_ready = false;
    std::string preview_error;
    try {
        preview = options_.preview(root, std::min(35.0, options_.timeout_s));
        preview_ready = true;
        const Wire video_error = wire_get(preview, "main_gcs_video_error");
        preview_error = video_error.is_string() ? video_error.get<std::string>() : "";
    } catch (const ValueError& error) {
        preview_error = error.what();
    } catch (const PreviewError& error) {
        preview_error = error.what();
    } catch (const OsError& error) {
        preview_error = error.what();
    }
    Wire result = Wire::object();
    wire_set(result, "ok", true);
    wire_set(result, "prepared", true);
    wire_set(result, "runtime_pid", *runtime_pid);
    wire_set(result, "follow_environment", environment);
    wire_set(result, "follow_controller", controller);
    wire_set(result, "tracker_pid", *tracker_pid);
    wire_set(result, "reused", reused);
    wire_set(result, "status", current_status);
    wire_set(result, "preview", preview_ready ? preview : Wire::object());
    wire_set(result, "preview_ready", preview_ready);
    wire_set(result, "reason",
             preview_error.empty()
                 ? std::string("영상·추적·대상 관리 준비 완료 · 대상 선택 후 추종 시작")
                 : preview_error);
    wire_set(result, "flight_command_sent", false);
    return result;
}

}  // namespace argus::gateway
