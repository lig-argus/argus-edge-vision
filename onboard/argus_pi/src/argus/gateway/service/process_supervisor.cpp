#include "argus/gateway/service/include/process_supervisor.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <system_error>
#include <thread>

#include "argus/gateway/common/include/msgpack_codec.hpp"

namespace argus::gateway {

namespace {

bool digits(const std::string& text) {
    return !text.empty() &&
           std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; });
}

std::string read_text(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw os_error(errno, path.string());
    }
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

std::vector<std::string> split_whitespace(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> out;
    std::string field;
    while (stream >> field) {
        out.push_back(field);
    }
    return out;
}

// 원본 subprocess는 끝난 Popen을 나중에 정리한다. 여기서는 띄운 자식 PID를 기억해 둔다.
std::mutex& children_mutex() {
    static std::mutex mutex;
    return mutex;
}
std::set<pid_t>& children() {
    static std::set<pid_t> pids;
    return pids;
}

}  // namespace

OsError os_error(int code, const std::string& filename) {
    std::string text = "[Errno " + std::to_string(code) + "] " + std::strerror(code);
    if (!filename.empty()) {
        text += ": " + python_repr_value(Wire(filename));
    }
    OsError error(text);
    return error;
}

int open_for_append(const std::filesystem::path& path) {
    // POSIX open()은 mode 인수 때문에 가변 인수 함수다
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (fd < 0) {
        throw os_error(errno, path.string());
    }
    return fd;
}

std::filesystem::path resolved(const std::filesystem::path& path) {
    std::error_code error;
    const auto absolute = std::filesystem::absolute(path, error);
    const auto canonical = std::filesystem::weakly_canonical(error ? path : absolute, error);
    return error ? path : canonical;
}

bool tcp_listening(int port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        return false;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    // POSIX 소켓 API는 sockaddr*를 받는다
    const int started = ::connect(fd, reinterpret_cast<const sockaddr*>(&address),  // NOLINT
                                  sizeof(address));
    bool connected = started == 0;
    if (!connected && errno == EINPROGRESS) {
        pollfd item{fd, POLLOUT, 0};
        if (::poll(&item, 1, 200) > 0) {
            int failure = 0;
            socklen_t length = sizeof(failure);
            ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &failure, &length);
            connected = failure == 0;
        }
    }
    ::close(fd);
    return connected;
}

std::optional<pid_t> matching_process(const std::filesystem::path& executable) {
    const auto expected = resolved(executable);
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator("/proc", error)) {
        const std::string name = entry.path().filename().string();
        if (!digits(name)) {
            continue;
        }
        std::error_code link_error;
        const auto target = std::filesystem::read_symlink(entry.path() / "exe", link_error);
        if (link_error) {
            continue;
        }
        if (resolved(target) == expected) {
            return static_cast<pid_t>(std::stol(name));
        }
    }
    return std::nullopt;
}

bool owns_listeners(pid_t pid, const std::set<int>& ports) {
    try {
        std::set<std::string> sockets;
        const auto fd_dir = std::filesystem::path("/proc") / std::to_string(pid) / "fd";
        for (const auto& entry : std::filesystem::directory_iterator(fd_dir)) {
            std::error_code error;
            const auto target = std::filesystem::read_symlink(entry.path(), error);
            if (!error) {
                sockets.insert(target.string());
            }
        }
        std::set<int> found;
        for (const char* table : {"tcp", "tcp6"}) {
            std::istringstream lines(read_text(std::filesystem::path("/proc/net") / table));
            std::string line;
            std::getline(lines, line);  // 제목 줄
            while (std::getline(lines, line)) {
                const auto fields = split_whitespace(line);
                if (fields.size() < 10) {
                    return false;  // 원본 IndexError → False
                }
                if (fields.at(3) == "0A" && sockets.count("socket:[" + fields.at(9) + "]") != 0) {
                    const std::string& local = fields.at(1);
                    found.insert(std::stoi(local.substr(local.rfind(':') + 1), nullptr, 16));
                }
            }
        }
        return std::includes(found.begin(), found.end(), ports.begin(), ports.end());
    } catch (const std::exception&) {
        return false;  // 원본 (OSError, IndexError, ValueError) → False
    }
}

std::optional<int> ChildProcess::poll() {
    if (returncode_) {
        return returncode_;
    }
    int status = 0;
    const pid_t done = ::waitpid(pid_, &status, WNOHANG);
    if (done == pid_) {
        returncode_ = WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
        const std::scoped_lock lock(children_mutex());
        children().erase(pid_);
    } else if (done < 0 && errno == ECHILD) {
        returncode_ = 0;  // 다른 곳에서 이미 정리됨
    }
    return returncode_;
}

void ChildProcess::terminate() const { ::kill(pid_, SIGTERM); }

void ChildProcess::kill() const { ::kill(pid_, SIGKILL); }

std::optional<int> ChildProcess::wait(double timeout_s) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        if (auto code = poll()) {
            return code;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return poll();
}

void reap_children() {
    const std::scoped_lock lock(children_mutex());
    for (auto it = children().begin(); it != children().end();) {
        int status = 0;
        const pid_t done = ::waitpid(*it, &status, WNOHANG);
        it = done == *it || (done < 0 && errno == ECHILD) ? children().erase(it) : std::next(it);
    }
}

ChildProcess spawn_logged(const std::filesystem::path& cwd, const std::filesystem::path& log,
                          const std::vector<std::string>& arguments) {
    reap_children();
    std::error_code error;
    const std::uintmax_t previous =
        std::filesystem::exists(log, error) ? std::filesystem::file_size(log, error) : 0;
    // 원본 open('ab', buffering=0)
    const int log_fd = open_for_append(log);
    std::vector<std::string> owned = arguments;
    std::vector<char*> argv;
    argv.reserve(owned.size() + 1);
    for (auto& argument : owned) {
        argv.push_back(argument.data());
    }
    argv.push_back(nullptr);
    std::array<int, 2> report{-1, -1};  // 실행 실패를 부모에 알리는 통로 (원본 Popen의 OSError)
    if (::pipe2(report.data(), O_CLOEXEC) != 0) {
        ::close(log_fd);
        throw os_error(errno);
    }
    const pid_t pid = ::fork();
    if (pid == 0) {
        // 게이트웨이는 신호 전용 스레드를 위해 SIGTERM·SIGINT를 막고 SIGPIPE를 무시한다.
        // 막힌 신호와 무시 설정은 exec 뒤에도 남으므로 자식에서 원본 Popen처럼 되돌린다.
        // 그렇지 않으면 자식이 정상 종료 신호를 받지 못한다 (2026-11-06 실기).
        sigset_t none;
        sigemptyset(&none);
        ::sigprocmask(SIG_SETMASK, &none, nullptr);
        ::signal(SIGPIPE, SIG_DFL);
        ::setsid();
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX open
        const int null_fd = ::open("/dev/null", O_RDONLY);
        if (null_fd >= 0) {
            ::dup2(null_fd, STDIN_FILENO);
        }
        ::dup2(log_fd, STDOUT_FILENO);
        ::dup2(log_fd, STDERR_FILENO);
        if (::chdir(cwd.c_str()) == 0) {
            ::execvp(argv.front(), argv.data());
        }
        const int code = errno;
        const ssize_t reported = ::write(report.at(1), &code, sizeof(code));
        static_cast<void>(reported);
        ::_exit(127);
    }
    ::close(log_fd);
    ::close(report.at(1));
    if (pid < 0) {
        ::close(report.at(0));
        throw os_error(errno);
    }
    int child_error = 0;
    const ssize_t got = ::read(report.at(0), &child_error, sizeof(child_error));
    ::close(report.at(0));
    if (got == sizeof(child_error)) {
        ::waitpid(pid, nullptr, 0);
        throw os_error(child_error, arguments.front());
    }
    {
        const std::scoped_lock lock(children_mutex());
        children().insert(pid);
    }
    return {pid, previous};
}

ChildProcess start_process(const std::filesystem::path& root, const std::filesystem::path& run,
                           const std::string& name, const std::vector<std::string>& arguments) {
    ChildProcess child = spawn_logged(root, run / (name + ".log"), arguments);
    std::ofstream pid_file(run / (name + ".pid"), std::ios::binary | std::ios::trunc);
    pid_file << child.pid();
    return child;
}

std::optional<std::vector<std::string>> process_arguments(pid_t pid) {
    try {
        const std::string data =
            read_text(std::filesystem::path("/proc") / std::to_string(pid) / "cmdline");
        std::vector<std::string> out;
        std::string current;
        for (const char c : data) {
            if (c == '\0') {
                out.push_back(current);
                current.clear();
            } else {
                current += c;
            }
        }
        out.push_back(current);  // 원본 split('\0')은 끝에 빈 문자열을 남긴다
        return out;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace argus::gateway
