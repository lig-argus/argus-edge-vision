// 소유 프로세스 확인·실행 (Pi src/argus/runtime/process_supervisor.py와 같은 규칙).
// 장치·FC 동작은 하지 않는다. 실행은 호출하는 쪽이 명시적으로 부를 때만 한다.
#pragma once

#include <sys/types.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace argus::gateway {

// 원본 OSError. str(exc) 형식 "[Errno N] 설명" 또는 "[Errno N] 설명: '파일'"
class OsError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
OsError os_error(int code, const std::string& filename = "");

// 원본 open(path, 'a') (잠금·로그 파일). 실패하면 OsError
int open_for_append(const std::filesystem::path& path);

// 원본 listening(port): 127.0.0.1:port에 0.2초 안에 TCP 연결되면 참
bool tcp_listening(int port);

// 원본 matching_process(executable): /proc/<pid>/exe가 같은 실행 파일인 첫 프로세스
std::optional<pid_t> matching_process(const std::filesystem::path& executable);

// 원본 owns_listeners(pid, ports): 그 프로세스가 LISTEN 중인 TCP 포트에 ports가 모두 있는지
bool owns_listeners(pid_t pid, const std::set<int>& ports);

// 원본 start()가 돌려주는 Popen
class ChildProcess {
public:
    ChildProcess(pid_t pid, std::uintmax_t log_start) : pid_(pid), log_start_(log_start) {}
    pid_t pid() const { return pid_; }
    // 원본 preview_log_start (실행 전 로그 크기)
    std::uintmax_t log_start() const { return log_start_; }
    // 원본 poll(): 끝났으면 종료 코드, 아직이면 nullopt
    std::optional<int> poll();
    // 원본 terminate()/kill()/wait(timeout)
    void terminate() const;
    void kill() const;
    std::optional<int> wait(double timeout_s);

private:
    pid_t pid_;
    std::uintmax_t log_start_;
    std::optional<int> returncode_;
};

// 원본 start(root, run, name, arguments): run/name.log에 출력을 붙이고 새 세션으로 실행한다.
// run/name.pid에 PID를 쓴다. 실행 파일은 PATH에서 찾는다 (원본 Popen).
ChildProcess start_process(const std::filesystem::path& root, const std::filesystem::path& run,
                           const std::string& name, const std::vector<std::string>& arguments);

// 원본 subprocess.Popen(command, cwd, stdout=stderr=log) 공통 부분
ChildProcess spawn_logged(const std::filesystem::path& cwd, const std::filesystem::path& log,
                          const std::vector<std::string>& arguments);

// /proc/<pid>/cmdline 인수 (읽지 못하면 nullopt)
std::optional<std::vector<std::string>> process_arguments(pid_t pid);

// 원본 Path.resolve() (strict=False)
std::filesystem::path resolved(const std::filesystem::path& path);

// 원본 끝난 자식 정리 (좀비가 남지 않게)
void reap_children();

}  // namespace argus::gateway
