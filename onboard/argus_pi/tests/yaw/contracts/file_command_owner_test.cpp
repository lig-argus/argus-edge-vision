// 실제 커널 flock의 상호 배제·프로세스 종료 시 해제. FC/카메라 연결 없음.
#include "argus/vehicle/mavsdk/include/file_command_owner.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cassert>
#include <csignal>
#include <filesystem>
using namespace follow;
int main() {
    const auto path = std::filesystem::temp_directory_path() /
                      ("thessen-owner-test-" + std::to_string(::getpid()));
    const auto name = path.string();
    FileCommandOwner first{name.c_str()}, second{name.c_str()};
    assert(first.acquire() && first.owns());
    assert(!second.acquire() && !second.owns());
    first.release();
    assert(second.acquire());
    second.release();
    int pipefd[2];
    assert(::pipe(pipefd) == 0);
    const auto child = ::fork();
    assert(child >= 0);
    if (child == 0) {
        ::close(pipefd[0]);
        FileCommandOwner remote{name.c_str()};
        const char acquired = remote.acquire() ? 1 : 0;
        ::write(pipefd[1], &acquired, 1);
        ::pause();
        ::_exit(1);
    }
    ::close(pipefd[1]);
    char acquired = 0;
    assert(::read(pipefd[0], &acquired, 1) == 1 && acquired);
    ::close(pipefd[0]);
    assert(!first.acquire());
    assert(::kill(child, SIGKILL) == 0);
    int status = 0;
    assert(::waitpid(child, &status, 0) == child);
    assert(first.acquire());
    first.release();
    std::filesystem::remove(path);
}
