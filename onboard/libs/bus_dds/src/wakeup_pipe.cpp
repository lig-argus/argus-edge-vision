#include "wakeup_pipe.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <array>
#include <cstdint>

namespace argus::bus_dds::detail {
namespace {

bool set_nonblocking(int fd) {
    const auto flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    const auto nonblocking =
        static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(O_NONBLOCK);
    return fcntl(fd, F_SETFL, static_cast<std::int32_t>(nonblocking)) == 0;
}

}  // namespace

WakeupPipe::WakeupPipe() {
    std::array<int, 2> fds{-1, -1};
    if (pipe(fds.data()) != 0) {
        return;
    }
    if (!set_nonblocking(fds[0]) || !set_nonblocking(fds[1])) {
        close(fds[0]);
        close(fds[1]);
        return;
    }
    read_fd_ = fds[0];
    write_fd_ = fds[1];
}

WakeupPipe::~WakeupPipe() {
    if (read_fd_ >= 0) {
        close(read_fd_);
        close(write_fd_);
    }
}

void WakeupPipe::notify() const {
    const std::uint8_t byte = 1;
    // 실패(EAGAIN: pipe 가 가득 참)는 이미 깨울 바이트가 있다는 뜻이므로 무시한다
    static_cast<void>(write(write_fd_, &byte, 1));
}

void WakeupPipe::drain() const {
    std::array<std::uint8_t, 64> buffer{};
    while (read(read_fd_, buffer.data(), buffer.size()) > 0) {
    }
}

}  // namespace argus::bus_dds::detail
