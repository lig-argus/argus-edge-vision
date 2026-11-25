// file descriptor 하나를 소유하고 소멸할 때 닫는다
#ifndef ARGUS_ONBOARD_UNIQUE_FD_HPP_
#define ARGUS_ONBOARD_UNIQUE_FD_HPP_

#include <unistd.h>

#include <cstdint>
#include <utility>

namespace argus::mavlink_gateway {

class UniqueFd {
public:
    // fd 가 음수인 경우 소유한 file descriptor 가 없는 객체
    explicit UniqueFd(std::int32_t fd = -1) : fd_(fd) {}
    ~UniqueFd() {
        if (fd_ >= 0) {
            close(fd_);
        }
    }
    UniqueFd(const UniqueFd&) = delete;
    UniqueFd& operator=(const UniqueFd&) = delete;
    UniqueFd(UniqueFd&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
    UniqueFd& operator=(UniqueFd&&) = delete;

    bool valid() const { return fd_ >= 0; }
    std::int32_t get() const { return fd_; }

private:
    std::int32_t fd_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_UNIQUE_FD_HPP_
