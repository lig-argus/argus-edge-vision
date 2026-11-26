#include "argus/gateway/ownership/include/command_owner.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdlib>
#include <system_error>
#include <utility>

namespace argus::gateway {

namespace {

[[noreturn]] void throw_errno(const char* what) {
    throw std::system_error(errno, std::generic_category(), what);
}

}  // namespace

CommandOwner::CommandOwner(std::string path) : path_(std::move(path)) {
    if (!path_.empty()) {
        // POSIX open()은 mode 인수 때문에 가변 인수 함수다.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        fd_ = ::open(path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd_ < 0) {
            throw_errno("송신자 잠금 파일 열기 실패");
        }
    }
}

CommandOwner CommandOwner::from_environment() {
    const char* value = std::getenv("ARGUS_COMMAND_OWNER_FILE");
    return CommandOwner(value != nullptr ? value : "");
}

CommandOwner::CommandOwner(CommandOwner&& other) noexcept
    : path_(std::move(other.path_)),
      fd_(std::exchange(other.fd_, -1)),
      held_(std::exchange(other.held_, false)) {}

CommandOwner::~CommandOwner() {
    try {
        close();
    } catch (...) {  // NOLINT(bugprone-empty-catch)
        // 소멸 중에는 잠금 해제 실패를 던지지 않는다 (fd를 닫으면 커널이 잠금을 푼다)
    }
}

bool CommandOwner::acquire() {
    if (path_.empty() || held_) {
        return true;
    }
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
        if (errno == EWOULDBLOCK) {
            return false;
        }
        throw_errno("송신자 잠금 실패");
    }
    const std::string label = R"({"role": "fc", "pid": )" + std::to_string(::getpid()) + "}";
    if (::ftruncate(fd_, 0) != 0 ||
        ::pwrite(fd_, label.data(), label.size(), 0) != static_cast<ssize_t>(label.size())) {
        const int saved = errno;
        ::flock(fd_, LOCK_UN);
        errno = saved;
        throw_errno("송신자 잠금 기록 실패");
    }
    held_ = true;
    return true;
}

void CommandOwner::release() {
    if (held_) {
        ::flock(fd_, LOCK_UN);
    }
    held_ = false;
}

nlohmann::json CommandOwner::status() const {
    if (path_.empty()) {
        return {{"enabled", false}, {"role", nullptr}, {"available", true}};
    }
    if (held_) {
        return {{"enabled", true}, {"role", "fc"}, {"available", false}};
    }
    // 다른 fd로 확인한다: 자기 fd로 잠갔다 풀면 그 fd의 소유가 바뀐다.
    // POSIX open()은 mode 인수 때문에 가변 인수 함수다.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    const int fd = ::open(path_.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {
        throw_errno("송신자 잠금 파일 열기 실패");
    }
    nlohmann::json result;
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        if (errno != EWOULDBLOCK) {
            const int saved = errno;
            ::close(fd);
            errno = saved;
            throw_errno("송신자 잠금 확인 실패");
        }
        std::array<char, 512> buffer{};
        const ssize_t count = ::pread(fd, buffer.data(), buffer.size(), 0);
        nlohmann::json label = nlohmann::json::object();
        if (count > 0) {
            label = nlohmann::json::parse(
                std::string(buffer.data(), static_cast<std::size_t>(count)), nullptr, false);
            if (label.is_discarded()) {
                label = nlohmann::json::object();
            }
        }
        nlohmann::json role = "unknown";
        if (label.is_object() && label.contains("role")) {
            role = label.at("role");
        }
        result = {{"enabled", true}, {"role", role}, {"available", false}};
    } else {
        ::flock(fd, LOCK_UN);
        result = {{"enabled", true}, {"role", nullptr}, {"available", true}};
    }
    ::close(fd);
    return result;
}

void CommandOwner::close() {
    release();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

}  // namespace argus::gateway
