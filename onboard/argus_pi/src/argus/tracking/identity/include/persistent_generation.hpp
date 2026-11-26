// generation을 사용하기 전에 디스크에 예약한다. 손상·경합·고갈 시 재사용 없이 종료한다.
#pragma once
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "argus/common/identity/include/float_identity.hpp"

namespace follow {
class PersistentGeneration {
public:
    static std::string default_path() {
        const auto* state = std::getenv("ARGUS_TRACKING_ID_STATE");
        if (state && *state) {
            return state;
        }
        const auto* home = std::getenv("HOME");
        if (!home || !*home) {
            throw TrackingIdentityError(
                "HOME/ARGUS_TRACKING_ID_STATE required for persistent identity");
        }
        return std::string(home) + "/.local/state/argus/tracking-ids/generation";
    }
    explicit PersistentGeneration(std::string path, bool initialize = false)
        : path_(std::move(path)) {
        try {
            if (!std::filesystem::path(path_).is_absolute()) {
                throw TrackingIdentityError("generation state path must be absolute");
            }
            const auto parent = std::filesystem::path(path_).parent_path();
            std::filesystem::create_directories(parent);
            lock_ = ::open((path_ + ".lock").c_str(), O_RDWR | O_CREAT | O_NOFOLLOW, 0600);
            if (lock_ < 0 || ::flock(lock_, LOCK_EX | LOCK_NB) != 0) {
                throw TrackingIdentityError("generation state already owned or lock unavailable");
            }
            if (initialize) {
                // 명시적 최초 생성만 허용한다. 기존 상태를 0으로 덮어쓰지 않는다.
                const int fd =
                    ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
                if (fd < 0) {
                    throw TrackingIdentityError("generation initialization refuses existing state");
                }
                const std::string initial = "ARGUS_FLOAT_IDS_V1\n0\n";
                const bool ok = ::write(fd, initial.data(), initial.size()) ==
                                    static_cast<ssize_t>(initial.size()) &&
                                ::fsync(fd) == 0;
                ::close(fd);
                if (!ok) {
                    throw TrackingIdentityError("generation initialization write failed");
                }
                sync_directory();
            }
            last_ = read_state();
        } catch (const std::exception& error) {
            if (lock_ >= 0) {
                ::close(lock_);
                lock_ = -1;
            }
            throw TrackingIdentityError(error.what());
        }
    }
    ~PersistentGeneration() {
        if (lock_ >= 0) {
            ::close(lock_);
        }
    }
    PersistentGeneration(const PersistentGeneration&) = delete;
    PersistentGeneration& operator=(const PersistentGeneration&) = delete;
    std::uint64_t allocate() {
        if (last_ >= kFloatIdentityMax) {
            throw TrackingIdentityError("generation exhausted; no wrap or automatic reset allowed");
        }
        const auto next = last_ + 1;
        const auto value = std::string("ARGUS_FLOAT_IDS_V1\n") + std::to_string(next) + "\n";
        const auto temporary = path_ + ".tmp." + std::to_string(::getpid());
        const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
        if (fd < 0) {
            throw TrackingIdentityError("generation reservation cannot create file");
        }
        const bool ok =
            ::write(fd, value.data(), value.size()) == static_cast<ssize_t>(value.size()) &&
            ::fsync(fd) == 0;
        ::close(fd);
        if (!ok || ::rename(temporary.c_str(), path_.c_str()) != 0) {
            ::unlink(temporary.c_str());
            throw TrackingIdentityError("generation reservation commit failed");
        }
        sync_directory();
        last_ = next;
        return next;
    }

private:
    std::uint32_t read_state() {
        const int fd = ::open(path_.c_str(), O_RDONLY | O_NOFOLLOW);
        if (fd < 0) {
            throw TrackingIdentityError(
                "generation state missing; explicit first initialization required");
        }
        char bytes[129];
        const auto count = ::read(fd, bytes, sizeof(bytes));
        ::close(fd);
        const std::string prefix = "ARGUS_FLOAT_IDS_V1\n";
        if (count <= static_cast<ssize_t>(prefix.size()) || count >= 128) {
            throw TrackingIdentityError("generation state corrupt");
        }
        const std::string value(bytes, count);
        std::uint32_t output = 0;
        const auto* end = value.data() + value.size() - 1;
        const auto parsed = std::from_chars(value.data() + prefix.size(), end, output);
        if (value.compare(0, prefix.size(), prefix) != 0 || value.back() != '\n' ||
            parsed.ec != std::errc{} || parsed.ptr != end || output > kFloatIdentityMax) {
            throw TrackingIdentityError("generation state corrupt or outside float identity range");
        }
        return output;
    }
    void sync_directory() {
        const auto parent = std::filesystem::path(path_).parent_path().string();
        const int fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
        if (fd < 0) {
            throw TrackingIdentityError("generation directory unavailable");
        }
        const int result = ::fsync(fd);
        ::close(fd);
        if (result != 0) {
            throw TrackingIdentityError("generation directory sync failed");
        }
    }
    std::string path_;
    int lock_{-1};
    std::uint32_t last_{0};
};
}  // namespace follow
