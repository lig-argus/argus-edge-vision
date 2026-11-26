#include "argus/runtime/include/owned_process.hpp"

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
extern char** environ;
namespace argus {
OwnedProcess::~OwnedProcess() { stop(); }
void OwnedProcess::start(const std::vector<std::string>& arguments, bool capture_stdout) {
    if (arguments.empty() || pid_ > 0) {
        throw std::invalid_argument("invalid owned process start");
    }
    int input[2]{-1, -1}, output[2]{-1, -1};
    if (pipe2(input, O_CLOEXEC) || (capture_stdout && pipe2(output, O_CLOEXEC))) {
        for (int fd : input) {
            if (fd >= 0) {
                close(fd);
            }
        }
        throw std::runtime_error("process pipe failed");
    }
    std::vector<char*> argv;
    for (const auto& arg : arguments) {
        argv.push_back(const_cast<char*>(arg.c_str()));
    }
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, input[0], STDIN_FILENO);
    posix_spawn_file_actions_addclose(&actions, input[0]);
    posix_spawn_file_actions_addclose(&actions, input[1]);
    if (capture_stdout) {
        posix_spawn_file_actions_adddup2(&actions, output[1], STDOUT_FILENO);
        posix_spawn_file_actions_addclose(&actions, output[0]);
        posix_spawn_file_actions_addclose(&actions, output[1]);
    }
    auto rc = posix_spawnp(&pid_, argv[0], &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(input[0]);
    if (capture_stdout) {
        close(output[1]);
    }
    if (rc) {
        close(input[1]);
        if (capture_stdout) {
            close(output[0]);
        }
        pid_ = -1;
        throw std::runtime_error("spawn failed: " + std::string(std::strerror(rc)));
    }
    input_ = input[1];
    if (capture_stdout) {
        output_ = output[0];
    }
}
bool OwnedProcess::running() {
    if (pid_ <= 0) {
        return false;
    }
    auto rc = waitpid(pid_, &exit_status_, WNOHANG);
    if (rc == pid_ || (rc < 0 && errno == ECHILD)) {
        pid_ = -1;
        return false;
    }
    return true;
}
void OwnedProcess::close_input() noexcept {
    if (input_ >= 0) {
        close(input_);
        input_ = -1;
    }
}
void OwnedProcess::stop() noexcept {
    close_input();
    // SDK captures may still be writing a complete frame to stdout when their
    // reader stops. Drain that tail so the child can observe stdin EOF and run
    // CloseTE instead of being killed while it still owns the USB stream.
    bool drain_output = false;
    if (output_ >= 0) {
        auto flags = fcntl(output_, F_GETFL);
        if (flags >= 0 && fcntl(output_, F_SETFL, flags | O_NONBLOCK) >= 0) {
            drain_output = true;
        }
    }
    auto wait_for = [&](int ms) {
        for (int elapsed = 0; elapsed < ms; elapsed += 20) {
            if (drain_output) {
                char discard[16384];
                for (unsigned chunk = 0; chunk < 64; ++chunk) {
                    if (::read(output_, discard, sizeof(discard)) <= 0) {
                        break;
                    }
                }
            }
            if (!running()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return !running();
    };
    if (pid_ > 0 && !wait_for(4000)) {
        kill(pid_, SIGTERM);
        if (!wait_for(1000)) {
            kill(pid_, SIGKILL);
            while (waitpid(pid_, &exit_status_, 0) < 0 && errno == EINTR) {
            }
            pid_ = -1;
        }
    }
    if (output_ >= 0) {
        close(output_);
        output_ = -1;
    }
}
}  // namespace argus
