#pragma once
#include <string>
#include <sys/types.h>
#include <vector>
namespace argus {
class OwnedProcess {
    pid_t pid_{-1};
    int input_{-1}, output_{-1};
    int exit_status_{};

  public:
    OwnedProcess() = default;
    ~OwnedProcess();
    OwnedProcess(const OwnedProcess &) = delete;
    OwnedProcess &operator=(const OwnedProcess &) = delete;
    void start(const std::vector<std::string> &arguments, bool capture_stdout = false);
    bool running();
    int output_fd() const {
        return output_;
    }
    void close_input() noexcept;
    void stop() noexcept;
};
} // namespace argus
