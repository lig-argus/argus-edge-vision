#pragma once
#include "argus/adapters/OwnedProcess.hpp"
#include "argus/ports/IFrameSource.hpp"
#include <atomic>
namespace argus {
class I3SdkFrameSource final : public IFrameSource {
    OwnedProcess process_;
    const IClock &clock_;
    std::atomic<bool> stopping_{};
    std::uint64_t last_sequence_{};
    unsigned timeout_s_;
    std::string name_;
    bool read_exact(std::uint8_t *, std::size_t, bool header);

  public:
    I3SdkFrameSource(const IClock &, const std::string &executable, unsigned device, unsigned timeout_s);
    FrameRead read(Frame &) override;
    void request_stop() noexcept override {
        stopping_ = true;
    }
    std::string source_name() const override {
        return name_;
    }
};
} // namespace argus
