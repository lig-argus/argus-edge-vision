#pragma once
#include "argus/adapters/OwnedProcess.hpp"
#include "argus/ports/IFrameSource.hpp"
#include <atomic>
#include <memory>
namespace argus {
class OpenCvFrameSource final : public IFrameSource {
    OwnedProcess process_;
    const IClock &clock_;
    std::atomic<bool> stopping_{};
    std::uint64_t sequence_{};
    bool replay_;
    std::string name_;
    bool read_exact(std::uint8_t *, std::size_t);

  public:
    OpenCvFrameSource(const IClock &, const std::string &path, bool replay, unsigned width, unsigned height,
                      unsigned fps, const std::string &fourcc);
    ~OpenCvFrameSource();
    FrameRead read(Frame &) override;
    void request_stop() noexcept override {
        stopping_ = true;
    }
    std::string source_name() const override {
        return name_;
    }
};
} // namespace argus
