#pragma once
#include "argus/contracts/Frame.hpp"
namespace argus {
enum class FrameRead { Ready, Retry, EndOfStream };
class IFrameSource {
  public:
    virtual ~IFrameSource() = default;
    virtual FrameRead read(Frame &) = 0;
    virtual void request_stop() noexcept = 0;
    virtual std::string source_name() const = 0;
};
} // namespace argus
