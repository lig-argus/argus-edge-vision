// Same command boundary for in-process handlers and transport adapters.
#pragma once

#include <functional>
#include <utility>

#include "argus/common/serialization/include/native_json_io.hpp"

namespace follow {
class IFollowCommand {
public:
    virtual ~IFollowCommand() = default;
    virtual NativeJson execute(const NativeJson&) = 0;
};

using IFollowControl = IFollowCommand;

class FunctionalFollowCommand final : public IFollowCommand {
public:
    explicit FunctionalFollowCommand(std::function<NativeJson(const NativeJson&)> handler)
        : handler_(std::move(handler)) {}
    NativeJson execute(const NativeJson& request) override { return handler_(request); }

private:
    std::function<NativeJson(const NativeJson&)> handler_;
};
}  // namespace follow
