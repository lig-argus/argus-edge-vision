#pragma once
#include <cstdint>
#include <optional>
#include <string>
namespace argus {
struct CommandRequest {
    std::string request_id, command;
    std::uint64_t issued_unix_ns{}, expires_monotonic_ns{};
    std::optional<std::int64_t> target_id;
    std::string schema{"argus.command.request.v1"};
};
struct CommandResult {
    std::string request_id;
    bool accepted{};
    std::string code, detail;
    std::uint64_t completed_unix_ns{};
    std::string schema{"argus.command.result.v1"};
}; // Data contracts only. CommandGateway/FSM execution is not part of this migration.
} // namespace argus
