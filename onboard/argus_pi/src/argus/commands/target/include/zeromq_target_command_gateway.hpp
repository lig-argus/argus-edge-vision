// command/request REP와 command/result 응답을 운용자 도구·TargetSelector 사이에 둔다.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include "argus/control/target/include/target_selector.hpp"
#include "argus/transport/zeromq/include/zeromq_observation_bus.hpp"

namespace follow {

struct TargetCommandGatewayStats {
    std::uint64_t requests{0};
    std::uint64_t rejected_malformed{0};
    std::uint64_t transport_errors{0};
    bool running{false};
    std::string last_error;
};

class ZeroMqTargetCommandGateway final {
public:
    ZeroMqTargetCommandGateway(
        ZeroMqContext&, TargetSelector&, std::string endpoint,
        BusSocketEndpoint action = BusSocketEndpoint::Bind,
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds(25),
        std::string expected_uid = {}, std::function<bool()> identity_verified = {},
        std::function<TargetCommandResult(const TargetCommand&)> executor = {});
    ~ZeroMqTargetCommandGateway();
    ZeroMqTargetCommandGateway(const ZeroMqTargetCommandGateway&) = delete;
    ZeroMqTargetCommandGateway& operator=(const ZeroMqTargetCommandGateway&) = delete;
    void start();
    void stop();
    TargetCommandGatewayStats stats() const;

private:
    void run(std::promise<void> initialized);
    TargetCommandResult process_request(void* socket);
    ZeroMqContext& context_;
    TargetSelector& selector_;
    const std::string endpoint_;
    const BusSocketEndpoint action_;
    const std::chrono::milliseconds poll_interval_;
    const std::string expected_uid_;
    const std::function<bool()> identity_verified_;
    const std::function<TargetCommandResult(const TargetCommand&)> executor_;
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    TargetCommandGatewayStats stats_;
};

class ZeroMqTargetCommandClient final {
public:
    explicit ZeroMqTargetCommandClient(ZeroMqContext& context) : context_(context) {}
    bool request(const std::string& endpoint, BusSocketEndpoint action,
                 const TargetCommand& command, TargetCommandResult& result, std::string& error,
                 std::chrono::milliseconds timeout = std::chrono::milliseconds(500));

private:
    ZeroMqContext& context_;
};

}  // namespace follow
