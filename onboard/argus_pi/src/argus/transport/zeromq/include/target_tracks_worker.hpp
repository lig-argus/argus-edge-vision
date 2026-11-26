// tracking/tracks SUB를 명시 선택 코어와 분리된 수신 worker로 연결한다.
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
#include "argus/transport/zeromq/include/zeromq_tracks_bus.hpp"

namespace follow {

struct TargetTracksWorkerStats {
    std::uint64_t messages_received{0};
    std::uint64_t invalid_messages{0};
    std::uint64_t rejected_updates{0};
    std::uint64_t empty_polls{0};
    bool running{false};
    std::string last_error;
};

class TargetTracksWorker final {
public:
    using UpdateCallback = std::function<bool(const TracksV1&, Clock::time_point)>;
    TargetTracksWorker(ZeroMqContext&, TargetSelector&, std::string endpoint,
                       std::chrono::milliseconds poll_interval = std::chrono::milliseconds(25),
                       BusSocketEndpoint endpoint_action = BusSocketEndpoint::Connect,
                       std::string diagnostics_csv = {}, UpdateCallback update = {});
    ~TargetTracksWorker();
    TargetTracksWorker(const TargetTracksWorker&) = delete;
    TargetTracksWorker& operator=(const TargetTracksWorker&) = delete;
    void start();
    void stop();
    TargetTracksWorkerStats stats() const;

private:
    void run(std::promise<void> initialized);
    ZeroMqContext& context_;
    TargetSelector& selector_;
    const std::string endpoint_;
    const std::chrono::milliseconds poll_interval_;
    const BusSocketEndpoint endpoint_action_;
    const std::string diagnostics_csv_;
    const UpdateCallback update_;
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    TargetTracksWorkerStats stats_;
};

}  // namespace follow
