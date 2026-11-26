// PUB/SUB 대기와 추적·제어 소비를 분리하는 구독 worker다.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include "argus/perception/observations/include/observation_inbox.hpp"
#include "argus/transport/zeromq/include/zeromq_observation_bus.hpp"

namespace follow {

struct ObservationBusWorkerStats {
    std::uint64_t messages_received{0};
    std::uint64_t empty_polls{0};
    std::uint64_t malformed_messages{0};
    std::uint64_t transport_errors{0};
    bool running{false};
    std::string last_error;
};

class ObservationBusWorker final {
public:
    ObservationBusWorker(ZeroMqContext& context, ObservationInbox& inbox, std::string endpoint,
                         std::chrono::milliseconds poll_interval = std::chrono::milliseconds(50));
    ~ObservationBusWorker();
    ObservationBusWorker(const ObservationBusWorker&) = delete;
    ObservationBusWorker& operator=(const ObservationBusWorker&) = delete;

    void start();
    void stop();
    ObservationBusWorkerStats stats() const;

private:
    void run(std::promise<void> initialized);

    ZeroMqContext& context_;
    ObservationInbox& inbox_;
    const std::string endpoint_;
    const std::chrono::milliseconds poll_interval_;
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    ObservationBusWorkerStats stats_;
};

}  // namespace follow
