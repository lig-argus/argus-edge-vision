// 독립 감시 thread는 안전 사건만 latch하고 PX4 포트는 제어 thread에 맡긴다.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

#include "argus/control/target/include/target_selector.hpp"
#include "argus/vehicle/contracts/include/i_vehicle_state_source.hpp"
#include "argus/vehicle/contracts/include/vehicle_flight_status.hpp"

namespace follow {

enum class FlightSafetyEvent : std::uint8_t {
    None = 0,
    TargetUnavailable = 1,
    OffboardLost = 2,
    FlightStateInvalid = 3,
    ManualOverride = 4
};

class FlightSafetyEventLatch final {
public:
    void publish(FlightSafetyEvent event,
                 std::optional<std::uint64_t> command_epoch = std::nullopt);
    FlightSafetyEvent consume(std::optional<std::uint64_t> command_epoch = std::nullopt);

private:
    std::mutex mutex_;
    FlightSafetyEvent event_{FlightSafetyEvent::None};
    std::optional<std::uint64_t> epoch_;
};

class FlightControlWatchdog final {
public:
    FlightControlWatchdog(TargetSelector&, const IVehicleFlightStatusSource&,
                          const IVehicleStateSource&, FlightSafetyEventLatch&,
                          const std::atomic<bool>& following_active,
                          std::chrono::milliseconds period, std::chrono::milliseconds max_pose_age);
    ~FlightControlWatchdog();
    FlightControlWatchdog(const FlightControlWatchdog&) = delete;
    FlightControlWatchdog& operator=(const FlightControlWatchdog&) = delete;
    void start();
    void stop();
    FlightSafetyEvent inspect_once(Clock::time_point now = Clock::now()) const;

private:
    std::pair<FlightSafetyEvent, std::optional<std::uint64_t>> inspect(Clock::time_point now) const;
    void run();
    TargetSelector& selector_;
    const IVehicleFlightStatusSource& flight_state_source_;
    const IVehicleStateSource& pose_source_;
    FlightSafetyEventLatch& latch_;
    const std::atomic<bool>& following_active_;
    const std::chrono::milliseconds period_;
    const std::chrono::milliseconds max_pose_age_;
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;
};

}  // namespace follow
