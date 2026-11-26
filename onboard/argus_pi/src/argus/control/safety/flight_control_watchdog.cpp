// 표적·기체 상태를 제어 주기와 독립해 확인하되 포트 명령은 생성하지 않는다.
#include "argus/control/safety/include/flight_control_watchdog.hpp"

#include <algorithm>
#include <stdexcept>

namespace follow {
void FlightSafetyEventLatch::publish(FlightSafetyEvent event, std::optional<std::uint64_t> epoch) {
    if (event == FlightSafetyEvent::None) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch && epoch_ && *epoch < *epoch_) {
        return;
    }
    if (epoch && epoch_ && *epoch > *epoch_) {
        event_ = FlightSafetyEvent::None;
    }
    if (event > event_ || (event == event_ && !epoch)) {
        event_ = event;
        epoch_ = epoch;
    }
}

FlightSafetyEvent FlightSafetyEventLatch::consume(std::optional<std::uint64_t> epoch) {
    std::lock_guard<std::mutex> lock(mutex_);
    // An inspection can finish after a newer Start/Stop; it must keep its original command epoch.
    if (epoch && epoch_ && *epoch_ > *epoch) {
        return FlightSafetyEvent::None;
    }
    const auto result = epoch && epoch_ && *epoch_ != *epoch ? FlightSafetyEvent::None : event_;
    event_ = FlightSafetyEvent::None;
    epoch_.reset();
    return result;
}

FlightControlWatchdog::FlightControlWatchdog(TargetSelector& selector,
                                             const IVehicleFlightStatusSource& flight_state_source,
                                             const IVehicleStateSource& pose_source,
                                             FlightSafetyEventLatch& latch,
                                             const std::atomic<bool>& following_active,
                                             std::chrono::milliseconds period,
                                             std::chrono::milliseconds max_pose_age)
    : selector_(selector),
      flight_state_source_(flight_state_source),
      pose_source_(pose_source),
      latch_(latch),
      following_active_(following_active),
      period_(period),
      max_pose_age_(max_pose_age) {
    if (period_.count() < 20 || period_ > std::chrono::milliseconds(50) ||
        max_pose_age_.count() <= 0) {
        throw std::invalid_argument("invalid watchdog period or pose age");
    }
}

FlightControlWatchdog::~FlightControlWatchdog() { stop(); }

void FlightControlWatchdog::start() {
    if (thread_.joinable()) {
        throw std::logic_error("flight watchdog already started");
    }
    stop_requested_.store(false);
    thread_ = std::thread([this] { run(); });
}

void FlightControlWatchdog::stop() {
    stop_requested_.store(true);
    if (thread_.joinable()) {
        thread_.join();
    }
}

FlightSafetyEvent FlightControlWatchdog::inspect_once(Clock::time_point now) const {
    return inspect(now).first;
}

std::pair<FlightSafetyEvent, std::optional<std::uint64_t>> FlightControlWatchdog::inspect(
    Clock::time_point now) const {
    // Offboard 준비/확인 중 Hold는 정상이다. 추종이 commit된 이후에만 모드 이탈을 감시한다.
    if (!following_active_.load(std::memory_order_acquire)) {
        return {FlightSafetyEvent::None, std::nullopt};
    }
    const auto target = selector_.snapshot(now);
    const auto event = [&](FlightSafetyEvent value) {
        return std::make_pair(value, std::optional<std::uint64_t>{target.command_epoch});
    };
    if (target.phase != TargetSelectionSnapshot::Phase::Following) {
        return event(FlightSafetyEvent::None);
    }

    const auto flight = flight_state_source_.read();
    if (flight.rc_override) {
        return event(FlightSafetyEvent::ManualOverride);
    }
    if (!flight.connected || !flight.armed || !flight.in_air) {
        return event(FlightSafetyEvent::FlightStateInvalid);
    }
    if (!flight.offboard) {
        return event(FlightSafetyEvent::OffboardLost);
    }
    const auto pose = pose_source_.read();
    const auto checked_now = std::max({now, target.snapshot_at, pose.snapshot_at});
    const bool missing_pose = !pose.have_position || !pose.have_attitude;
    const bool future_pose = checked_now < pose.position_at || checked_now < pose.attitude_at;
    const bool stale_pose = checked_now - pose.position_at > max_pose_age_ ||
                            checked_now - pose.attitude_at > max_pose_age_;
    if (missing_pose || future_pose || stale_pose) {
        return event(FlightSafetyEvent::FlightStateInvalid);
    }
    if (target.recovery_pause) {
        return event(FlightSafetyEvent::None);
    }
    if (!target.selected_track || !target.latest_frame ||
        (target.state != TargetState::Confirmed && target.state != TargetState::Predicted)) {
        return event(FlightSafetyEvent::TargetUnavailable);
    }
    return event(FlightSafetyEvent::None);
}

void FlightControlWatchdog::run() {
    auto next = Clock::now();
    while (!stop_requested_.load()) {
        next += period_;
        std::pair<FlightSafetyEvent, std::optional<std::uint64_t>> event{FlightSafetyEvent::None,
                                                                         std::nullopt};
        try {
            event = inspect(Clock::now());
        } catch (...) {
            // 공급자 예외는 감시 실패로 처리해 제어 측에 안전 사건을 남긴다.
            event = {FlightSafetyEvent::FlightStateInvalid, std::nullopt};
        }
        if (event.first != FlightSafetyEvent::None) {
            latch_.publish(event.first, event.second);
        }
        std::this_thread::sleep_until(next);
        if (next < Clock::now()) {
            next = Clock::now();
        }
    }
}
}  // namespace follow
