// 테스트용 IVehicleTelemetry. 테스트가 상태, 자세, 위치, 거리와 조회 시점을 넣고 비행 모드
// 이벤트를 직접 보낸다.
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_VEHICLE_TELEMETRY_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_VEHICLE_TELEMETRY_HPP_

#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api/vehicle_telemetry.hpp"

namespace argus::vehicle_api_testing {

class FakeVehicleTelemetry : public vehicle_api::IVehicleTelemetry {
public:
    FakeVehicleTelemetry() : handlers_(std::make_shared<Handlers>()) {}

    std::optional<vehicle_api::VehicleState> latest() const override { return latest_; }

    std::optional<vehicle_api::Attitude> attitude_at(std::int64_t monotonic_ns) const override {
        return pick_nearest_fresh(attitudes_, monotonic_ns);
    }

    std::optional<vehicle_api::LocalPosition> local_position_at(
        std::int64_t monotonic_ns) const override {
        return pick_nearest_fresh(local_positions_, monotonic_ns);
    }

    std::optional<vehicle_api::DownwardDistance> downward_distance_at(
        std::int64_t monotonic_ns) const override {
        return pick_nearest_fresh(downward_distances_, monotonic_ns);
    }

    [[nodiscard]] core::Subscription subscribe_flight_mode_events(
        FlightModeEventHandler handler) override {
        const std::uint64_t id = handlers_->next_id++;
        handlers_->by_id[id] = std::move(handler);
        const std::weak_ptr<Handlers> weak = handlers_;
        return core::Subscription([weak, id] {
            if (const auto handlers = weak.lock()) {
                handlers->by_id.erase(id);
            }
        });
    }

    // 테스트용 조작
    void set_latest(const std::optional<vehicle_api::VehicleState>& state) { latest_ = state; }
    // 구현의 시계 역할. *_at() 이 kTelemetryMaxAgeNs 를 재는 기준 시점
    void set_now_monotonic_ns(std::int64_t now_monotonic_ns) {
        now_monotonic_ns_ = now_monotonic_ns;
    }
    void add_attitude(const vehicle_api::Attitude& attitude) { attitudes_.push_back(attitude); }
    void add_local_position(const vehicle_api::LocalPosition& position) {
        local_positions_.push_back(position);
    }
    void add_downward_distance(const vehicle_api::DownwardDistance& distance) {
        downward_distances_.push_back(distance);
    }

    // 실제 구현은 process_events() 안에서 handler 를 부른다. 이 Fake 는 테스트가 부른 자리에서
    // 바로 부른다. handler 가 등록을 풀거나 더해도 안전하도록 복사본을 돈다
    void simulate_flight_mode_event(const vehicle_api::FlightModeEvent& event) const {
        const auto snapshot = handlers_->by_id;
        for (const auto& [id, handler] : snapshot) {
            if (handlers_->by_id.count(id) != 0) {
                handler(event);
            }
        }
    }

private:
    struct Handlers {
        std::map<std::uint64_t, FlightModeEventHandler> by_id;
        std::uint64_t next_id = 1;
    };

    // 조회 시점 기준 kTelemetryMaxAgeNs 이내에 받은 표본인지. 경과 시간은 방향이 있는 값이라
    // 부호 있는 차를 쓴다. 조회 시점보다 뒤에 받은 표본은 오래된 것이 아니므로 통과한다
    template <typename T>
    bool is_fresh(const T& sample) const {
        const std::int64_t age = now_monotonic_ns_ - sample.received_monotonic_ns;
        return age <= vehicle_api::kTelemetryMaxAgeNs;
    }

    // IVehicleTelemetry 의 공통 규칙: is_fresh 인 표본 중 monotonic_ns 에 가장 가까운 것, 그 차가
    // kTelemetryMatchToleranceNs 이내일 때만. 거리는 프레임 앞뒤 어느 쪽이든 같으므로 절댓값
    template <typename T>
    std::optional<T> pick_nearest_fresh(const std::vector<T>& samples,
                                        std::int64_t monotonic_ns) const {
        const T* nearest = nullptr;
        std::int64_t nearest_diff = 0;
        for (const T& sample : samples) {
            const std::int64_t diff = std::abs(sample.received_monotonic_ns - monotonic_ns);
            const bool closer = nearest == nullptr || diff < nearest_diff;
            if (is_fresh(sample) && closer) {
                nearest = &sample;
                nearest_diff = diff;
            }
        }
        if (nearest == nullptr || nearest_diff > vehicle_api::kTelemetryMatchToleranceNs) {
            return std::nullopt;
        }
        return *nearest;
    }

    std::optional<vehicle_api::VehicleState> latest_;
    std::int64_t now_monotonic_ns_ = 0;
    std::vector<vehicle_api::Attitude> attitudes_;
    std::vector<vehicle_api::LocalPosition> local_positions_;
    std::vector<vehicle_api::DownwardDistance> downward_distances_;
    std::shared_ptr<Handlers> handlers_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_VEHICLE_TELEMETRY_HPP_
