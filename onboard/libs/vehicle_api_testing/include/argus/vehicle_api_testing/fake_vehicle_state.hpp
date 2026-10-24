// 테스트용 IVehicleState. 테스트가 상태, 자세를 넣고 모드 이벤트를 직접 일으킨다.
#ifndef ARGUS_VEHICLE_API_TESTING_FAKE_VEHICLE_STATE_HPP_
#define ARGUS_VEHICLE_API_TESTING_FAKE_VEHICLE_STATE_HPP_

#include <cstdint>
#include <cstdlib>
#include <optional>
#include <utility>
#include <vector>

#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api/vehicle_state.hpp"

namespace argus::vehicle_api_testing {

class FakeVehicleState : public vehicle_api::IVehicleState {
public:
    std::optional<vehicle_api::VehicleState> latest() const override { return latest_; }

    std::optional<vehicle_api::Attitude> attitude_at(std::int64_t monotonic_ns) const override {
        std::optional<vehicle_api::Attitude> nearest;
        std::int64_t nearest_diff = 0;
        for (const auto& attitude : attitudes_) {
            const std::int64_t diff = std::abs(attitude.received_monotonic_ns - monotonic_ns);
            if (diff <= vehicle_api::kAttitudeMatchToleranceNs &&
                (!nearest || diff < nearest_diff)) {
                nearest = attitude;
                nearest_diff = diff;
            }
        }
        return nearest;
    }

    void on_mode_event(ModeEventHandler handler) override {
        handlers_.push_back(std::move(handler));
    }

    // 테스트용 조작
    void set_latest(const std::optional<vehicle_api::VehicleState>& state) { latest_ = state; }
    void add_attitude(const vehicle_api::Attitude& attitude) { attitudes_.push_back(attitude); }
    void emit_mode_event(const vehicle_api::ModeEvent& event) const {
        for (const auto& handler : handlers_) {
            handler(event);
        }
    }

private:
    std::optional<vehicle_api::VehicleState> latest_;
    std::vector<vehicle_api::Attitude> attitudes_;
    std::vector<ModeEventHandler> handlers_;
};

}  // namespace argus::vehicle_api_testing

#endif  // ARGUS_VEHICLE_API_TESTING_FAKE_VEHICLE_STATE_HPP_
