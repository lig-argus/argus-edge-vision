// 테스트용 IFcStateSource. 테스트가 연결 상태와 값을 직접 정한다
#ifndef ARGUS_ONBOARD_TEST_FAKE_FC_STATE_SOURCE_HPP_
#define ARGUS_ONBOARD_TEST_FAKE_FC_STATE_SOURCE_HPP_

#include <cstdint>

#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

class FakeFcStateSource : public IFcStateSource {
public:
    bool fc_connected() const override { return connected; }
    std::uint64_t fc_epoch() const override { return epoch; }
    vehicle_api::VehicleState latest() const override { return state; }

    // 테스트용: received_ns 에 받은 비행 상태(mode, armed)를 넣는다
    void set_status(vehicle_api::FlightMode mode, std::int64_t received_ns, bool armed = true) {
        vehicle_api::FlightStatus status;
        status.mode = mode;
        status.armed = armed;
        status.received_monotonic_ns = received_ns;
        status.fc_epoch = epoch;
        state.status = status;
    }

    bool connected = true;
    std::uint64_t epoch = 1;
    vehicle_api::VehicleState state;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_TEST_FAKE_FC_STATE_SOURCE_HPP_
