// mavlink_gateway(compid 195)의 HEARTBEAT 를 통제기와 FC 에 송신
// 규약: docs/spec/icd.md 2장 (HEARTBEAT), 6.7 (통제기의 mavlink_gateway 두절 판단)
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 tick() 을 kHeartbeatPeriod 주기 작업으로
//      등록
#ifndef ARGUS_ONBOARD_HEARTBEAT_SENDER_HPP_
#define ARGUS_ONBOARD_HEARTBEAT_SENDER_HPP_

#include <chrono>

#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/mavlink_connection.hpp"

namespace argus::mavlink_gateway {

// HEARTBEAT 송신 주기 (MAVLink 의 컴포넌트 HEARTBEAT 1 Hz)
inline constexpr std::chrono::milliseconds kHeartbeatPeriod{1000};

// tick() 마다 HEARTBEAT 하나를 모든 통제기에 broadcast 하고 FC 로 보낸다 (mavlink-router 경유).
// type 은 MAV_TYPE_ONBOARD_CONTROLLER, system_status 는 ARGUS_HEALTH_FC_CONNECTED 가 1 이면
// MAV_STATE_ACTIVE, 0 이면 MAV_STATE_CRITICAL. compid 191 은 HEARTBEAT 를 보내지 않는다
class HeartbeatSender {
public:
    // link, connection, health 는 이 객체보다 오래 유지되어야 한다
    HeartbeatSender(IGcsLink& link, IMavlinkConnection& connection, const HealthState& health)
        : link_(link), connection_(connection), health_(health) {}

    // kHeartbeatPeriod 마다 호출한다
    void tick();

private:
    IGcsLink& link_;
    IMavlinkConnection& connection_;
    const HealthState& health_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_HEARTBEAT_SENDER_HPP_
