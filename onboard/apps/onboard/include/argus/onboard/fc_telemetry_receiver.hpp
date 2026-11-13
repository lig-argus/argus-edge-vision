// FC 의 telemetry 를 받아 vehicle_api 의 값 타입으로 변환한다
// 규약: docs/spec/icd.md 2장 (메시지), docs/spec/process_interfaces.md 5.1 (fc/attitude, fc/state)
// 사용: onboard 의 main 이 subscribe_attitude(), subscribe_state() 로 발행 클래스를 연결
#ifndef ARGUS_ONBOARD_FC_TELEMETRY_RECEIVER_HPP_
#define ARGUS_ONBOARD_FC_TELEMETRY_RECEIVER_HPP_

#include <cstdint>
#include <functional>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/fc_link_tracker.hpp"
#include "argus/onboard/fc_time_converter.hpp"
#include "argus/onboard/handler_list.hpp"
#include "argus/onboard/mavlink_connection.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::onboard {

class FcTelemetryReceiver {
public:
    using AttitudeHandler = std::function<void(const vehicle_api::Attitude&)>;
    using StateHandler = std::function<void(const vehicle_api::VehicleState&)>;

    // 인자로 받은 객체들은 이 객체보다 오래 유지되어야 한다.
    // FC(sysid 1, compid 1)가 송신한 메시지를 받을 때마다 link 에 수신을 알리고, 값마다 link 가
    // 반환한 수신 시각과 link 의 fc_epoch 를 기록한다
    FcTelemetryReceiver(IMavlinkConnection& connection, FcLinkTracker& link,
                        const IFcTimeConverter& time_converter);

    // ATTITUDE 를 받을 때마다 handler 를 호출한다.
    // 반환된 Subscription 이 소멸하면 등록이 해제된다
    [[nodiscard]] core::Subscription subscribe_attitude(AttitudeHandler handler);

    // LOCAL_POSITION_NED, GLOBAL_POSITION_INT, 아래 방향 센서의 DISTANCE_SENSOR, HEARTBEAT 를 받을
    // 때마다 가장 최근 값들을 모은 VehicleState 로 handler 를 호출한다. ATTITUDE 는 이 호출의
    // 대상이 아니다. 반환된 Subscription 이 소멸하면 등록이 해제된다
    [[nodiscard]] core::Subscription subscribe_state(StateHandler handler);

    // 가장 최근 값들. FC 에서 아직 받지 못한 항목은 nullopt. fc_epoch 가 증가한 경우 이전 fc_epoch
    // 의 값은 삭제된다
    const vehicle_api::VehicleState& latest() const { return state_; }

private:
    using MessageHandler = void (FcTelemetryReceiver::*)(const mavlink_message_t&);

    // msgid 의 메시지 중 FC 가 송신한 것만 handler 로 전달하도록 connection 에 등록한다
    void subscribe_fc_message(IMavlinkConnection& connection, std::uint32_t msgid,
                              MessageHandler handler);

    void on_attitude(const mavlink_message_t& message);
    void on_local_position(const mavlink_message_t& message);
    void on_global_position(const mavlink_message_t& message);
    void on_distance_sensor(const mavlink_message_t& message);
    void on_heartbeat(const mavlink_message_t& message);

    FcLinkTracker& link_;
    const IFcTimeConverter& time_converter_;

    vehicle_api::VehicleState state_;
    HandlerList<const vehicle_api::Attitude&> attitude_handlers_;
    HandlerList<const vehicle_api::VehicleState&> state_handlers_;

    std::vector<core::Subscription> message_subscriptions_;
    core::Subscription epoch_subscription_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_FC_TELEMETRY_RECEIVER_HPP_
