// mavlink_gateway 의 FC 쪽 객체를 생성하고 서로 연결한 뒤 bus 와 연결한다
// 규약: docs/design/uml/class_mavlink_gateway.md 2절, docs/spec/process_interfaces.md 5.1, 5.2
// 사용: mavlink_gateway 의 main 이 통신 구현과 시계를 생성한 뒤 하나를 생성
//
// - FC telemetry 를 fc/attitude, fc/state 로, FC 의 모드 변경을 fc/mode_event 로 발행한다
// - 추종 제어의 control/setpoint 와 control/mode_request 를 FcControl 로 전달한다
// - FcControl::tick, FcLinkTracker::check_timeout, TimeSync::send_request 를 주기 작업으로 등록한다
// 통신 구현(ZeroMQ, UDP)과 실제 시계를 알지 못하고 인터페이스로만 사용한다
#ifndef ARGUS_ONBOARD_FC_GATEWAY_HPP_
#define ARGUS_ONBOARD_FC_GATEWAY_HPP_

#include <chrono>
#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/event_loop.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink_gateway/fc_command_lock_release.hpp"
#include "argus/mavlink_gateway/fc_control.hpp"
#include "argus/mavlink_gateway/fc_link_tracker.hpp"
#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/mavlink_gateway/fc_state_view.hpp"
#include "argus/mavlink_gateway/fc_telemetry_receiver.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/mavlink_connection.hpp"
#include "argus/mavlink_gateway/stop_requester.hpp"
#include "argus/mavlink_gateway/time_sync.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/bus_flight_mode_control_server.hpp"
#include "argus/vehicle_api_bus/bus_vehicle_telemetry_publisher.hpp"
#include "argus/vehicle_api_bus/bus_velocity_setpoint_subscriber.hpp"

namespace argus::mavlink_gateway {

// FcControl::tick 의 호출 주기 (20 Hz, FcControl 의 규약)
inline constexpr std::chrono::milliseconds kFcControlTickPeriod{50};

// FcLinkTracker::check_timeout 의 호출 주기. 두절 기준(kFcLinkTimeoutNs, 0.5 s)의 1/10 이다.
// FcControl 은 연결 상태를 호출 시각으로 계산하므로, 이 주기는 HealthState 의 FC 연결 비트가 바뀌는
// 지연에만 영향을 준다
inline constexpr std::chrono::milliseconds kFcLinkCheckPeriod{50};

// TimeSync::send_request 의 호출 주기 (10 Hz). docs/spec/icd.md 에 주기가 없다. 재연결 뒤 변환값이
// 없는 시간을 약 0.1 s 로 줄이고, 왕복 시간이 짧은 응답을 받을 기회를 늘린다
inline constexpr std::chrono::milliseconds kTimeSyncPeriod{100};

class FcGateway {
public:
    // 인자로 받은 객체들은 반환된 객체보다 오래 유지되어야 한다.
    // bus 가 발행자, 구독자, 응답하는 쪽 중 하나라도 만들지 못하면 nullptr 를 반환한다 (이유는 bus
    // 구현이 로그에 남긴다)
    static std::unique_ptr<FcGateway> create(bus::IBus& bus, IMavlinkConnection& connection,
                                             const core::IClock& clock, core::EventLoop& loop,
                                             HealthState& health);

    FcGateway(const FcGateway&) = delete;
    FcGateway& operator=(const FcGateway&) = delete;
    FcGateway(FcGateway&&) = delete;
    FcGateway& operator=(FcGateway&&) = delete;
    ~FcGateway() = default;

    // 통제기 명령 처리가 사용한다 (다음 작업). 반환된 참조는 이 객체가 유지되는 동안 유효하다
    IStopRequester& stop_requester() { return control_; }
    IFcCommandLockRelease& lock_release() { return control_; }
    const IFcStateSource& fc_state() const { return state_view_; }

private:
    FcGateway(IMavlinkConnection& connection, const core::IClock& clock, HealthState& health);

    // bus 의 발행자, 구독자, 응답하는 쪽을 만든다. 하나라도 실패하면 false
    bool create_bus_adapters(bus::IBus& bus);
    // FC 쪽 객체의 출력을 bus 로, bus 의 입력을 FcControl 로 연결한다
    void connect_to_bus();
    void start_periodic_tasks(core::EventLoop& loop);

    // fc/state 를 발행한다. FcTelemetryReceiver 는 offboard_hover 를 알지 못하므로(HEARTBEAT 에
    // 없는 값) FcControl 의 값으로 채운다 (process_interfaces.md 5.1)
    void publish_state(const vehicle_api::VehicleState& state);

    // 멤버는 선언의 역순으로 소멸한다. 주기 작업, 구독, bus 어댑터를 그것이 참조하는 객체보다 뒤에
    // 선언하여 먼저 소멸시킨다
    FcLinkTracker link_;
    TimeSync time_sync_;
    FcTelemetryReceiver telemetry_;
    FcStateView state_view_;
    FcControl control_;

    std::unique_ptr<vehicle_api_bus::BusVehicleTelemetryPublisher> telemetry_publisher_;
    std::unique_ptr<vehicle_api_bus::BusVelocitySetpointSubscriber> setpoint_subscriber_;
    std::unique_ptr<vehicle_api_bus::BusFlightModeControlServer> mode_control_server_;

    core::Subscription attitude_subscription_;
    core::Subscription state_subscription_;
    core::Subscription mode_event_subscription_;
    core::Subscription setpoint_subscription_;

    core::PeriodicTimer control_tick_timer_;
    core::PeriodicTimer link_check_timer_;
    core::PeriodicTimer time_sync_timer_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_GATEWAY_HPP_
