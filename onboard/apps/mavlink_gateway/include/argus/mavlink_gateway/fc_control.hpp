// mavlink_gateway 가 FC 로 보내는 명령(Offboard 진입 명령, setpoint)의 송신
// 규약: docs/spec/icd.md 4.2, 5장, 6.2, 6.4, 6.6, docs/spec/process_interfaces.md 3장, 5.1, 5.2
// 사용: BusFlightModeControlServer 와 BusVelocitySetpointSubscriber (vehicle_api_bus)가 추종 제어의
//      요청과 setpoint 를 전달. mavlink_gateway 의 main 이 subscribe_mode_events() 를 fc/mode_event
//      발행에 연결하고 tick() 을 20 Hz 주기 작업으로 등록
#ifndef ARGUS_ONBOARD_FC_CONTROL_HPP_
#define ARGUS_ONBOARD_FC_CONTROL_HPP_

#include <cstdint>
#include <functional>
#include <optional>

#include "argus/core/clock.hpp"
#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_command_lock_release.hpp"
#include "argus/mavlink_gateway/fc_command_policy.hpp"
#include "argus/mavlink_gateway/fc_command_state.hpp"
#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/mavlink_gateway/mavlink_connection.hpp"
#include "argus/mavlink_gateway/setpoint_inbox.hpp"
#include "argus/mavlink_gateway/stop_requester.hpp"
#include "argus/vehicle_api/flight_mode_control.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api/velocity_setpoint_sink.hpp"

namespace argus::mavlink_gateway {

struct FcControlConfig {
    FcCommandPolicyConfig policy;

    // 받은 후 이 시간 이내인 자세와 local 위치로만 제자리 유지의 목표를 정한다
    std::int64_t pose_max_age_ns = 200'000'000;

    // 추종 제어가 보낸 setpoint 의 유효 시간 상한
    std::int64_t setpoint_max_valid_ns = 1'000'000'000;

    // setpoint 의 송신과 송신 사이가 이 시간을 넘으면 끊긴 것으로 본다 (20 Hz 기준 4회)
    std::int64_t stream_max_gap_ns = 200'000'000;
};

// 상태(FcCommandState) 하나를 보관하고, 요청과 주기 작업마다 fc_command_policy 의 판단을 실행한다.
// 판단은 fc_command_policy 가, MAVLink 메시지의 구성은 setpoint_encoding 과 mode_command_encoding
// 이 맡으며, 이 클래스는 입력을 모으고 결과를 송신한다.
//
// - 주기마다 setpoint 를 하나만 보낸다. FC 가 Offboard 일 수 있는 상태(Entering, Following,
//   CancellingEntry, Hovering)에서는 반드시 하나를 보낸다
// - setpoint 의 송신을 끝내는 것은 FC 가 Offboard 가 아님을 비행 상태로 확인한 경우뿐이다
// - 요청의 응답은 수락 여부만 뜻한다. FC 의 실제 모드 변경은 subscribe_mode_events() 로 전달한다
class FcControl : public vehicle_api::IFlightModeControl,
                  public vehicle_api::IVelocitySetpointSink,
                  public IStopRequester,
                  public IFcCommandLockRelease {
public:
    using ModeEventHandler = std::function<void(const vehicle_api::FlightModeEvent&)>;

    // 인자로 받은 객체들은 이 객체보다 오래 유지되어야 한다
    FcControl(IMavlinkConnection& connection, const IFcStateSource& fc_state,
              const core::IClock& clock, FcControlConfig config = {});

    vehicle_api::FlightModeReply request_offboard(
        const vehicle_api::RequestHeader& header) override;
    vehicle_api::FlightModeReply request_hold(const vehicle_api::RequestHeader& header) override;

    // 추종 제어가 보낸 setpoint 를 보관한다. FC 로의 송신은 tick() 이 한다
    void send(const vehicle_api::VelocitySetpoint& setpoint) override;

    bool stop_from_onboard() override;
    bool release_lock() override;

    // FC 의 실제 모드가 바뀔 때마다 handler 를 호출한다. 현재 fc_epoch 의 비행 상태를 처음 받은
    // 경우와, FC 가 이미 Offboard 인 상태에서 받은 진입 요청을 수락한 경우에도 호출한다.
    // Offboard 로의 변경은 한 번만 전달한다: 진입의 확인을 기다리는 중에 본 변경은 FC 의 응답까지
    // 받은 때에 그 요청의 command_id 를 넣어 전달하고, 요청이 실패로 끝난 경우에는 command_id
    // 없이 전달한다. handler 는 tick() 과 요청 함수 안에서 호출된다. 반환된 Subscription 이
    // 소멸하면 등록이 해제된다
    [[nodiscard]] core::Subscription subscribe_mode_events(ModeEventHandler handler);

    // 상태를 갱신하고 setpoint 를 하나 송신한다. 주기적으로 호출한다 (20 Hz)
    void tick();

    const FcCommandState& state() const { return state_; }

    // FC 가 Offboard 이고 mavlink_gateway 가 제자리 유지의 위치 setpoint 를 보내고 있는지
    // (fc/state 의 offboard_hover, icd.md 4.2 의 정지 상태). 속도 0 을 보내는 동안은 false
    bool offboard_hover() const { return offboard_hover_; }

private:
    FcObservation observe() const;
    std::optional<HoverTarget> fresh_pose(const vehicle_api::VehicleState& vehicle,
                                          std::int64_t now_ns) const;

    // 요청 없이 일어나는 전환을 적용하고, FC 의 모드 변경을 전달한다
    void sync(const FcObservation& observation);
    void apply(const FcCommandState& state, const FcCommandEffects& effects,
               const FcObservation& observation);
    // 진입 명령을 송신한다. 실패한 경우 상태에 반영하고 false 를 반환
    bool send_entry_command(const FcObservation& observation);
    void send_setpoint(const SetpointChoice& choice, std::int64_t now_ns);
    void report_mode_change(const FcObservation& observation);
    void emit(vehicle_api::FlightMode mode, std::int64_t received_ns,
              std::optional<vehicle_api::CommandId> command_id,
              vehicle_api::OffboardExitCause exit_cause);
    void on_command_ack(const mavlink_message_t& message);

    IMavlinkConnection& connection_;
    const IFcStateSource& fc_state_;
    const core::IClock& clock_;
    FcControlConfig config_;

    FcCommandState state_;
    SetpointInbox inbox_;
    SetpointStreamRecord stream_;
    bool offboard_hover_ = false;

    // 마지막으로 전달한 FC 의 모드와 그 fc_epoch. 전달한 적이 없으면 nullopt
    std::optional<vehicle_api::FlightMode> reported_mode_;
    std::uint64_t reported_fc_epoch_ = 0;

    core::HandlerList<const vehicle_api::FlightModeEvent&> mode_event_handlers_;
    core::Subscription ack_subscription_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_CONTROL_HPP_
