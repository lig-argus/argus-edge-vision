// 통제기의 메시지를 FC 로 전달하고 FC 의 COMMAND_ACK 를 명령을 보낸 통제기에 전달한다
// 규약: docs/spec/icd.md 4.1, 4.3, 5.2, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성한다. 착륙 잠금(LandingLock)과 조종사 개입 판정
//      (RcOverrideMonitor)은 IForwardedCommandEvents 로 전달과 응답을 받는다
#ifndef ARGUS_ONBOARD_GCS_FC_FORWARDER_HPP_
#define ARGUS_ONBOARD_GCS_FC_FORWARDER_HPP_

#include <cstdint>
#include <string>

#include "argus/core/clock.hpp"
#include "argus/core/handler_list.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/command_activity.hpp"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/fc_command_status.hpp"
#include "argus/mavlink_gateway/fc_state_source.hpp"
#include "argus/mavlink_gateway/forwarded_command_events.hpp"
#include "argus/mavlink_gateway/gcs_forward_policy.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/gcs_message_source.hpp"
#include "argus/mavlink_gateway/gcs_presence.hpp"
#include "argus/mavlink_gateway/landing_lock_state.hpp"
#include "argus/mavlink_gateway/mavlink_connection.hpp"
#include "argus/mavlink_gateway/rc_override_state.hpp"
#include "argus/mavlink_gateway/stop_requester.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"

namespace argus::mavlink_gateway {

// 전달 판단의 입력과 실행 대상. 모두 GcsFcForwarder 보다 오래 유지되어야 한다
struct GcsFcForwarderInputs {
    IGcsMessageSource& messages;     // 인정한 통제기의 메시지 수신
    IGcsLink& link;                  // 직접 응답과 FC 의 COMMAND_ACK 송신
    const IGcsPresence& presence;    // COMMAND_ACK 를 받을 통제기의 주소
    IMavlinkConnection& connection;  // FC 로 전달, FC 의 COMMAND_ACK 수신
    const core::IClock& clock;
    const IControlAuthority& authority;
    const IFcStateSource& fc_state;
    const IFcCommandStatus& fc_command_status;
    const ICommandActivity& commands;
    const vehicle_api::IFollowStatusSource& follow_status;
    const ILandingLockState& landing_lock;
    const IRcOverrideState& rc_override;
    IStopRequester& stop_requester;  // 통제기의 HOLD
};

// 통제기 메시지마다 입력을 모아 decide_forward() 를 부르고 결과를 실행한다. 판단은
// gcs_forward_policy 가 맡는다.
// - FC 로 전달: 받은 메시지를 그대로(통제기의 sysid, compid 유지) 보낸다. FC 의 COMMAND_ACK 대상이
//   통제기가 되어 mavlink_gateway 자신의 명령의 ACK 와 구분된다 (icd 4.1)
// - 직접 응답: COMMAND_ACK 와 이유(STATUSTEXT)를 보낸 통제기에만, 송신 compid 195 로 보낸다
// - FC 의 COMMAND_ACK 중 대상이 통제기인 것을 그 통제기의 마지막 주소로만 보낸다
class GcsFcForwarder : public IForwardedCommandEvents {
public:
    explicit GcsFcForwarder(const GcsFcForwarderInputs& inputs);

    [[nodiscard]] core::Subscription subscribe(Handler handler) override;

private:
    void on_gcs_message(const mavlink_message_t& message, const GcsPeer& sender);
    void on_fc_command_ack(const mavlink_message_t& message);
    ForwardObservation observe(const GcsPeer& sender) const;
    void execute(const ForwardDecision& decision, const mavlink_message_t& message,
                 const GcsPeer& sender, const GcsMessageRequest& request);
    void reply(const GcsPeer& sender, std::uint16_t command, MAV_RESULT result,
               const std::string& reason);

    GcsFcForwarderInputs inputs_;
    core::HandlerList<const ForwardedCommandEvent&> handlers_;
    core::Subscription heartbeat_subscription_;
    core::Subscription command_long_subscription_;
    core::Subscription command_int_subscription_;
    core::Subscription manual_control_subscription_;
    core::Subscription param_request_read_subscription_;
    core::Subscription param_set_subscription_;
    core::Subscription command_ack_subscription_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_GCS_FC_FORWARDER_HPP_
