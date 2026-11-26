// FC 의 RC_CHANNELS, SYS_STATUS 를 받아 최신 값을 보관한다
// 규약: docs/spec/icd.md 2장, 5.4 (복귀 조건)
// 사용: FcGateway 가 하나를 생성하여 IFcRcStatusSource 로 통제기 쪽(RcOverrideMonitor)에 내보낸다
#ifndef ARGUS_ONBOARD_FC_RC_STATUS_RECEIVER_HPP_
#define ARGUS_ONBOARD_FC_RC_STATUS_RECEIVER_HPP_

#include <optional>
#include <vector>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/fc_link_tracker.hpp"
#include "argus/mavlink_gateway/fc_rc_status_source.hpp"
#include "argus/mavlink_gateway/mavlink_connection.hpp"

namespace argus::mavlink_gateway {

// FC(sysid 1, compid 1)가 보낸 메시지만 받는다. 받은 시각은 FcLinkTracker 가 정하며, 이 메시지들은
// FC 연결 상태와 fc_epoch 를 바꾸지 않는다
class FcRcStatusReceiver : public IFcRcStatusSource {
public:
    // connection 과 link 는 이 객체보다 오래 유지되어야 한다
    FcRcStatusReceiver(IMavlinkConnection& connection, FcLinkTracker& link);

    std::optional<RcChannels> rc_channels() const override { return rc_channels_; }
    std::optional<FcSensorHealth> sensor_health() const override { return sensor_health_; }

private:
    void on_rc_channels(const mavlink_message_t& message);
    void on_sys_status(const mavlink_message_t& message);

    FcLinkTracker& link_;
    std::optional<RcChannels> rc_channels_;
    std::optional<FcSensorHealth> sensor_health_;

    // 멤버 중 가장 먼저 소멸하도록 마지막에 선언한다
    std::vector<core::Subscription> subscriptions_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_RC_STATUS_RECEIVER_HPP_
