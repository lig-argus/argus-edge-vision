// 통제기의 인정과 통제기별 마지막 HEARTBEAT 시각, 주소의 기록
// 규약: docs/spec/icd.md 5.1 (통제기 인정, sysid 겹침),
//      docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 ControlAuthority,
//      MavlinkCommandReceiver, GcsLinkMonitor 에 필요한 인터페이스로만 전달
#ifndef ARGUS_ONBOARD_GCS_REGISTRY_HPP_
#define ARGUS_ONBOARD_GCS_REGISTRY_HPP_

#include <cstdint>
#include <map>
#include <optional>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_handler_registry.hpp"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/gcs_message_source.hpp"
#include "argus/mavlink_gateway/gcs_presence.hpp"

namespace argus::mavlink_gateway {

// sysid 250~253, compid 190 이고 HEARTBEAT 의 type 이 MAV_TYPE_GCS 인 송신자를 통제기로 인정한다.
// 인정은 sysid 마다 한 번이며 그 뒤 이 sysid, compid 190 의 메시지를 넘긴다. 그 밖의 메시지는
// 넘기지 않는다. 같은 sysid 의 HEARTBEAT 를 다른 주소에서 받으면 두 주소에 STATUSTEXT 경고를
// 보내고, 기록하는 주소는 마지막으로 받은 주소로 바꾼다 (메시지는 막지 않는다)
class GcsRegistry : public IGcsMessageSource, public IGcsPresence {
public:
    // link, clock 은 이 객체보다 오래 유지되어야 한다
    GcsRegistry(IGcsLink& link, const core::IClock& clock);

    [[nodiscard]] core::Subscription subscribe(std::uint32_t msgid, Handler handler) override;
    std::optional<std::int64_t> last_heartbeat_ns(std::uint8_t sysid) const override;
    std::optional<GcsPeer> peer(std::uint8_t sysid) const override;

private:
    struct Record {
        GcsPeer peer;
        std::int64_t last_heartbeat_ns = 0;
    };

    void on_heartbeat(const mavlink_message_t& message, const GcsPeer& sender);
    bool recognized(const GcsPeer& sender) const;
    void warn_duplicate(const GcsPeer& recorded, const GcsPeer& sender);

    IGcsLink& link_;
    const core::IClock& clock_;
    std::map<std::uint8_t, Record> records_;  // sysid 마다
    GcsHandlerRegistry handlers_;

    // link 의 등록. msgid 마다 하나이며 처음 subscribe 할 때 만든다. 멤버 중 가장 먼저 소멸하도록
    // 마지막에 선언한다
    core::Subscription heartbeat_subscription_;
    std::map<std::uint32_t, core::Subscription> link_subscriptions_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_GCS_REGISTRY_HPP_
