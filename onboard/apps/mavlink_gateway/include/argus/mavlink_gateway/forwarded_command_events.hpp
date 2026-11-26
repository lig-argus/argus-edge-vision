// 통제기의 명령을 FC 로 전달한 것과 FC 의 응답을 알리는 인터페이스
// 규약: docs/spec/icd.md 4.3, 5.2, 5.3 (착륙 잠금의 시작과 FC 거부 시 해제),
//      5.4 (조종사 개입 판정의 통제기 모드 명령)
// 구현: GcsFcForwarder
// 사용: LandingLock (MAV_CMD_NAV_LAND), RcOverrideMonitor (MAV_CMD_DO_SET_MODE)
#ifndef ARGUS_ONBOARD_FORWARDED_COMMAND_EVENTS_HPP_
#define ARGUS_ONBOARD_FORWARDED_COMMAND_EVENTS_HPP_

#include <cstdint>
#include <functional>
#include <optional>

#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

enum class ForwardStage : std::uint8_t {
    Forwarded,  // 통제기의 COMMAND_LONG 을 FC 로 보냄
    Answered,   // 대상이 통제기인 FC 의 COMMAND_ACK 를 받음 (IN_PROGRESS 포함)
};

struct ForwardedCommandEvent {
    GcsPeer sender;             // 명령을 보낸 통제기
    std::uint16_t command = 0;  // MAV_CMD
    ForwardStage stage = ForwardStage::Forwarded;
    std::optional<MAV_RESULT> result;  // Answered 일 때 FC 의 결과

    // MAV_CMD_DO_SET_MODE 일 때 요청한 PX4 모드 (해석하지 못하면 Unknown)
    std::optional<vehicle_api::FlightMode> requested_mode;
};

class IForwardedCommandEvents {
public:
    using Handler = std::function<void(const ForwardedCommandEvent&)>;

    IForwardedCommandEvents() = default;
    virtual ~IForwardedCommandEvents() = default;
    IForwardedCommandEvents(const IForwardedCommandEvents&) = delete;
    IForwardedCommandEvents& operator=(const IForwardedCommandEvents&) = delete;
    IForwardedCommandEvents(IForwardedCommandEvents&&) = delete;
    IForwardedCommandEvents& operator=(IForwardedCommandEvents&&) = delete;

    // FC 로 전달한 COMMAND_LONG 마다 Forwarded 를, 대상이 통제기인 FC 의 COMMAND_ACK 마다 Answered
    // 를 handler 에 알린다. Forwarded 는 FC 로 보낸 직후에 부른다. handler 는
    // core::IPoller::process_events() 안에서 불린다. 반환된 Subscription 이 사라지면 그 handler 는
    // 더 불리지 않는다
    [[nodiscard]] virtual core::Subscription subscribe(Handler handler) = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FORWARDED_COMMAND_EVENTS_HPP_
