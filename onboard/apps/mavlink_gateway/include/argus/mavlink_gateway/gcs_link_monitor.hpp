// 제어권을 가진 통제기의 HEARTBEAT 두절 판단과 처리
// 규약: docs/spec/icd.md 6.1, docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 하나를 생성하여 tick() 을 kGcsLinkCheckPeriod 주기 작업으로 등록
#ifndef ARGUS_ONBOARD_GCS_LINK_MONITOR_HPP_
#define ARGUS_ONBOARD_GCS_LINK_MONITOR_HPP_

#include <chrono>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink_gateway/control_authority.hpp"
#include "argus/mavlink_gateway/gcs_presence.hpp"
#include "argus/mavlink_gateway/health_state.hpp"
#include "argus/mavlink_gateway/request_header_factory.hpp"
#include "argus/vehicle_api/follow_control_client.hpp"
#include "argus/vehicle_api/follow_status_source.hpp"

namespace argus::mavlink_gateway {

// tick() 의 호출 주기. 두절 기준(3 s)의 1/30 이며 두절을 알아차리는 지연에만 영향을 준다
inline constexpr std::chrono::milliseconds kGcsLinkCheckPeriod{100};

// 추종 제어에 보내는 정지 요청의 유효 시간과 응답 대기 시간
inline constexpr std::chrono::milliseconds kLinkLossStopTimeout{1000};

struct GcsLinkMonitorInputs {
    const IGcsPresence& presence;
    const IControlAuthority& authority;
    const core::IClock& clock;
    const vehicle_api::IFollowStatusSource& follow_status;
    vehicle_api::IFollowControlClient& follow_control;
    RequestHeaderFactory& request_headers;
    HealthState& health;
};

// 제어권을 가진 통제기가 있고 그 HEARTBEAT 가 kGcsHeartbeatTimeoutNs 이내이면
// ARGUS_HEALTH_GCS_LINK = 1, 아니면 0. 제어권을 가진 통제기의 두절이 시작될 때 추종 또는 스캔
// 중이면 추종 제어에 정지(StopReason::LinkLoss)를 요청한다 (TRACK_STOP 과 같은 절차). 다시 받으면
// 비트만 복구하며 추종은 재개하지 않는다. 정지 요청의 결과는 기다리지 않으며 실패하면 로그에
// 남긴다 (추종 제어가 멈춘 경우는 icd 6.4 ⓑ 의 처리가 맡는다)
class GcsLinkMonitor {
public:
    // inputs 의 객체들은 이 객체보다 오래 유지되어야 한다
    explicit GcsLinkMonitor(const GcsLinkMonitorInputs& inputs) : inputs_(inputs) {}

    void tick();

private:
    void request_stop();

    GcsLinkMonitorInputs inputs_;
    bool link_ok_ = false;
    core::Subscription stop_request_;  // 응답을 기다리는 정지 요청
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_GCS_LINK_MONITOR_HPP_
