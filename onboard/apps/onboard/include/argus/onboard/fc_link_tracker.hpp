// FC 연결의 두절 판단과 fc_epoch 관리
// 규약: docs/spec/icd.md 6.2 (FC 연결 두절), docs/spec/process_interfaces.md 3장 (fc_epoch)
// 사용: onboard 의 main 이 하나를 생성하여 check_timeout() 을 주기 작업으로 등록.
//      FC 메시지를 받는 클래스(FcTelemetryReceiver)가 on_fc_message() 를 호출하고,
//      fc_epoch 에 묶인 값을 가진 클래스가 subscribe_epoch_changes() 로 변경을 받는다
#ifndef ARGUS_ONBOARD_FC_LINK_TRACKER_HPP_
#define ARGUS_ONBOARD_FC_LINK_TRACKER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/onboard/fc_epoch_source.hpp"
#include "argus/onboard/handler_list.hpp"
#include "argus/onboard/health_state.hpp"

namespace argus::onboard {

// FC 의 메시지(HEARTBEAT 포함)를 이 시간 동안 받지 못한 경우 FC 연결 두절로 판단 (icd.md 6.2)
inline constexpr std::int64_t kFcLinkTimeoutNs = 2'000'000'000;

// time_boot_ms 를 FC 재부팅 감지에 사용하는 FC 메시지의 종류. 종류마다 직전 값과 비교한다.
// 종류가 다른 메시지는 FC 가 값을 읽는 시점이 달라 time_boot_ms 의 순서가 바뀔 수 있으므로 서로
// 비교하지 않는다
enum class FcTimedMessage : std::uint8_t {
    Attitude,
    LocalPosition,
    GlobalPosition,
    DownwardDistance,
    Count,  // 종류의 수. 메시지 종류가 아님
};

class FcLinkTracker : public IFcEpochSource {
public:
    // clock 과 health 는 이 객체보다 오래 유지되어야 한다
    FcLinkTracker(const core::IClock& clock, HealthState& health);

    // FC 메시지를 받을 때마다 호출한다. 수신 시각(RPi monotonic clock, ns)을 반환한다.
    // fc_epoch 는 다음 경우에 1 증가한다
    // - FC 메시지를 처음 받은 경우 (0 → 1)
    // - 두절 후 다시 받은 경우
    // - 같은 종류의 직전 메시지보다 time_boot_ms 가 작아진 경우 (FC 재부팅)
    // 증가한 경우 반환하기 전에 subscribe_epoch_changes() 의 handler 를 호출한다. 호출자는 반환
    // 후의 fc_epoch() 를 그 메시지의 fc_epoch 로 사용한다. 한계: FC 가 두절 시간(2 s)보다 짧은 시간
    // 안에 재부팅한 경우, 재부팅은 time_boot_ms 를 가진 메시지가 도착한 시점에 감지된다. 그 전에
    // 도착한 HEARTBEAT 는 이전 fc_epoch 로 기록된다
    std::int64_t on_fc_message(FcTimedMessage kind, std::uint32_t time_boot_ms);

    // 재부팅 감지에 사용하지 않는 메시지용 (time_boot_ms 가 없는 HEARTBEAT 등)
    std::int64_t on_fc_message();

    // 두절 여부를 판단하여 HealthState 의 ARGUS_HEALTH_FC_CONNECTED 를 갱신한다. 주기적으로
    // 호출한다 (두절은 메시지가 오지 않는 상태이므로 메시지 수신으로는 판단할 수 없음)
    void check_timeout();

    // FC 메시지를 받은 적이 있고 마지막 수신 후 두절 시간이 지나지 않은 경우 true. 호출 시점의
    // 시각으로 판단하므로 check_timeout() 의 호출 주기와 무관하다
    bool connected() const;

    std::uint64_t fc_epoch() const override { return fc_epoch_; }

    // 호출 시점: fc_epoch(), connected(), HealthState 가 새 값으로 바뀐 후
    [[nodiscard]] core::Subscription subscribe_epoch_changes(EpochHandler handler) override;

private:
    bool timed_out(std::int64_t now_ns) const;
    // fc_epoch 를 1 증가시키고 재부팅 비교용 값을 비운다. handler 는 호출하지 않는다
    void advance_epoch();

    const core::IClock& clock_;
    HealthState& health_;

    // 마지막으로 판단한 연결 상태 (HealthState 에 기록한 값과 같음)
    bool connected_ = false;
    std::uint64_t fc_epoch_ = 0;
    std::int64_t last_received_ns_ = 0;
    std::array<std::optional<std::uint32_t>, static_cast<std::size_t>(FcTimedMessage::Count)>
        last_time_boot_ms_{};
    HandlerList<std::uint64_t> epoch_handlers_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_FC_LINK_TRACKER_HPP_
