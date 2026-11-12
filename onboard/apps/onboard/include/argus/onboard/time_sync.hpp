// TIMESYNC 로 FC 시각과 RPi monotonic clock 의 차를 구한다
// 규약: docs/spec/icd.md 1.1 (compid 191), 2장 (TIMESYNC)
// 사용: onboard 의 main 이 send_request() 를 주기 작업으로 등록, FcTelemetryReceiver 가 변환값 사용
#ifndef ARGUS_ONBOARD_TIME_SYNC_HPP_
#define ARGUS_ONBOARD_TIME_SYNC_HPP_

#include <cstdint>
#include <optional>

#include "argus/core/clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/mavlink.h"
#include "argus/onboard/fc_epoch_source.hpp"
#include "argus/onboard/fc_time_converter.hpp"
#include "argus/onboard/mavlink_connection.hpp"

namespace argus::onboard {

struct TimeSyncConfig {
    // 요청 송신부터 응답 수신까지의 시간이 이 값을 넘는 응답은 사용하지 않는다. 왕복 시간이 긴
    // 응답은 FC 가 응답한 시점의 추정 오차가 크다 (오차의 상한은 왕복 시간의 절반)
    std::int64_t max_round_trip_ns = 50'000'000;

    // 마지막으로 사용한 응답의 수신 시각부터 이 시간이 지난 경우 변환값을 무효로 한다
    std::int64_t max_age_ns = 5'000'000'000;
};

class TimeSync : public IFcTimeConverter {
public:
    // 인자로 받은 객체들은 이 객체보다 오래 유지되어야 한다.
    // epoch_source 의 fc_epoch 가 증가한 경우 변환값을 폐기한다. 두절과 FC 재부팅 후에는 FC 시각과
    // RPi 시각의 차가 이전과 같다고 볼 수 없으므로, 새 응답으로 변환값을 다시 구하기 전까지 무효로
    // 한다
    TimeSync(IMavlinkConnection& connection, const core::IClock& clock,
             IFcEpochSource& epoch_source, TimeSyncConfig config = {});

    // FC 로 TIMESYNC 요청 하나를 송신한다. 주기적으로 호출한다.
    // 이전 요청의 응답을 받지 못한 상태에서 호출한 경우 이전 요청은 폐기하며, 이후에 도착하는 이전
    // 요청의 응답은 사용하지 않는다. false = 송신 실패 (응답을 기다리는 요청은 그대로 유지)
    bool send_request();

    std::optional<std::int64_t> to_monotonic_ns(std::uint32_t fc_time_boot_ms) const override;

    // 유효한 변환값이 있는지 (to_monotonic_ns() 가 값을 반환하는지)
    bool valid() const;

    // 사용하지 않고 폐기한 응답 수 (대조 실패, 왕복 시간 초과)
    std::uint64_t rejected_count() const { return rejected_count_; }

private:
    void on_timesync(const mavlink_message_t& message);
    // 변환값과 응답을 기다리는 요청을 폐기한다
    void invalidate();

    IMavlinkConnection& connection_;
    const core::IClock& clock_;
    TimeSyncConfig config_;

    // 응답을 기다리는 요청의 송신 시각(요청의 ts1). 기다리는 요청이 없는 경우 nullopt
    std::optional<std::int64_t> pending_request_ns_;

    // RPi monotonic 시각(ns) = FC 시각(ns) + offset_ns_
    std::optional<std::int64_t> offset_ns_;
    std::int64_t offset_updated_ns_ = 0;  // offset_ns_ 를 구한 응답의 수신 시각
    std::uint64_t rejected_count_ = 0;

    core::Subscription timesync_subscription_;
    core::Subscription epoch_subscription_;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_TIME_SYNC_HPP_
