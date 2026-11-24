// 추종 제어가 보낸 속도 setpoint 의 보관과, FC 로 송신한 기록
// 사용: FcControl
#ifndef ARGUS_ONBOARD_SETPOINT_INBOX_HPP_
#define ARGUS_ONBOARD_SETPOINT_INBOX_HPP_

#include <algorithm>
#include <cstdint>
#include <optional>

#include "argus/mavlink_gateway/setpoint_validation.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// 추종 제어가 보낸 가장 최근의 setpoint 하나를 보관한다
class SetpointInbox {
public:
    // max_valid_ns: setpoint 의 유효 시간 상한. 추종 제어가 먼 시각을 넣어도 이 시간 후에는 쓰지
    // 않는다
    explicit SetpointInbox(std::int64_t max_valid_ns) : max_valid_ns_(max_valid_ns) {}

    // 유효 시간이 지난 값과 FC 로 송신할 수 없는 값(NaN, 무한대)은 보관하지 않으며, 그 전에 받은
    // 값을 그대로 둔다
    void put(const vehicle_api::VelocitySetpoint& setpoint, std::int64_t now_ns) {
        if (now_ns > setpoint.valid_until_monotonic_ns || !has_sendable_values(setpoint)) {
            return;
        }
        setpoint_ = setpoint;
        setpoint_.valid_until_monotonic_ns =
            std::min(setpoint.valid_until_monotonic_ns, now_ns + max_valid_ns_);
        received_ns_ = now_ns;
    }

    // 보관한 값의 유효 시간이 지나지 않은 경우 그 값을 받은 시각
    std::optional<std::int64_t> valid_received_ns(std::int64_t now_ns) const {
        if (!received_ns_.has_value() || now_ns > setpoint_.valid_until_monotonic_ns) {
            return std::nullopt;
        }
        return received_ns_;
    }

    const vehicle_api::VelocitySetpoint& setpoint() const { return setpoint_; }

private:
    std::int64_t max_valid_ns_;
    vehicle_api::VelocitySetpoint setpoint_;
    std::optional<std::int64_t> received_ns_;
};

// 추종 제어의 setpoint 를 FC 로 끊기지 않고 송신해 온 시간을 센다.
// PX4 는 Offboard 로 전환하기 전부터 setpoint 를 받고 있어야 전환을 허용한다 (icd.md 4.2)
class SetpointStreamRecord {
public:
    // max_gap_ns: 송신과 송신 사이가 이 시간을 넘으면 끊긴 것으로 본다
    explicit SetpointStreamRecord(std::int64_t max_gap_ns) : max_gap_ns_(max_gap_ns) {}

    void record_sent(std::int64_t now_ns) {
        if (!streaming(now_ns)) {
            started_ns_ = now_ns;
        }
        last_ns_ = now_ns;
        sent_ = true;
    }

    // 추종 제어의 setpoint 가 아닌 것을 송신했거나 송신하지 않은 경우
    void reset() { sent_ = false; }

    // 끊기지 않고 송신해 온 시간. 끊긴 경우 0
    std::int64_t streaming_for_ns(std::int64_t now_ns) const {
        return streaming(now_ns) ? last_ns_ - started_ns_ : 0;
    }

private:
    bool streaming(std::int64_t now_ns) const { return sent_ && now_ns - last_ns_ <= max_gap_ns_; }

    std::int64_t max_gap_ns_;
    bool sent_ = false;  // reset() 후 송신한 적이 있는지
    std::int64_t started_ns_ = 0;
    std::int64_t last_ns_ = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_SETPOINT_INBOX_HPP_
