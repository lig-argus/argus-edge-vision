// 인정한 통제기의 마지막 HEARTBEAT 시각과 주소를 조회하는 인터페이스
// 규약: docs/spec/icd.md 5.1 (통제기마다 sysid 와 마지막 주소 기억), 6.1 (HEARTBEAT 3 초 두절)
// 구현: GcsRegistry
// 사용: ControlAuthority (요청 거부 조건), GcsLinkMonitor (두절 판단)
#ifndef ARGUS_ONBOARD_GCS_PRESENCE_HPP_
#define ARGUS_ONBOARD_GCS_PRESENCE_HPP_

#include <cstdint>
#include <optional>

#include "argus/mavlink_gateway/gcs_link.hpp"

namespace argus::mavlink_gateway {

class IGcsPresence {
public:
    IGcsPresence() = default;
    virtual ~IGcsPresence() = default;
    IGcsPresence(const IGcsPresence&) = delete;
    IGcsPresence& operator=(const IGcsPresence&) = delete;
    IGcsPresence(IGcsPresence&&) = delete;
    IGcsPresence& operator=(IGcsPresence&&) = delete;

    // sysid 통제기의 HEARTBEAT 를 마지막으로 받은 시각 (RPi monotonic clock, ns).
    // 인정한 적이 없으면 nullopt
    virtual std::optional<std::int64_t> last_heartbeat_ns(std::uint8_t sysid) const = 0;

    // sysid 통제기가 마지막으로 HEARTBEAT 를 보낸 주소. 인정한 적이 없으면 nullopt
    virtual std::optional<GcsPeer> peer(std::uint8_t sysid) const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_GCS_PRESENCE_HPP_
