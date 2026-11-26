// FC 가 보낸 조종기 입력(RC_CHANNELS)과 센서 상태(SYS_STATUS)의 최신 값을 조회하는 인터페이스
// 규약: docs/spec/icd.md 2장 (RC_CHANNELS, SYS_STATUS), 5.4 (복귀 조건)
// 구현: FcRcStatusReceiver
// 사용: RcOverrideMonitor
#ifndef ARGUS_ONBOARD_FC_RC_STATUS_SOURCE_HPP_
#define ARGUS_ONBOARD_FC_RC_STATUS_SOURCE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace argus::mavlink_gateway {

// RC_CHANNELS 가 싣는 채널의 수 (MAVLink RC_CHANNELS 의 chan1_raw ~ chan18_raw)
inline constexpr std::size_t kRcChannelCount = 18;

// 조종기 채널의 값 (RC_CHANNELS)
struct RcChannels {
    // 채널 1~18 의 PWM 값(us). 받지 않은 채널은 UINT16_MAX (MAVLink 의 사용하지 않는 채널 값)
    std::array<std::uint16_t, kRcChannelCount> raw_us{};
    std::uint8_t channel_count = 0;          // FC 가 받는 채널의 수. 0 이면 조종기 신호 없음
    std::int64_t received_monotonic_ns = 0;  // mavlink_gateway 가 받은 시각
    std::uint64_t fc_epoch = 0;
};

// FC 의 센서 상태 중 복귀 조건에 쓰는 것 (SYS_STATUS 의 onboard_control_sensors_*)
struct FcSensorHealth {
    bool rc_receiver_ok = false;  // MAV_SYS_STATUS_SENSOR_RC_RECEIVER 의 present 와 health
    bool local_position_ok =
        false;                 // MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL 의 present 와 health
    bool attitude_ok = false;  // MAV_SYS_STATUS_AHRS 의 health
    std::int64_t received_monotonic_ns = 0;
    std::uint64_t fc_epoch = 0;
};

class IFcRcStatusSource {
public:
    IFcRcStatusSource() = default;
    virtual ~IFcRcStatusSource() = default;
    IFcRcStatusSource(const IFcRcStatusSource&) = delete;
    IFcRcStatusSource& operator=(const IFcRcStatusSource&) = delete;
    IFcRcStatusSource(IFcRcStatusSource&&) = delete;
    IFcRcStatusSource& operator=(IFcRcStatusSource&&) = delete;

    // 가장 최근 값. 받지 못했으면 nullopt. fc_epoch 가 바뀌면 이전 값은 삭제된다
    virtual std::optional<RcChannels> rc_channels() const = 0;
    virtual std::optional<FcSensorHealth> sensor_health() const = 0;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FC_RC_STATUS_SOURCE_HPP_
