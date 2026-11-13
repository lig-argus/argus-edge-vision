// FC 시각(time_boot_ms)을 RPi monotonic clock 의 시각으로 변환하는 인터페이스
// 구현: TimeSync, 테스트용 FakeFcTimeConverter (onboard test/)
// 사용: FcTelemetryReceiver
#ifndef ARGUS_ONBOARD_FC_TIME_CONVERTER_HPP_
#define ARGUS_ONBOARD_FC_TIME_CONVERTER_HPP_

#include <cstdint>
#include <optional>

namespace argus::onboard {

class IFcTimeConverter {
public:
    IFcTimeConverter() = default;
    virtual ~IFcTimeConverter() = default;
    IFcTimeConverter(const IFcTimeConverter&) = delete;
    IFcTimeConverter& operator=(const IFcTimeConverter&) = delete;
    IFcTimeConverter(IFcTimeConverter&&) = delete;
    IFcTimeConverter& operator=(IFcTimeConverter&&) = delete;

    // fc_time_boot_ms(FC 부팅 후 경과 시간)에 해당하는 RPi CLOCK_MONOTONIC 시각(ns).
    // 유효한 변환값이 없는 경우 nullopt
    virtual std::optional<std::int64_t> to_monotonic_ns(std::uint32_t fc_time_boot_ms) const = 0;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_FC_TIME_CONVERTER_HPP_
