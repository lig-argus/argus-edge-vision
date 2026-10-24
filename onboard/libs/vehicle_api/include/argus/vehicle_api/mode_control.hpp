// 비행 모드 전환을 요청하는 인터페이스
// 규약: docs/spec/process_interfaces.md 4장, 5.2 control/mode_request
// 구현: onboard FC 통신의 ZeroMQ 어댑터(vehicle_api_zmq)
//      테스트용 FakeModeControl(vehicle_api_testing)
// 사용: 추종 제어
#ifndef ARGUS_VEHICLE_API_MODE_CONTROL_HPP_
#define ARGUS_VEHICLE_API_MODE_CONTROL_HPP_

#include <cstdint>

#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

// 반환값은 onboard 가 요청을 받아들였는지(수락)만 뜻한다. FC 의 실제 모드 전환 완료와
// Offboard 이탈은 IVehicleState::on_mode_event 로 온다.
// valid_until_monotonic_ns 가 지난 요청은 onboard 가 FC 로 보내지 않고 거부한다.
class IModeControl {
public:
    IModeControl() = default;
    virtual ~IModeControl() = default;
    IModeControl(const IModeControl&) = delete;
    IModeControl& operator=(const IModeControl&) = delete;
    IModeControl(IModeControl&&) = delete;
    IModeControl& operator=(IModeControl&&) = delete;

    virtual ModeReply request_offboard(std::int64_t valid_until_monotonic_ns) = 0;
    virtual ModeReply request_hold(std::int64_t valid_until_monotonic_ns) = 0;

    // Hold 상태에서 절대 heading(rad, NED, 북쪽 0) 으로 회전 (docs/spec/icd.md E3)
    virtual ModeReply request_heading(double heading_rad,
                                      std::int64_t valid_until_monotonic_ns) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_MODE_CONTROL_HPP_
