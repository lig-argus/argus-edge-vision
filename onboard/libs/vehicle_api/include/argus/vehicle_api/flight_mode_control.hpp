// 비행 모드 전환 요청을 받는 인터페이스 (구현용. 응답을 바로 반환한다)
// 규약: docs/spec/process_interfaces.md 3장(요청 공통 필드), 4장, 5.2 control/mode_request
// 구현: (다음 PR) onboard 의 MavlinkFlightModeControl (FC 로 DO_SET_MODE 송신),
//      테스트용 FakeFlightModeControl (vehicle_api_testing)
// 사용: (다음 PR) onboard 의 ZeroMQ 어댑터 ZmqFlightModeControlServer 가 REQ 를 받아 부르고
//      응답을 REP 로 보낸다. 추종 제어 프로세스에서 요청하는 쪽은 IFlightModeControlClient 를 쓴다
#ifndef ARGUS_VEHICLE_API_FLIGHT_MODE_CONTROL_HPP_
#define ARGUS_VEHICLE_API_FLIGHT_MODE_CONTROL_HPP_

#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

// 반환값은 onboard 가 요청을 받아들였는지(수락)만 뜻한다. FC 의 실제 모드 전환 완료와
// Offboard 이탈은 IVehicleTelemetry::subscribe_flight_mode_events 로 온다.
// header.valid_until_monotonic_ns 가 지난 요청은 onboard 가 FC 로 보내지 않고 거부한다.
// 요청의 취소와, stop 이나 RC 개입 때 FC 로 보내려고 대기 중인 송신의 폐기는 onboard 가 맡는다.
// 수락 뒤 실패의 전달과 늦게 도착한 응답의 무효화는 process_interfaces.md 미결 1 에서 정한다.
class IFlightModeControl {
public:
    IFlightModeControl() = default;
    virtual ~IFlightModeControl() = default;
    IFlightModeControl(const IFlightModeControl&) = delete;
    IFlightModeControl& operator=(const IFlightModeControl&) = delete;
    IFlightModeControl(IFlightModeControl&&) = delete;
    IFlightModeControl& operator=(IFlightModeControl&&) = delete;

    virtual FlightModeReply request_offboard(const RequestHeader& header) = 0;

    virtual FlightModeReply request_hold(const RequestHeader& header) = 0;

    // Hold 상태에서 절대 heading(rad, NED, 북쪽 0) 으로 회전 (docs/spec/icd.md E3)
    virtual FlightModeReply request_heading(const RequestHeader& header, double heading_rad) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_FLIGHT_MODE_CONTROL_HPP_
