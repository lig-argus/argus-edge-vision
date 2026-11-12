// vehicle_api_bus 가 bus 로 주고받는 메시지의 타입 (bus::MessageType) 과, vehicle_api 에 맞는 값
// 타입이 없는 요청 메시지의 값 타입.
// 메시지 하나에 C++ 값 타입 하나가 대응한다. 이름은 메시지 정의(argus_msg/*.proto)의 메시지 이름과
// 같다. 통신 구현은 값 타입으로 자기 변환 방법을 찾는다 (ZmqBus 는 zmq_codecs() 의 표)
// 규약: docs/spec/process_interfaces.md 5장
#ifndef ARGUS_VEHICLE_API_BUS_MESSAGE_TYPES_HPP_
#define ARGUS_VEHICLE_API_BUS_MESSAGE_TYPES_HPP_

#include <cstdint>
#include <string_view>
#include <variant>

#include "argus/bus/message_type.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api_bus {

// control/mode_request 의 요청 하나. action 이 IFlightModeControl 의 함수에 대응한다
struct FlightModeRequest {
    enum class Action : std::uint8_t {
        Hold,
        Offboard,
    };

    vehicle_api::RequestHeader header;

    // 기본값은 Hold 다. 값을 채우지 않은 요청이 전달되어도 기체가 제자리에 머무는 쪽이 되게 한다
    // (Offboard 는 속도 명령으로 기체를 움직이는 모드를 요청한다)
    Action action = Action::Hold;
};

// follow/command 의 요청 하나. action 의 타입이 IFollowControl 의 함수 3개(track_start, stop,
// scan_start)에 대응한다
struct FollowCommandRequest {
    vehicle_api::RequestHeader header;

    // 기본값은 stop (이유 Onboard) 이다. 값을 채우지 않은 요청이 전달되어도 추종과 Scan 을
    // 끝내는 쪽이 되게 한다 (track_start 와 scan_start 는 기체를 움직인다). 이유를 Onboard 로 두는
    // 것은 운용자 입력이나 링크 두절로 생긴 요청이 아니기 때문이다
    std::variant<vehicle_api::StopReason, vehicle_api::TrackTarget, vehicle_api::ScanRequest>
        action = vehicle_api::StopReason::Onboard;
};

// 이름은 "..."sv 로 쓴다. const char* 로 std::string_view 를 만드는 생성자는 noexcept 가 아니어서
// namespace 범위 상수의 초기화에 쓰면 정적 분석이 경고한다
using std::string_view_literals::operator""sv;

// 토픽 (argus/protocol/topics.hpp 의 이름 순서)
// fc/attitude
inline const bus::MessageType kFcAttitudeType =
    bus::message_type<vehicle_api::Attitude>("argus.msg.FcAttitude"sv);
// fc/state. local, downward_distance, global, status 만 나른다. attitude 는 나르지 않으므로
// 받는 쪽은 attitude 를 읽지 않는다 (자세는 fc/attitude 로 보낸다)
inline const bus::MessageType kFcStateType =
    bus::message_type<vehicle_api::VehicleState>("argus.msg.FcState"sv);
// fc/mode_event
inline const bus::MessageType kFcModeEventType =
    bus::message_type<vehicle_api::FlightModeEvent>("argus.msg.FcModeEvent"sv);
// control/setpoint
inline const bus::MessageType kControlSetpointType =
    bus::message_type<vehicle_api::VelocitySetpoint>("argus.msg.ControlSetpoint"sv);

// 요청-응답
// control/mode_request 의 요청과 응답
inline const bus::MessageType kControlModeRequestType =
    bus::message_type<FlightModeRequest>("argus.msg.ControlModeRequest"sv);
inline const bus::MessageType kControlModeReplyType =
    bus::message_type<vehicle_api::FlightModeReply>("argus.msg.ControlModeReply"sv);
// follow/command 의 요청과 응답
inline const bus::MessageType kFollowCommandType =
    bus::message_type<FollowCommandRequest>("argus.msg.FollowCommand"sv);
inline const bus::MessageType kFollowResultType =
    bus::message_type<vehicle_api::FollowResult>("argus.msg.FollowResult"sv);

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_MESSAGE_TYPES_HPP_
