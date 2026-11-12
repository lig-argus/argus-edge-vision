// follow/command 요청을 받아 IFollowControl 을 부르고 그 반환값(FollowResult)으로 응답한다
// 규약: docs/spec/process_interfaces.md 5.2 follow/command, 5.2.1 follow/result
// 사용: 추종 제어 프로세스의 main() 이 만든다
// 요청 클래스: BusFollowControlClient
#ifndef ARGUS_VEHICLE_API_BUS_BUS_FOLLOW_CONTROL_SERVER_HPP_
#define ARGUS_VEHICLE_API_BUS_BUS_FOLLOW_CONTROL_SERVER_HPP_

#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/replier.hpp"
#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/follow_control.hpp"

namespace argus::vehicle_api_bus {

// 형식이 맞지 않는 요청은 control 을 부르지 않고 거부로 응답한다 (outcome Rejected,
// request_accepted false, id 는 빈 값, reason "malformed request"). 이 객체가 사라지면 요청을 더
// 받지 않는다
class BusFollowControlServer {
public:
    // control 은 이 객체보다 오래 살아야 한다. control 의 함수는 core::IPoller::process_events()
    // 안에서, 그 함수를 부른 스레드에서 불린다.
    // bus 가 응답하는 쪽을 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusFollowControlServer> create(bus::IBus& bus,
                                                          vehicle_api::IFollowControl& control);

private:
    BusFollowControlServer(std::unique_ptr<bus::IReplier> replier, core::Subscription serving);

    std::unique_ptr<bus::IReplier> replier_;
    core::Subscription serving_;  // 요청 처리 등록. 이 객체가 사라지면 해제된다
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_BUS_FOLLOW_CONTROL_SERVER_HPP_
