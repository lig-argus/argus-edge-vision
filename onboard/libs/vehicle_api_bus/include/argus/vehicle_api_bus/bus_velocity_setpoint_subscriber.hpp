// control/setpoint 토픽을 구독해 vehicle_api::VelocitySetpoint 로 변환해 전달한다
// 규약: docs/spec/process_interfaces.md 5.1 control/setpoint
// 사용: onboard 프로세스. 받은 값을 FC 로 보내는 주기와 유효 시간 판단은 사용하는 코드가 맡는다
// 발행 클래스: BusVelocitySetpointPublisher
#ifndef ARGUS_VEHICLE_API_BUS_BUS_VELOCITY_SETPOINT_SUBSCRIBER_HPP_
#define ARGUS_VEHICLE_API_BUS_BUS_VELOCITY_SETPOINT_SUBSCRIBER_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>

#include "argus/bus/bus.hpp"
#include "argus/bus/message.hpp"
#include "argus/bus/topic_subscriber.hpp"
#include "argus/core/subscription.hpp"
#include "argus/vehicle_api/types.hpp"
#include "argus/vehicle_api_bus/seq_tracker.hpp"

namespace argus::vehicle_api_bus {

class BusVelocitySetpointSubscriber {
public:
    using Handler = std::function<void(const vehicle_api::VelocitySetpoint&)>;

    // bus 가 구독자를 만들지 못하면 nullptr 를 반환한다 (이유는 bus 구현이 로그에 남긴다)
    static std::unique_ptr<BusVelocitySetpointSubscriber> create(bus::IBus& bus);

    // setpoint 를 받을 때마다 handler 를 부른다. 여러 번 등록하면 등록한 순서대로 모두 부른다.
    // handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서 불린다.
    // 형식이 맞지 않는 메시지(파싱 실패, 바이너리 포함)는 폐기하고 로그에 남긴다.
    // 반환된 Subscription 이 사라지면 등록이 해제된다. handler 가 비어 있으면 등록하지 않는다
    [[nodiscard]] core::Subscription subscribe(Handler handler);

    // 처음 받은 메시지 이후에 손실된 메시지 수 (seq 로 센다). 발행자가 다시 시작해 seq 가 작아진
    // 경우는 세지 않는다
    std::uint64_t lost_count() const;

private:
    // Subscription 이 이 객체보다 오래 살아도 안전하도록 handler 목록은 공유 객체에 두고,
    // Subscription 과 bus 의 handler 는 이 객체를 소유하지 않고 weak_ptr 로 참조한다
    struct State {
        std::map<std::uint64_t, Handler> handlers;  // 등록한 순서
        std::uint64_t next_id = 1;
        SeqTracker seq;
    };

    BusVelocitySetpointSubscriber(std::unique_ptr<bus::ITopicSubscriber> subscriber,
                                  std::shared_ptr<State> state, core::Subscription subscription);

    static void on_message(State& state, const bus::BusMessage& message);

    std::shared_ptr<State> state_;
    std::unique_ptr<bus::ITopicSubscriber> subscriber_;
    core::Subscription subscription_;  // bus 구독. 이 객체가 사라지면 해제된다
};

}  // namespace argus::vehicle_api_bus

#endif  // ARGUS_VEHICLE_API_BUS_BUS_VELOCITY_SETPOINT_SUBSCRIBER_HPP_
