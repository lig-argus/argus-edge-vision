// 프로세스가 붙는 메시지 통로. 토픽 발행자, 구독자, 요청하는 쪽, 응답하는 쪽을 만든다.
// 통신 구현(ZeroMQ, Fast DDS 등)을 교체해도 이 인터페이스와 넷은 바뀌지 않는다. 구현별 설정(주소,
// QoS 프로파일)은 구현 클래스의 생성자 인자이고, 이 인터페이스에는 두지 않는다.
// 규약: docs/spec/process_interfaces.md 3장. 설계: docs/design/uml/class_onboard.md 1절
// 구현: (#59) ZmqBus, 테스트용 FakeBus (bus_testing)
//
// contract (모든 구현이 따라야 한다)
// - handler 는 core::IPoller::process_events() 안에서, 그 함수를 부른 스레드에서만 불린다.
//   라이브러리 스레드에서 콜백을 주는 구현은 큐에 넣어 두었다가 process_events() 에서 비운다.
// - 소멸자가 닫는다. 아직 보내지 못한 메시지는 폐기한다.
// - 큐 길이를 연결 단위로만 설정할 수 있는 구현은 만든 엔티티들의 queue_depth 최댓값을 적용한다.
// - 요청-응답 기능이 없는 구현은 토픽 둘과 상관 번호로 요청-응답을 만든다.
// - 토픽 이름은 구현이 그 통신 라이브러리가 허용하는 형식으로 변환할 수 있다. 이름 규칙은
//   argus/protocol/topics.hpp
#ifndef ARGUS_BUS_BUS_HPP_
#define ARGUS_BUS_BUS_HPP_

#include <memory>
#include <string_view>

#include "argus/bus/replier.hpp"
#include "argus/bus/requester.hpp"
#include "argus/bus/topic_publisher.hpp"
#include "argus/bus/topic_subscriber.hpp"

namespace argus::bus {

class IBus {
public:
    IBus() = default;
    virtual ~IBus() = default;
    IBus(const IBus&) = delete;
    IBus& operator=(const IBus&) = delete;
    IBus(IBus&&) = delete;
    IBus& operator=(IBus&&) = delete;

    // 토픽마다 하나 만든다. queue_depth 는 보관할 최대 메시지 수(1 이상. 기본값은
    // argus/protocol/topics.hpp). 실패하면 nullptr 를 반환한다. 실패 이유는 구현이 로그에 남긴다.
    virtual std::unique_ptr<ITopicPublisher> create_publisher(std::string_view topic,
                                                              int queue_depth) = 0;
    virtual std::unique_ptr<ITopicSubscriber> create_subscriber(std::string_view topic,
                                                                int queue_depth) = 0;

    // 요청-응답 이름마다 하나 만든다. 실패 규칙은 위와 같다.
    virtual std::unique_ptr<IRequester> create_requester(std::string_view name) = 0;
    virtual std::unique_ptr<IReplier> create_replier(std::string_view name) = 0;
};

}  // namespace argus::bus

#endif  // ARGUS_BUS_BUS_HPP_
