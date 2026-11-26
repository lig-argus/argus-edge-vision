// bus::IBus 의 Fast DDS 구현. 토픽은 IDL 타입 토픽, 요청-응답은 이름마다 토픽 둘로 보낸다.
// 규약: docs/spec/process_interfaces.md 3장, docs/design/uml/class_mavlink_gateway.md 1절
// 사용: Fast DDS 로 통신하는 실행 파일의 main() 이 하나를 만들어 어댑터에 넘긴다
//
// 규칙
// - DdsBus 하나가 DomainParticipant 하나를 만든다. 같은 domain_id 의 DdsBus 끼리 통신한다.
// - 값과 IDL 표본 사이의 변환은 생성할 때 받은 표(argus/bus_dds/dds_type_table.hpp)의 함수로 한다.
//   from_idl 이 실패한 표본은 폐기하고 로그에 남긴다.
// - 토픽: Reliability RELIABLE, Durability VOLATILE, History KEEP_LAST(depth = queue_depth).
//   발행자에는 생성할 때 받은 표(argus/bus_dds/dds_topic_qos.hpp)의 Deadline 을 더한다.
//   Deadline 초과와 QoS 불일치는 로그에만 남기고 사용하는 쪽에 알리지 않는다.
//   수신 두절의 판단은 받는 프로세스가 메시지 값으로 한다 (process_interfaces.md 6장).
//   구독자는 처리하지 못한 표본이 queue_depth 개를 넘으면 오래된 표본부터 폐기한다.
//   MessageInfo::sequence 는 DDS 가 발행자의 표본마다 붙이는 순번(1 부터)이다.
// - 구독자가 연결되기 전에 발행된 표본은 전달되지 않는다 (VOLATILE).
// - 요청-응답: 이름마다 토픽 <이름>_Request 와 <이름>_Reply 를 사용한다. 응답하는 쪽은 요청 표본의
//   식별자를 응답에 붙이고, 요청하는 쪽은 자기 요청의 식별자가 붙은 응답만 받는다. 요청은 응답을
//   기다리지 않고 여러 개를 보낼 수 있다. timeout 은 request() 를 부른 때부터 재고, 응답이 없으면
//   on_reply 를 nullptr 로 부른다. 요청을 변환하지 못하거나 보내지 못한 경우에도 timeout 뒤 nullptr
//   로 부른다. 응답하는 쪽은 handler 가 등록되어 있는 동안에만 응답한다.
// - Fast DDS 스레드의 수신 알림은 pipe 에 1 바이트만 기록하고, IFdWatcher 에 등록한 handler 가
//   process_events() 안에서 표본을 꺼내 전달한다 (bus contract 의 "큐를 거침").
// - Fast DDS 의 Data Sharing(발행자와 구독자가 공유 메모리의 표본을 직접 읽는 기능)은 끈다. 같은
//   컴퓨터 안의 전달은 Fast DDS 의 기본 전송(공유 메모리 전송)을 사용한다.
// - watcher, scheduler, clock 은 DdsBus 와 DdsBus 가 만든 객체보다 오래 살아야 한다.
#ifndef ARGUS_BUS_DDS_DDS_BUS_HPP_
#define ARGUS_BUS_DDS_DDS_BUS_HPP_

#include <cstdint>
#include <memory>
#include <string_view>

#include "argus/bus/bus.hpp"
#include "argus/bus/message_type.hpp"
#include "argus/bus_dds/dds_topic_qos.hpp"
#include "argus/bus_dds/dds_type_table.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/fd_watcher.hpp"
#include "argus/core/timer_scheduler.hpp"

namespace argus::bus_dds {

namespace detail {
// DdsBus 와 DdsBus 가 만든 엔티티가 공유하는 상태 (DomainParticipant 등). 구현 파일에 정의한다
struct DdsBusState;
}  // namespace detail

class DdsBus : public bus::IBus {
public:
    // topic_qos 는 토픽별 발행자 QoS. 표에 없는 토픽은 Deadline 을 두지 않는다.
    // DomainParticipant 를 만들지 못하면 nullptr 를 반환하고 이유를 std::cerr 에 남긴다
    static std::unique_ptr<DdsBus> create(core::IFdWatcher& watcher,
                                          core::ITimerScheduler& scheduler,
                                          const core::IClock& clock, std::uint32_t domain_id,
                                          DdsTypeTable types, DdsTopicQosTable topic_qos);
    ~DdsBus() override;

    // 아래 넷은 실패하면 nullptr 를 반환하고 이유를 std::cerr 에 남긴다. 실패하는 경우: 빈 이름,
    // 표에 없는 타입, 1 미만의 queue_depth, Fast DDS 의 엔티티 생성 실패
    std::unique_ptr<bus::ITopicPublisher> create_publisher(std::string_view topic,
                                                           const bus::MessageType& type,
                                                           std::int32_t queue_depth) override;
    std::unique_ptr<bus::ITopicSubscriber> create_subscriber(std::string_view topic,
                                                             const bus::MessageType& type,
                                                             std::int32_t queue_depth) override;
    std::unique_ptr<bus::IRequester> create_requester(std::string_view name,
                                                      const bus::MessageType& request,
                                                      const bus::MessageType& reply) override;
    std::unique_ptr<bus::IReplier> create_replier(std::string_view name,
                                                  const bus::MessageType& request,
                                                  const bus::MessageType& reply) override;

private:
    explicit DdsBus(std::shared_ptr<detail::DdsBusState> state);

    // 만든 엔티티도 이 상태를 공유한다. 엔티티가 DdsBus 보다 늦게 소멸해도 DomainParticipant 는
    // 마지막 엔티티가 소멸할 때 지운다
    std::shared_ptr<detail::DdsBusState> state_;
};

}  // namespace argus::bus_dds

#endif  // ARGUS_BUS_DDS_DDS_BUS_HPP_
