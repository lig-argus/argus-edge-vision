// bus::IBus 의 ZeroMQ 구현. 토픽은 PUB/SUB, 요청-응답은 REQ/REP 소켓으로 보낸다.
// 규약: docs/spec/process_interfaces.md 3장 (multipart 구성 [0] 이름, [1] 본문, 필요하면
// [2] 바이너리), docs/design/uml/class_onboard.md 1절
// 사용: 실행 파일의 main() 이 하나를 만들어 어댑터에 넘긴다
//
// 규칙
// - 값과 전송 바이트 사이의 변환은 생성자에 받은 변환 표(argus/bus_zmq/zmq_codec.hpp)의 함수로
//   한다. 발행자는 발행마다 1 씩 올린 번호(1 부터)를 encode 에 넘기고, 보내지 못한 발행도 번호를
//   쓴다. 구독자는 decode 가 돌려준 번호를 bus::MessageInfo::sequence 로 handler 에 전달한다.
// - decode 가 실패한 메시지는 폐기하고 로그에 남긴다. 요청을 decode 하지 못하면 handler 를 부르지
//   않고 빈 본문으로 응답한다. 응답을 decode 하지 못하면 응답이 오지 않은 것과 같이 on_reply 를
//   nullptr 로 부른다. 요청을 encode 하지 못하면 보내지 않고 timeout 뒤 on_reply 를 nullptr 로
//   부른다. 요청과 응답에는 바이너리가 없으며, encode 가 바이너리를 채우면 실패로 다룬다.
// - 이름(토픽 이름 또는 요청-응답 이름)마다 엔드포인트 하나를 주소표로 정한다. 발행자와 응답하는
//   쪽은 bind 하고, 구독자와 요청하는 쪽은 connect 한다.
// - 한 엔드포인트를 여러 토픽의 발행자가 함께 쓸 수 있다 (PUB 소켓 하나를 공유). 이때 큐 길이는
//   그 발행자들의 queue_depth 최댓값이며, 큐 길이를 늘린 뒤에 연결된 구독자부터 적용된다.
// - 큐는 구독자마다 따로 있다. 느린 구독자의 큐가 가득 차면 그 구독자에게 갈 메시지만 버려지고
//   다른 구독자는 영향을 받지 않는다. 이 경우에도 publish() 는 true 를 반환한다 (ZeroMQ 는 어느
//   구독자의 큐가 찼는지 알려 주지 않는다). false 는 소켓 오류일 때만 반환한다.
// - 구독자는 처리하지 못한 메시지가 queue_depth 개를 넘는 경우 가장 오래된 메시지부터 폐기하고 최근
//   queue_depth 개를 받은 순서대로 전달한다. 단, 발행 쪽 큐가 가득 찬 동안 발행된 메시지는 구독자에
//   도착하지 않으므로, 오래 멈췄던 구독자가 받는 메시지는 멈춘 시점의 메시지일 수 있다 (큰
//   메시지에서 발생). 받는 쪽은 메시지의 시각으로 오래된 메시지를 판단한다.
// - 구독자의 연결이 끝나기 전에 발행된 메시지는 전달되지 않는다 (발행자 또는 구독자가 시작한 직후의
//   메시지). 한 번만 발행하는 값은 이 구간에서 손실될 수 있다.
// - 요청은 한 번에 하나만 보낸다. 앞 요청의 응답이나 timeout 뒤에 다음 요청을 보낸다. timeout 은
//   request() 를 부른 때부터 잰다.
// - 응답하는 쪽은 handler 가 등록되어 있는 동안에만 요청을 읽는다. 등록 전이나 해제 뒤에 온
//   요청에는 응답하지 않는다 (요청한 쪽은 timeout 으로 안다).
// - context, poller, clock 은 ZmqBus 와 ZmqBus 가 만든 객체보다 오래 살아야 한다.
#ifndef ARGUS_BUS_ZMQ_ZMQ_BUS_HPP_
#define ARGUS_BUS_ZMQ_ZMQ_BUS_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <zmq.hpp>

#include "argus/bus/bus.hpp"
#include "argus/bus/message_type.hpp"
#include "argus/bus_zmq/zmq_codec.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/clock.hpp"

namespace argus::bus_zmq {

// 이름(토픽 이름 또는 요청-응답 이름) → ZeroMQ 엔드포인트 (예: "tcp://127.0.0.1:6370")
using ZmqEndpointTable = std::map<std::string, std::string, std::less<>>;

class ZmqBus : public bus::IBus {
public:
    ZmqBus(zmq::context_t& context, ZmqPoller& poller, const core::IClock& clock,
           ZmqEndpointTable endpoints, ZmqCodecTable codecs);
    ~ZmqBus() override;

    // 아래 넷은 실패하면 nullptr 를 반환하고 이유를 std::cerr 에 남긴다. 실패하는 경우: 빈 이름,
    // 주소표에 없는 이름, 변환 표에 없는 타입, 1 미만의 queue_depth, bind 또는 connect 실패
    std::unique_ptr<bus::ITopicPublisher> create_publisher(std::string_view topic,
                                                           const bus::MessageType& type,
                                                           int queue_depth) override;
    std::unique_ptr<bus::ITopicSubscriber> create_subscriber(std::string_view topic,
                                                             const bus::MessageType& type,
                                                             int queue_depth) override;
    std::unique_ptr<bus::IRequester> create_requester(std::string_view name,
                                                      const bus::MessageType& request,
                                                      const bus::MessageType& reply) override;
    std::unique_ptr<bus::IReplier> create_replier(std::string_view name,
                                                  const bus::MessageType& request,
                                                  const bus::MessageType& reply) override;

private:
    struct Shared;
    std::shared_ptr<Shared> shared_;
};

}  // namespace argus::bus_zmq

#endif  // ARGUS_BUS_ZMQ_ZMQ_BUS_HPP_
