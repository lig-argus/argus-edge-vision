// MAV_CMD_REQUEST_MESSAGE (512) 의 처리: 요청한 메시지 번호(param1)의 메시지를 만들어 보낸다
// 규약: docs/spec/icd.md 4.1, docs/spec/video.md 7장,
//      docs/design/uml/class_mavlink_gateway.md 3절
// 사용: mavlink_gateway 의 main 이 생성하여 메시지 번호별 만드는 함수를 등록하고
//      MavlinkCommandReceiver 에 kRules 로 등록
//
// 규칙
// - param1 이 0~16,777,215 의 정수가 아니거나 등록되지 않은 메시지 번호면 DENIED
// - 만드는 함수의 결과를 COMMAND_ACK 로 먼저 보내고, ACCEPTED 이면 만든 메시지를 요청한 통제기에만
//   보낸다. param7(응답 대상)은 보지 않는다: 메시지 안의 값(예: RTSP 주소의 호스트)이 요청한
//   통제기에 맞춰 만들어지기 때문이다
#ifndef ARGUS_ONBOARD_REQUEST_MESSAGE_HANDLER_HPP_
#define ARGUS_ONBOARD_REQUEST_MESSAGE_HANDLER_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/gcs_link.hpp"
#include "argus/mavlink_gateway/mavlink_command_handler.hpp"
#include "argus/mavlink_gateway/mavlink_command_receiver.hpp"

namespace argus::mavlink_gateway {

// 요청한 메시지를 만든 결과
struct RequestedMessages {
    MAV_RESULT result = MAV_RESULT_FAILED;    // ACCEPTED 이면 messages 를 보낸다
    std::string reason;                       // DENIED, FAILED 의 원인 (STATUSTEXT)
    std::vector<mavlink_message_t> messages;  // 보낼 순서대로
};

class RequestMessageHandler : public IMavlinkCommandHandler {
public:
    // param7 이 응답 대상이므로 명령 번호 규칙을 적용하지 않는다. 제어권이 없는 통제기도, 착륙
    // 잠금이나 조종사 개입 중에도 요청할 수 있다: 요청한 메시지를 알려 줄 뿐 기체를 움직이지 않는다
    static constexpr CommandRules kRules{false, false, CommandKind::Other, true};

    // 요청 하나에 대해 보낼 메시지를 만드는 함수
    using MessageBuilder = std::function<RequestedMessages(const CommandRequest&)>;

    // link 는 이 객체보다 오래 유지되어야 한다
    explicit RequestMessageHandler(IGcsLink& link) : link_(link) {}

    // message_id 의 메시지를 만드는 함수를 등록한다. 이미 등록된 번호이거나 builder 가 비어 있으면
    // 등록하지 않고 false
    bool add_message(std::uint32_t message_id, MessageBuilder builder);

    std::uint16_t command() const override { return MAV_CMD_REQUEST_MESSAGE; }
    void handle(const CommandRequest& request, CommandResponder responder) override;

private:
    IGcsLink& link_;
    std::map<std::uint32_t, MessageBuilder> builders_;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_REQUEST_MESSAGE_HANDLER_HPP_
