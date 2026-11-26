#pragma once
// 게이트웨이의 6371 요청-응답 (2026-11-25 batch 7, 팀 6371 전환).
// 추종 제어 (ZeroMqFollowBackend) 는 모드 요청과 제어권 RPC 를 모두 팀 규약
// [control/mode_request][ControlModeRequest] 로 보내고 [control/mode_request][ControlModeReply] 를
// 받는다 (응답 첫 프레임은 요청 이름, 팀 ZmqReplier 와 같다). 추종 송신부 (FollowBOwner) 는
// 지금처럼 msgpack 값으로 request, mode_request 를 부른다. 실제 전송 (ZmqFollowTransport 의
// rpc_receive, rpc_send) 만 이 코드로 본문을 바꾼다.
// - 요청: 프레임 둘, 이름 정확 일치, 본문 상한 (team_wire kMaxRpcBytes 256 KiB), 빈 본문 거부,
//   protobuf 해석과 to_json. 실패하면 Invalid 이고 전송이 [control/mode_request][빈 본문] 으로
//   답한다 (팀 replier 의 처리 실패 응답과 같은 모양. REP 상태를 지킨다).
// - 응답: msgpack 값 -> to_proto (ControlModeReply). 변환하지 못하면 false 이고 전송이 빈 본문으로
//   답한다.
// 이 헤더는 nlohmann 을 포함하지 않는다 (control_protobuf.hpp 와 같은 까닭으로 변환은
// argus_gateway_mode_rpc_pb 라이브러리에 둔다).
#include <cstdint>
#include <string>
#include <vector>

namespace argus::gateway {

enum class ModeRpcFrame : std::uint8_t {
    Ok,       // packed 에 요청의 msgpack 바이트
    Invalid,  // 프레임 수, 이름, 본문 상한, 빈 본문, protobuf 해석, to_json 실패
};

// 6371 의 요청, 응답 이름 (팀 argus::protocol::kControlModeRequest)
const std::string& mode_rpc_name();

// parts: REP 로 받은 메시지 하나. error 에 Invalid 의 까닭
ModeRpcFrame mode_rpc_request_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                                      std::vector<std::uint8_t>& packed, std::string& error);

// 응답 msgpack -> ControlModeReply 본문. false 면 body 는 비고 error 에 까닭. unknown 에 proto 에
// 없어 버린 키
bool mode_rpc_reply_body(const std::vector<std::uint8_t>& packed, std::vector<std::uint8_t>& body,
                         std::vector<std::string>& unknown, std::string& error);

}  // namespace argus::gateway
