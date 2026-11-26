#pragma once
// 게이트웨이의 6373 운용자 요청-응답 (2026-11-25 batch 8, 팀 6373 전환).
// FollowProxy 는 추종 제어 (VMyOperatorGateway) 에 팀 규약 [follow/command][FollowCommand] 를
// 보내고 [follow/command][FollowResult] 를 받는다 (응답 첫 프레임은 요청 이름, 팀 ZmqReplier 와
// 같다). FollowProxy 의 요청 만들기와 응답 검사는 지금처럼 msgpack 값 (Wire) 으로 한다. 이 코드는
// 본문만 바꾼다 (Wire -> msgpack -> NativeJson -> protobuf, 응답은 거꾸로).
// - 요청: msgpack 값 -> to_proto (FollowCommand). 변환하지 못하면 false (보내지 않는다).
// - 응답: 프레임 둘, 이름 정확 일치 (예전 이름 follow/result 는 받지 않음), 본문 상한
//   (team_wire kMaxRpcBytes 256 KiB), 빈 본문 (추종 제어가 해석하지 못한 요청의 응답) 거부,
//   protobuf 해석과 to_json.
// 이 헤더는 nlohmann 을 포함하지 않는다 (mode_rpc_protobuf.hpp 와 같은 까닭으로 변환은
// argus_gateway_follow_command_pb 라이브러리에 둔다).
#include <cstdint>
#include <string>
#include <vector>

namespace argus::gateway {

enum class FollowReplyFrame : std::uint8_t {
    Ok,            // packed 에 응답의 msgpack 바이트
    BadShape,      // 프레임 수 또는 이름이 다름
    DecodeFailed,  // 본문 상한, 빈 본문, protobuf 해석, to_json 실패
};

// 6373 의 요청, 응답 이름 (팀 argus::protocol::kFollowCommand)
const std::string& follow_command_name();

// 요청 msgpack -> FollowCommand 본문. false 면 body 는 비고 error 에 까닭. unknown 에 proto 에
// 없어 버린 키
bool follow_command_request_body(const std::vector<std::uint8_t>& packed,
                                 std::vector<std::uint8_t>& body, std::vector<std::string>& unknown,
                                 std::string& error);

// parts: REQ 로 받은 응답 메시지 하나. error 에 Ok 가 아닌 까닭
FollowReplyFrame follow_command_reply_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                                              std::vector<std::uint8_t>& packed,
                                              std::string& error);

}  // namespace argus::gateway
