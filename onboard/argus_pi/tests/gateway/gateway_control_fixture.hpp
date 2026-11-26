#pragma once
// GatewayControlProtobufTest 의 본문 만들기 (2026-11-25 batch 6b, 장비 없음).
// 게이트웨이 번역 단위에 vendor nlohmann 경로가 섞이지 않도록 nlohmann 을 포함하지 않는다
// (gateway_fc_fixture.hpp 와 같은 이유). 본문 만들기는 team_wire 와 생성 protobuf 그대로다.
// 2026-11-25 batch 7: GatewayModeRpcProtobufTest, GatewayStage4Test 의 6371 요청과 응답 본문도
// 둔다.
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gateway_control_fixture {

using Bytes = std::vector<std::uint8_t>;

// follow/status 본문의 command_results 를 마지막 행을 되풀이해 rows 행으로 늘린다 (생산자 to_proto
// 는 32 행까지만 만든다. 팀 생산자나 손으로 만든 본문처럼 32 행을 넘는 본문을 만든다)
Bytes status_with_rows(const Bytes& body, std::size_t rows);

// 추종 제어 ZeroMqFollowBackend 모양의 6371 요청 msgpack (envelope_locked 의 키 전부, 첫 acquire
// 처럼 토큰과 문맥은 빈 값). mode 이면 모드 요청 (schema argus.control.mode_request.v1, mode,
// valid_until_monotonic_ns), 아니면 제어권 RPC (argus.follow.rpc.v1)
Bytes mode_rpc_request(const std::string& action, bool mode, std::uint64_t sequence,
                       const std::string& onboard_session);
// 6371 요청 msgpack -> ControlModeRequest 본문 (실패하거나 proto 에 없는 키가 있으면 빈 본문)
Bytes encode_mode_request(const Bytes& packed);
// ControlModeReply 본문 -> to_json 값의 msgpack. 실패 (빈 본문 포함) 하면 false 와 error
bool decode_mode_reply(const Bytes& body, Bytes& packed, std::string& error);

// 2026-11-25 batch 8: 6373 (GatewayFollowCommandProtobufTest). FollowCommand 본문 -> to_json 값의
// msgpack (실패하면 false 와 error), 응답 msgpack -> FollowResult 본문 (실패하거나 proto 에 없는
// 키가 있으면 빈 본문)
bool decode_follow_command(const Bytes& body, Bytes& packed, std::string& error);
Bytes encode_follow_result(const Bytes& packed);

}  // namespace gateway_control_fixture
