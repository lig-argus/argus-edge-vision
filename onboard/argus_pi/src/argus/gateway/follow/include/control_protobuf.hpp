#pragma once
// 게이트웨이의 6372 받기 (2026-11-25 batch 6b, 팀 6372 전환).
// 추종 제어 (ZeroMqFollowBackend) 는 control/setpoint, follow/status 를 팀 규약 [이름][protobuf] 로
// 보낸다. 추종 송신부 (FollowBOwner) 는 지금처럼 [이름][msgpack] 을 받아 receive_setpoint,
// receive_status 의 검사를 그대로 한다. 실제 전송 (ZmqFollowTransport::command_receive) 만 이
// 코드로 본문을 to_json 값의 msgpack 바이트로 되살린다. 그래서 IFollowTransport 를 바꿔 끼우는
// stage 시험과 differential 의 C++ 쪽 msgpack 내용은 그대로다.
// - 이름은 정확히 같아야 한다. 다른 이름은 OtherName (FollowBOwner 는 건너뛴다).
// - 크기 상한 (team_wire pi_topics.hpp), 빈 본문, protobuf 해석, to_json 실패는 Invalid. 받는 쪽은
//   예전 msgpack 해석 실패처럼 last_reason invalid_setpoint_wire 로 남긴다 (송신권, 기한은 그대로).
// 이 헤더는 nlohmann 을 포함하지 않는다 (fc_protobuf.hpp 와 같은 까닭으로 변환은
// argus_gateway_control_pb 라이브러리에 둔다).
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace argus::gateway {

enum class ControlFrame : std::uint8_t {
    Ok,         // packed 에 msgpack 바이트
    OtherName,  // 이름이 control/setpoint, follow/status 가 아님 (건너뛴다)
    Invalid,    // 크기 상한, 빈 본문, protobuf 해석, to_json 실패
};

// name, body: [이름][본문] 두 프레임. error 에 Invalid 의 까닭
ControlFrame control_msgpack(const std::string& name, const std::vector<std::uint8_t>& body,
                             std::vector<std::uint8_t>& packed, std::string& error);

}  // namespace argus::gateway
