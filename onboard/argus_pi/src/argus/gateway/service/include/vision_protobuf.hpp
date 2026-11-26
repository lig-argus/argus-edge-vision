#pragma once
// 게이트웨이의 영상 6374 받기 (2026-11-25 batch 4, 팀 6374 전환).
// argus-onboard 는 팀 규약 [이름][protobuf][binary] 로 보낸다. 이 함수는 본문을 Pi 생산자가 예전에
// 보내던 msgpack 바이트로 되살린다 (src/argus/transport/team_wire/include/vision_body.hpp). 받는
// 쪽은 그 바이트를 예전처럼 msgpack_unpack 으로 읽고 같은 판정(preview_ready.hpp)을 거친다. 이
// 헤더는 nlohmann 을 포함하지 않는다: 게이트웨이(시스템 nlohmann)와 변환 코드(vendor/vmy) 의 JSON
// 헤더 경로를 한 번역 단위에서 섞지 않으려고 변환은 argus_gateway_vision_pb 라이브러리에 둔다
// (tracks_protobuf.hpp 와 같은 방식).
#include <cstdint>
#include <string>
#include <vector>

namespace argus::gateway {

enum class VisionFrame : std::uint8_t {
    Ok,         // packed 에 msgpack 바이트
    NotVision,  // 이름이 6374 토픽과 정확히 같지 않거나 프레임 수가 규약과 다름 (건너뛴다)
    // 본문을 되살리지 못함 (크기 상한, protobuf 해석, 중복 필드, 빈 바이너리. 이유는 error)
    Invalid,
    // pi_schema 없는 팀 생산자 본문 (받지 않음). 수와 1, 2, 4 ... 번째 로그는 team_wire 가 남긴다
    MissingPiSchema,
};

// parts: 받은 multipart. 영상 4토픽은 [이름][본문][binary] (binary 는 비어 있지 않다), 관측과
// 진단은 [이름][본문]. 이름은 parts[0] 그대로 쓴다
VisionFrame vision_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                           std::vector<std::uint8_t>& packed, std::string& error);

}  // namespace argus::gateway
