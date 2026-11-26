#pragma once
// 게이트웨이의 tracking/tracks 받기 (2026-11-25 batch 2, 팀 6375 전환).
// 추적기는 팀 규약 [tracking/tracks][protobuf] 로 보낸다. 이 함수는 본문을 Pi 추적기가 예전에
// 보내던 msgpack 바이트로 되살린다 (src/argus/transport/team_wire/include/tracks_body.hpp). 받는
// 쪽은 그 바이트를 예전처럼 msgpack_unpack 으로 읽고 같은 검사(cv_observation,
// MainGcsBridge::tracks)를 거친다. 이 헤더는 nlohmann 을 포함하지 않는다: 게이트웨이(시스템
// nlohmann)와 변환 코드(vendor/vmy) 의 JSON 헤더 경로를 한 번역 단위에서 섞지 않으려고 변환은
// argus_gateway_tracks_pb 라이브러리에 둔다.
#include <cstdint>
#include <string>
#include <vector>

namespace argus::gateway {

enum class TracksFrame : std::uint8_t {
    Ok,         // packed 에 msgpack 바이트
    NotTracks,  // [이름][본문] 이 아니거나 이름이 tracking/tracks 와 정확히 같지 않음 (건너뛴다)
    Invalid,    // 본문을 되살리지 못함 (크기 상한, protobuf 해석, 중복 필드 등. 이유는 error)
    // pi_schema 없는 팀 생산자 본문 (받지 않음). 수는 team_wire::missing_pi_schema_rejections 가
    // 세고 1, 2, 4 ... 번째마다 std::cerr 에 남기므로, 받는 쪽은 프레임마다 오류로 덮어쓰지 않는다
    MissingPiSchema,
};

TracksFrame tracks_msgpack(const std::vector<std::vector<std::uint8_t>>& parts,
                           std::vector<std::uint8_t>& packed, std::string& error);

}  // namespace argus::gateway
