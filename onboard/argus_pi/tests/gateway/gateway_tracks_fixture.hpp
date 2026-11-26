#pragma once
// GatewayTracksProtobufTest 의 본문 만들기 (2026-11-25 batch 2 리뷰, 장비 없음).
// tests/team_wire/fixtures 의 Pi tracks 값(v1, v5)을 team_wire 로 묶은 protobuf 본문과, 예전 Pi
// 생산자가 보내던 msgpack 바이트를 돌려준다. 게이트웨이 번역 단위에 vendor nlohmann 경로가 섞이지
// 않도록 nlohmann 을 포함하지 않는다 (src/argus/gateway/follow/include/tracks_protobuf.hpp 와 같은
// 이유). 구현은 argus_team_wire 를 PRIVATE 로 링크한 시험 전용 라이브러리에 있다.
#include <cstdint>
#include <string>
#include <vector>

namespace gateway_tracks_fixture {

using Bytes = std::vector<std::uint8_t>;

struct PiBody {
    std::string schema;     // argus.tracking.tracks.v1 또는 v5
    Bytes body;             // [tracking/tracks][본문] 의 본문 (protobuf)
    Bytes legacy_msgpack;   // 예전 Pi 생산자가 6357 로 보내던 msgpack 바이트
    bool generation_field;  // generation 이 field 6 에 있음 (uint32 밖이면 pi_generation)
};

// fixture 의 tracks 값 전부 (팀 seq 는 1 부터)
std::vector<PiBody> pi_bodies();

// fixture 첫 값의 schema 를 비워 묶은 본문 (팀 생산자 본문과 같은 모양)
Bytes missing_pi_schema_body();

}  // namespace gateway_tracks_fixture
