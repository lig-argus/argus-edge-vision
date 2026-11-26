#pragma once
// GatewayFcProtobufTest 의 본문 해석과 만들기 (2026-11-25 batch 6a, 장비 없음).
// 게이트웨이 번역 단위에 vendor nlohmann 경로가 섞이지 않도록 nlohmann 을 포함하지 않는다
// (gateway_tracks_fixture.hpp, gateway_vision_fixture.hpp 와 같은 이유). 변환은 team_wire 그대로다.
#include <cstdint>
#include <string>
#include <vector>

namespace gateway_fc_fixture {

using Bytes = std::vector<std::uint8_t>;

Bytes read_file(const std::string& path);
// [이름][protobuf] 본문 -> to_json -> msgpack 바이트. team_seq 에 팀 seq. 실패하면 false 와 error
bool decode(const std::string& topic, const Bytes& body, Bytes& packed, std::uint64_t& team_seq,
            std::string& error);
// 두 msgpack 이 같은 값인지 (키, null, 정수와 실수 구분까지). 같으면 빈 문자열, 다르면 두 값
std::string msgpack_difference(const Bytes& expected, const Bytes& actual);
// Pi 모양 msgpack -> 팀 본문 (팀 seq 1). 실패하면 빈 본문
Bytes encode(const std::string& topic, const Bytes& packed);
// fc/state 본문에서 status 를 지운 본문 (팀 필드만 쓰는 생산자처럼 armed, in_air 가 없다)
Bytes without_status(const Bytes& body);
// msgpack 의 키 하나를 바꾼 msgpack (value_msgpack 은 새 값 하나의 msgpack)
Bytes with_key(const Bytes& packed, const std::string& key, const Bytes& value_msgpack);

}  // namespace gateway_fc_fixture
