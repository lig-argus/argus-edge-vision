// 게이트웨이 tracking/tracks 받기 시험 (2026-11-25 batch 2 리뷰, 장비와 소켓 없음).
// follow_owner (cv_gap 관측) 와 main_gcs_bridge (메인 GUI 연결부) 가 쓰는 tracks_msgpack 를 실제
// team_wire 본문으로 확인한다.
// - Ok: Pi tracks fixture (v1, v5) 본문 -> 예전 Pi 생산자 msgpack 과 같은 바이트, 중복 키 거부
//   msgpack_unpack(packed, true) 통과, schema 보존.
// - NotTracks: 프레임 수가 2 가 아님, 이름이 tracking/tracks 와 정확히 같지 않음.
// - MissingPiSchema: pi_schema 없는 팀 생산자 본문 (받는 쪽은 프레임마다 오류로 덮어쓰지 않는다).
// - Invalid: 빈 본문, 1 MiB 초과, 해석 실패, 같은 필드가 두 번 온 본문.
#include <cstdint>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/tracks_protobuf.hpp"
#include "tests/gateway/gateway_tracks_fixture.hpp"

namespace gw = argus::gateway;
using gateway_tracks_fixture::Bytes;

namespace {

int g_checks = 0;

void check(bool condition, const std::string& what) {
    ++g_checks;
    if (!condition) {
        throw std::runtime_error("실패: " + what);
    }
}

Bytes text(const std::string& value) { return {value.begin(), value.end()}; }

std::vector<Bytes> frame(const Bytes& body, const std::string& name = "tracking/tracks") {
    return {text(name), body};
}

gw::TracksFrame convert(const std::vector<Bytes>& parts, Bytes& packed, std::string& error) {
    return gw::tracks_msgpack(parts, packed, error);
}

void pi_bodies() {
    const auto bodies = gateway_tracks_fixture::pi_bodies();
    check(!bodies.empty(), "fixture");
    bool v1 = false;
    bool v5 = false;
    for (const auto& entry : bodies) {
        Bytes packed;
        std::string error;
        check(convert(frame(entry.body), packed, error) == gw::TracksFrame::Ok, "Ok " + error);
        check(error.empty(), "Ok 는 오류 없음");
        check(packed == entry.legacy_msgpack, "예전 Pi 생산자 msgpack 바이트 " + entry.schema);
        const gw::Wire value = gw::msgpack_unpack(packed, true);
        check(value.at("schema").get<std::string>() == entry.schema, "schema 보존");
        check(value.at("tracks").is_array(), "tracks 목록");
        v1 = v1 || entry.schema == "argus.tracking.tracks.v1";
        v5 = v5 || entry.schema == "argus.tracking.tracks.v5";
    }
    check(v1 && v5, "v1 과 v5");
}

void not_tracks() {
    const Bytes body = gateway_tracks_fixture::pi_bodies().front().body;
    Bytes packed{1};
    std::string error = "x";
    check(convert({text("tracking/tracks")}, packed, error) == gw::TracksFrame::NotTracks,
          "프레임 1개");
    check(packed.empty() && error.empty(), "NotTracks 는 비운다");
    check(
        convert({text("tracking/tracks"), body, body}, packed, error) == gw::TracksFrame::NotTracks,
        "프레임 3개");
    for (const char* name : {"tracking/tracksX", "tracking/track", "tracking/status", ""}) {
        check(convert(frame(body, name), packed, error) == gw::TracksFrame::NotTracks,
              std::string("이름 정확 일치: ") + name);
    }
}

void rejected(const Bytes& body, gw::TracksFrame expected, const std::string& what,
              const std::string& reason) {
    Bytes packed;
    std::string error;
    check(convert(frame(body), packed, error) == expected, what);
    check(packed.empty(), what + ": packed 비움");
    check(error.find(reason) != std::string::npos, what + ": 진단 " + error);
}

void invalid(const Bytes& body, const std::string& what, const std::string& reason) {
    rejected(body, gw::TracksFrame::Invalid, what, reason);
}

void invalid_bodies() {
    rejected(gateway_tracks_fixture::missing_pi_schema_body(), gw::TracksFrame::MissingPiSchema,
             "pi_schema 없음", "pi_schema");
    invalid({}, "빈 본문", "빈 본문");
    invalid(Bytes((1U << 20U) + 1U, 0), "1 MiB 초과", "크기 상한");
    invalid({0xff, 0xff, 0xff, 0xff}, "해석 실패", "해석 실패");
    for (const auto& entry : gateway_tracks_fixture::pi_bodies()) {
        // 같은 필드를 한 번 더: protobuf 는 마지막 값을 쓴다. 예전 msgpack 경로는 중복 키를
        // duplicate_tracking_field 로 거부했다
        Bytes pi_schema = entry.body;
        pi_schema.insert(pi_schema.end(),
                         {0xa2, 0x06, static_cast<std::uint8_t>(entry.schema.size())});
        pi_schema.insert(pi_schema.end(), entry.schema.begin(), entry.schema.end());
        invalid(pi_schema, "pi_schema 중복 " + entry.schema, "중복 필드");
        if (entry.generation_field) {
            Bytes generation = entry.body;
            generation.insert(generation.end(), {0x30, 0x07});  // field 6 (generation) = 7
            invalid(generation, "generation 중복 " + entry.schema, "중복 필드");
        }
    }
}

}  // namespace

int main() {
    try {
        pi_bodies();
        not_tracks();
        invalid_bodies();
        std::printf("GatewayTracksProtobufTest: %d checks ok\n", g_checks);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "GatewayTracksProtobufTest: %s\n", error.what());
        return 1;
    }
}
