// 2026-11-09 열 외형 같은 대상 확인 시험. 합성 RAW14 프레임과 합성 트랙만 쓴다 (카메라·FC 없음).
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/control/target/include/target_manager.hpp"
#include "argus/perception/observations/include/observation_v2_codec.hpp"
#include "argus/perception/thermal/include/thermal_measure.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"
#include "argus/tracking/contracts/include/tracks_v1_codec.hpp"

using namespace follow;
using namespace std::chrono_literals;

namespace {
constexpr auto kMaxAge = 250ms;
constexpr std::int64_t kOriginNs = 1000000000LL;

Clock::time_point at(double seconds) {
    return Clock::time_point{std::chrono::nanoseconds{kOriginNs + std::llround(seconds * 1e9)}};
}

// 640x480 RAW14: 배경 3000, 박스 안은 위쪽 절반 hot, 아래쪽 절반 warm
std::vector<std::uint8_t> raw_frame(int hot, int warm) {
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(640) * 480 * 2);
    for (int y = 0; y < 480; ++y) {
        for (int x = 0; x < 640; ++x) {
            int value = 3000 + (((x * 7) + (y * 3)) % 5);
            if (x >= 200 && x < 300 && y >= 100 && y < 160) {
                value = y < 130 ? hot : warm;
            }
            const std::size_t at =
                (static_cast<std::size_t>(y) * 1280) + (static_cast<std::size_t>(x) * 2);
            pixels.at(at) = static_cast<std::uint8_t>(value & 0xFF);
            pixels.at(at + 1) = static_cast<std::uint8_t>(value >> 8);
        }
    }
    return pixels;
}

ThermalSignature signature(int hot, int warm) {
    const auto pixels = raw_frame(hot, warm);
    const auto measured =
        argus::measure_thermal_signature(pixels, 640, 480, 1280, 200, 100, 300, 160);
    assert(measured && valid_thermal_signature(*measured));
    return *measured;
}

TrackRecordV1 tank(double t, const std::optional<ThermalSignature>& thermal) {
    TrackRecordV1 track;
    track.track_id = 38;
    track.class_id = 0;
    track.label = "tank";
    track.confidence = .93;
    track.x1 = 596 + (t * 10);
    track.x2 = 834 + (t * 10);
    track.y1 = 155;
    track.y2 = 286;
    track.state = "CONFIRMED";
    track.real_detection = true;
    track.last_real_detection_monotonic_ns = kOriginNs + std::llround(t * 1e9);
    track.ground = std::array<double, 6>{8 + (.6 * t), 0, .2, 0, .2, 0};
    track.ground_anchor = std::array<double, 2>{1, 0};
    track.thermal = thermal;
    return track;
}

TracksV1 frame(unsigned number, double t, std::vector<TrackRecordV1> tracks) {
    TracksV1 message;
    message.schema = std::string{static_cast<const char*>(kTracksV5Schema)};
    message.source = "thermal-identity-test";
    message.frame_id = number;
    message.generation = 1;
    message.sample_monotonic_ns = kOriginNs + std::llround(t * 1e9);
    message.clock_domain = "host_monotonic";
    message.timestamp_source = "original-ground-sample";
    message.image_width = 1204;
    message.image_height = 718;
    message.motion_sample_ns = message.sample_monotonic_ns;
    message.ground_pose = std::array<double, 6>{0, 0, -1.5, 0, 0, 0};
    message.tracks = std::move(tracks);
    return message;
}

TargetRecoveryConfig config(ThermalIdentityMode mode, const std::string& log = "") {
    TargetRecoveryConfig value;
    value.enabled = true;
    value.max_gap = 4000ms;
    value.prefer_selected_continuity = true;
    value.trust_selected_id = true;
    value.thermal_identity = mode;
    value.thermal_identity_log = log;
    return value;
}

// 옆 탱크(ID 23): 같은 종류, 화면 아래쪽, 지면 동쪽 0.8m
TrackRecordV1 neighbor(double t, const std::optional<ThermalSignature>& thermal) {
    auto track = tank(t, thermal);
    track.track_id = 23;
    track.y1 = 400;
    track.y2 = 520;
    track.ground = std::array<double, 6>{8 + (.6 * t), .8, .2, 0, .2, 0};
    return track;
}

// 0~0.8초: 선택 대상(own)과 옆 탱크(other)가 함께 보이는 상태로 잠근다
TargetManager locked(ThermalIdentityMode mode, const ThermalSignature& own,
                     const ThermalSignature& other, const std::string& log = "") {
    TargetManager manager(config(mode, log));
    for (int i = 0; i <= 16; ++i) {
        const double t = i * .05;
        auto f = frame(i + 1, t, {tank(t, own), neighbor(t, other)});
        if (i == 0) {
            manager.select(f, f.tracks.at(0), at(t));
        } else {
            manager.observe(f, at(t), kMaxAge);
        }
        assert(manager.snapshot(at(t), kMaxAge).phase == TargetPhase::Locked);
    }
    return manager;
}

void measurement_test() {
    const auto hot = signature(3800, 3400);
    assert(std::abs(hot.at(0) - 3002) < 3);  // 주변 배경
    assert(std::abs(hot.at(1) - 800) < 5);   // 대비 95%
    assert(std::abs(hot.at(2) - 800) < 5);   // 뜨거운 절반 평균
    const auto same = signature(3790, 3395);
    const auto cool = signature(3150, 3100);
    const auto near = compare_thermal(hot, same);
    const auto far = compare_thermal(hot, cool);
    assert(near.contrast_ratio > .95 && near.histogram > .9);
    assert(far.contrast_ratio < .25);
    const auto pixels = raw_frame(3800, 3400);
    assert(!argus::measure_thermal_signature(pixels, 640, 480, 1280, 10, 10, 12, 12));  // 너무 작음
    // 배경만 있는 박스는 대비가 판단 최소값(기본 40)보다 훨씬 작다
    const auto flat = argus::measure_thermal_signature(pixels, 640, 480, 1280, 400, 300, 450, 340);
    assert(!flat || flat->at(1) < 10);
    std::cout << "PASS RAW14 thermal measurement" << '\n';
}

void codec_test() {
    const auto hot = signature(3800, 3400);
    auto message = frame(1, 0, {tank(0, hot)});
    const auto decoded = TracksV1Codec::decode(TracksV1Codec::encode(message));
    assert(decoded.valid && decoded.message.tracks.size() == 1);
    const auto& thermal = decoded.message.tracks.at(0).thermal;
    assert(thermal.has_value() && thermal.value_or(ThermalSignature{}) == hot);
    message.tracks.at(0).thermal.reset();
    const auto plain = TracksV1Codec::decode(TracksV1Codec::encode(message));
    assert(plain.valid && !plain.message.tracks.at(0).thermal);
    std::cout << "PASS tracks v5 thermal round trip" << '\n';
}

// 감지 메시지에 "thermal"이 붙어도 기존 observation v2 수신은 그대로 읽는다 (다른 필드는 거부)
void observation_codec_test() {
    ObservationV2 message;
    message.source = "thermal-identity-test";
    message.frame_id = 7;
    message.seq = 7;
    message.captured_unix_ns = 1794228640663173124LL;
    message.captured_monotonic_ns = kOriginNs;
    message.published_monotonic_ns = kOriginNs + 30000000;
    message.received_unix_ns = message.captured_unix_ns;
    message.received_monotonic_ns = kOriginNs;
    message.image_width = 640;
    message.image_height = 480;
    message.model = "test";
    message.coordinate_frame = "camera_pixel";
    message.observation_kind = "detections";
    message.clock_domain = "host_monotonic";
    message.timestamp_source = "host_receive";
    ObservationDetectionV2 detection;
    detection.class_id = 0;
    detection.label = "tank";
    detection.confidence = .9;
    detection.x1 = 200;
    detection.y1 = 100;
    detection.x2 = 300;
    detection.y2 = 160;
    message.detections.push_back(detection);
    const std::string packed = ObservationV2Codec::encode(message);
    const auto baseline = ObservationV2Codec::decode(packed);
    if (!baseline.valid) {
        std::cerr << "baseline observation invalid: " << baseline.error << '\n';
    }
    assert(baseline.valid);
    auto json = NativeJson::from_msgpack(packed.begin(), packed.end());
    const auto hot = signature(3800, 3400);
    json.at("detections").at(0).emplace("thermal", std::vector<double>(hot.begin(), hot.end()));
    const auto with_thermal = NativeJson::to_msgpack(json);
    const auto decoded =
        ObservationV2Codec::decode(std::string(with_thermal.begin(), with_thermal.end()));
    assert(decoded.valid && decoded.message.detections.size() == 1);
    assert(decoded.message.detections.at(0).x2 == 300);
    json.at("detections").at(0).erase("thermal");
    json.at("detections").at(0).emplace("unexpected", 1);
    const auto bad = NativeJson::to_msgpack(json);
    assert(!ObservationV2Codec::decode(std::string(bad.begin(), bad.end())).valid);
    std::cout << "PASS observation v2 accepts optional thermal only" << '\n';
}

void manager_test() {
    const auto own = signature(3800, 3400);
    const auto other = signature(3150, 3100);
    const auto cooled = signature(3700, 3350);
    const auto swapped = [&](unsigned id, double t) {
        return frame(id, t, {tank(t, other), neighbor(t, own)});  // ID는 그대로, 열은 맞바뀜
    };
    // off: 열이 맞바뀌어도 기존처럼 받아들인다
    {
        auto m = locked(ThermalIdentityMode::Off, own, other);
        m.observe(swapped(30, .85), at(.85), kMaxAge);
        assert(m.snapshot(at(.85), kMaxAge).reason == "selected_native_id_trusted");
    }
    // on: 옆 후보가 선택 대상 기억과 더 닮으면 그 프레임을 버리고, 돌아오면 바로 수용
    {
        auto m = locked(ThermalIdentityMode::On, own, other);
        m.observe(swapped(30, .85), at(.85), kMaxAge);
        auto q = m.snapshot(at(.85), kMaxAge);
        assert(q.phase == TargetPhase::GapHold && q.reason == "selected_id_thermal_mismatch");
        m.observe(frame(31, .9, {tank(.9, own), neighbor(.9, other)}), at(.9), kMaxAge);
        q = m.snapshot(at(.9), kMaxAge);
        assert(q.phase == TargetPhase::Locked && q.reason == "selected_native_id_trusted");
    }
    // on: 맞바뀐 상태가 0.5초 넘게 이어지면 다른 대상을 따르지 않고 잃은 것으로 본다
    {
        auto m = locked(ThermalIdentityMode::On, own, other);
        TargetRecoverySnapshot q;
        for (unsigned step = 0; step < 13; ++step) {
            const double t = .85 + (step * .05);
            m.observe(swapped(30 + step, t), at(t), kMaxAge);
            q = m.snapshot(at(t), kMaxAge);
            if (q.phase == TargetPhase::Lost) {
                break;
            }
            assert(q.reason == "selected_id_thermal_mismatch");
        }
        assert(q.phase == TargetPhase::Lost &&
               q.reason == "selected_id_thermal_mismatch_persisted");
    }
    // on: 비교할 후보가 없거나(혼자), 둘이 비슷하거나, 열 정보가 없으면(NUC 직후 등) 판단하지
    // 않는다
    {
        auto m = locked(ThermalIdentityMode::On, own, other);
        m.observe(frame(30, .85, {tank(.85, other)}), at(.85), kMaxAge);
        assert(m.snapshot(at(.85), kMaxAge).reason == "selected_native_id_trusted");
    }
    {
        auto m = locked(ThermalIdentityMode::On, own, other);
        m.observe(frame(30, .85, {tank(.85, cooled), neighbor(.85, own)}), at(.85), kMaxAge);
        assert(m.snapshot(at(.85), kMaxAge).reason == "selected_native_id_trusted");
        m.observe(frame(31, .9, {tank(.9, std::nullopt), neighbor(.9, own)}), at(.9), kMaxAge);
        assert(m.snapshot(at(.9), kMaxAge).reason == "selected_native_id_trusted");
        m.observe(frame(32, .95, {tank(.95, other), neighbor(.95, std::nullopt)}), at(.95),
                  kMaxAge);
        assert(m.snapshot(at(.95), kMaxAge).reason == "selected_native_id_trusted");
    }
    // log: 판단은 바꾸지 않고 상대 비교 결과를 한 줄씩 남긴다
    {
        const std::string path = "/tmp/argus-thermal-identity-test.jsonl";
        std::remove(path.c_str());
        auto m = locked(ThermalIdentityMode::Log, own, other, path);
        m.observe(swapped(30, .85), at(.85), kMaxAge);
        assert(m.snapshot(at(.85), kMaxAge).reason == "selected_native_id_trusted");
        std::ifstream in(path);
        std::string line;
        std::string last;
        int lines = 0;
        while (std::getline(in, line)) {
            ++lines;
            last = line;
        }
        assert(lines == 17);
        assert(last.find(R"("mode":"log")") != std::string::npos);
        assert(last.find(R"("best_other_id":23)") != std::string::npos);
        assert(last.find(R"("mismatch":true)") != std::string::npos);
        std::remove(path.c_str());
    }
    std::cout << "PASS thermal identity relative off/log/on" << '\n';
}

// NUC처럼 화면 전체가 바뀐 프레임과 직후 0.5초는 열 외형을 붙이지 않는다
void nuc_guard_test() {
    const auto normal = raw_frame(3800, 3400);
    std::vector<std::uint8_t> nuc(normal.size());
    for (std::size_t i = 0; i + 1 < nuc.size(); i += 2) {
        const auto value = static_cast<int>(1700 + ((i * 13) % 400));
        nuc.at(i) = static_cast<std::uint8_t>(value & 0xFF);
        nuc.at(i + 1) = static_cast<std::uint8_t>(value >> 8);
    }
    argus::ThermalFrameGuard guard;
    std::int64_t now = kOriginNs;
    for (int i = 0; i < 20; ++i, now += 33000000) {
        assert(guard.update(normal, 640, 480, 1280, now));
    }
    assert(!guard.update(nuc, 640, 480, 1280, now));
    now += 33000000;
    assert(!guard.update(normal, 640, 480, 1280, now));  // 직후 0.5초 대기
    now += 600000000;
    assert(guard.update(normal, 640, 480, 1280, now));
    std::cout << "PASS NUC frame guard" << '\n';
}
}  // namespace

int main(int argc, char** argv) {
    try {
        measurement_test();
        codec_test();
        observation_codec_test();
        manager_test();
        nuc_guard_test();
        // 배포 프로필(materialize된 것)을 넘기면 thermal_identity 기본값이 off인지 확인한다
        for (int i = 1; i < argc; ++i) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic) C++17 main 인자
            const auto configured = load_bus_following_config(argv[i]);
            assert(configured.selector.recovery.thermal_identity == ThermalIdentityMode::Off);
        }
        std::cout << "PASS deployed thermal_identity default off (" << (argc - 1) << " profiles)\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
