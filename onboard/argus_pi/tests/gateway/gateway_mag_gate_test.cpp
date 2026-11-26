// 2026-11-18 이륙 조건 보강 시험: PX4 강한 자기장 간섭 경고(0x01efc45f)로 ARM과 이륙을 막는다.
// 검토 반영: ms 경계를 넘는 묶음, 늦은 ARM heartbeat(1.9 s), 불완전 보고의 fail-open.
// 순번, FC 시각은 sess275 ulog(요약 0x01a893e0 묶음, ARM 118.009 s, 해제 169.321 s)를 본떴다.
// FC 없는 FlightLink와 가짜 링크만 쓴다 (소켓, FC, 카메라는 열지 않음).
#include <unistd.h>

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/gateway/events/include/event_parser.hpp"
#include "argus/gateway/service/include/maintenance.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"
#include "argus/gateway/vehicle/include/mag_interference.hpp"

namespace gw = argus::gateway;
using Change = gw::MagInterferenceMonitor::Change;

namespace {

int failures = 0;
int checks = 0;

void check(bool value, const std::string& message) {
    ++checks;
    if (!value) {
        ++failures;
        std::cerr << "FAIL " << message << '\n';
    }
}

constexpr const char* kUid = "000600000000353733323333511700280027";
constexpr std::uint32_t kMag = 0x01efc45fU;
constexpr std::uint32_t kSummary = 0x01a893e0U;
constexpr std::uint32_t kHealth = 0x011d3727U;
// sess275 묶음에 함께 온 다른 검사들 (관계없는 이벤트)
constexpr std::uint32_t kOther[] = {0x0198c273U, 0x0105c2d9U, 0x012f1dc7U, 0x01013630U};
constexpr std::uint32_t kArmedBy = 0x0190caf3U;
constexpr const char* kArmRefusal =
    "FC 강한 자기장 간섭 경고 - 방향(yaw) 정렬이 안 될 수 있습니다. "
    "이륙 위치를 옮기고 경고가 사라진 뒤 ARM하세요";
constexpr const char* kTakeoffRefusal =
    "ARM 직전 FC 강한 자기장 간섭 경고 - 착륙 해제(disarm) 후 위치를 옮겨 다시 시도하세요";

struct Event {
    int sequence;
    std::uint32_t id;
    std::int64_t fc_ms;
};

// 요약 + (경고) + 다른 검사 + 건강 요약. 마지막 순번을 돌려준다.
std::vector<Event> burst(int first, std::int64_t fc_ms, bool mag, int others = 4) {
    std::vector<Event> rows{{first, kSummary, fc_ms}};
    int sequence = first;
    if (mag) {
        rows.push_back({++sequence, kMag, fc_ms});
    }
    for (int i = 0; i < others; ++i) {
        rows.push_back({++sequence, kOther[i], fc_ms});
    }
    rows.push_back({++sequence, kHealth, fc_ms});
    return rows;
}

int last_sequence(const std::vector<Event>& rows) { return rows.back().sequence; }

std::vector<Change> feed(gw::MagInterferenceMonitor& monitor, const std::vector<Event>& rows,
                         std::optional<bool> armed, double now, int epoch = 0) {
    std::vector<Change> changes;
    for (const Event& row : rows) {
        changes.push_back(monitor.observe(epoch, row.sequence, row.id, "", row.fc_ms, armed, now));
    }
    return changes;
}

// FC heartbeat 도착(무장 상태를 알 때)과 주기 판정
Change step(gw::MagInterferenceMonitor& monitor, int epoch, std::optional<bool> armed,
            std::optional<int> latest, double now) {
    if (armed) {
        monitor.observe_heartbeat(*armed, now);
    }
    return monitor.update(epoch, armed, latest, now);
}

bool contains(const std::vector<Change>& changes, Change wanted) {
    for (const Change change : changes) {
        if (change == wanted) {
            return true;
        }
    }
    return false;
}

// sess275: ARM 전 경고 -> ARM 거부, 무장 해제 중 경고 없는 새 묶음 -> 해제 -> ARM 허용
void sess275_pre_arm() {
    gw::MagInterferenceMonitor monitor;
    check(!monitor.active().has_value(), "보고 전 모름");
    check(step(monitor, 0, false, std::nullopt, 100.0) == Change::kNone, "보고 전 변화 없음");
    check(!monitor.active().has_value() && !monitor.armed_with_warning(), "보고 전 막지 않음");
    check(feed(monitor, {{122, kHealth, 93580}}, false, 100.0).back() == Change::kNone,
          "건강 요약만으로 판정 안 함");
    const auto first = burst(123, 95615, true);
    const auto changes = feed(monitor, first, false, 100.2);
    check(changes.at(1) == Change::kSet, "무장 해제 중 경고 즉시 활성");
    check(monitor.active() == std::optional<bool>(true), "경고 활성");
    check(step(monitor, 0, false, last_sequence(first), 104.0) == Change::kNone,
          "경고 묶음 완료: 그대로 활성");
    check(monitor.status().at("last_event_fc_ms") == 95615, "마지막 경고 FC 시각");
    // 경고 없는 새 묶음 (무장 해제 중). 판정은 kSettleS 뒤
    const auto second = burst(131, 118108, false);
    check(!contains(feed(monitor, second, false, 110.0), Change::kCleared), "수신 즉시 해제 안 함");
    check(step(monitor, 0, false, last_sequence(second), 111.0) == Change::kNone,
          "기다리는 동안 활성 유지");
    check(monitor.active() == std::optional<bool>(true), "기다리는 동안 활성");
    check(step(monitor, 0, false, last_sequence(second), 112.1) == Change::kCleared,
          "경고 없는 묶음 -> 해제");
    check(monitor.active() == std::optional<bool>(false), "해제 상태");
    check(step(monitor, 0, true, last_sequence(second), 113.0) == Change::kNone, "ARM");
    check(!monitor.armed_with_warning(), "해제 뒤 ARM은 고정 없음");
    std::cout << "PASS sess275 pre-arm set/clear\n";
}

// 무장 순간 경고 활성 -> 이륙 고정, 무장 중 묶음은 해제하지 않음, 무장 해제에서 고정 풀림
void armed_latch() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, false, std::nullopt, 100.0);
    const auto first = burst(123, 95615, true);
    (void)feed(monitor, first, false, 100.1);
    (void)step(monitor, 0, false, last_sequence(first), 101.0);
    // RC나 다른 GCS가 무장: 무장 이벤트와 ARM 직후 묶음(경고 빠짐)이 heartbeat보다 먼저 온다
    const auto at_arm = burst(131, 118108, false);
    (void)feed(monitor, {{130, kArmedBy, 118009}}, false, 102.0);
    (void)feed(monitor, at_arm, false, 102.1);
    check(step(monitor, 0, true, last_sequence(at_arm), 102.6) == Change::kNone, "ARM 전환");
    check(monitor.armed_with_warning(), "무장 순간 경고 -> 이륙 고정");
    // 무장 중 새 묶음 (경고 빠짐)은 상태를 바꾸지 않는다
    const auto in_air = burst(138, 120155, false, 3);
    (void)feed(monitor, in_air, true, 104.0);
    check(step(monitor, 0, true, last_sequence(in_air), 110.0) == Change::kNone,
          "무장 중 묶음 무시");
    check(monitor.active() == std::optional<bool>(true) && monitor.armed_with_warning(),
          "무장 중 활성, 고정 유지");
    // heartbeat 만료도 고정을 풀지 않는다
    (void)step(monitor, 0, std::nullopt, last_sequence(in_air), 111.0);
    check(monitor.armed_with_warning(), "heartbeat 만료 중 고정 유지");
    // 무장 해제: 고정 풀림. 해제 뒤 새 보고가 없으면 마지막 보고(경고 없음)가 현재 결과
    check(step(monitor, 0, false, last_sequence(in_air), 112.0) == Change::kNone, "해제");
    check(!monitor.armed_with_warning(), "무장 해제 -> 고정 풀림");
    check(step(monitor, 0, false, last_sequence(in_air), 114.0) == Change::kNone,
          "해제 직후 kDisarmStableS 동안 유지");
    check(monitor.active() == std::optional<bool>(true), "해제 직후 활성 유지");
    check(step(monitor, 0, false, last_sequence(in_air), 115.1) == Change::kCleared,
          "해제가 이어진 뒤 마지막 보고로 해제");
    std::cout << "PASS armed latch\n";
}

// 착륙 해제 직후 경고 묶음이 해제 heartbeat보다 먼저 와도 다음 비행 ARM을 막는다 (sess278)
void disarm_burst_race() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, true, std::nullopt, 100.0);
    const auto landing = burst(200, 234955, true);
    const auto changes = feed(monitor, landing, true, 100.5);
    check(!contains(changes, Change::kSet), "무장 중 경고는 바로 바꾸지 않음");
    check(!monitor.active().has_value(), "무장 중 모름 유지");
    (void)step(monitor, 0, false, last_sequence(landing), 101.0);
    check(step(monitor, 0, false, last_sequence(landing), 103.0) == Change::kNone,
          "해제 직후 기다림");
    check(step(monitor, 0, false, last_sequence(landing), 104.1) == Change::kSet,
          "해제가 이어진 뒤 경고 묶음 -> 활성");
    check(monitor.active() == std::optional<bool>(true), "다음 ARM 거부 상태");
    std::cout << "PASS disarm burst race\n";
}

// 묶음 중간 순번이 빠지면 재요청을 기다리고, 늦게 온 경고는 바로 활성
void gap_and_late_member() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, false, std::nullopt, 100.0);
    (void)feed(monitor, {{300, kSummary, 5000}, {302, kOther[0], 5000}}, false, 101.0);
    check(step(monitor, 0, false, 302, 110.0) == Change::kNone, "빠진 순번: 판정 보류");
    check(!monitor.active().has_value(), "보류 중 모름");
    check(monitor.observe(0, 301, kMag, "", 5000, false, 110.5) == Change::kSet,
          "늦게 온 경고 -> 활성");
    check(step(monitor, 0, false, 302, 111.0) == Change::kNone, "완료된 묶음 그대로");
    check(monitor.active() == std::optional<bool>(true), "활성");
    // 요약만 받고 나머지가 아직 없는 경우: FC가 알린 최신 순번까지 받았으면 끝
    (void)feed(monitor, {{303, kSummary, 6000}}, false, 112.0);
    check(step(monitor, 0, false, 305, 115.0) == Change::kNone, "304, 305 대기");
    (void)feed(monitor, {{304, kOther[1], 6000}, {305, kHealth, 6000}}, false, 115.2);
    check(step(monitor, 0, false, 305, 116.0) == Change::kCleared, "다 받은 뒤 해제");
    std::cout << "PASS gap and late member\n";
}

// 이력 재요청으로 늦게 온 예전 묶음의 경고, 관계없는 이벤트는 상태를 바꾸지 않는다
void stale_and_unrelated() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, false, std::nullopt, 100.0);
    const auto newest = burst(400, 9000, false);
    (void)feed(monitor, newest, false, 100.0);
    check(monitor.observe(0, 391, kMag, "", 8000, false, 100.1) == Change::kNone,
          "예전 묶음 경고 무시");
    check(!monitor.active().has_value(), "예전 경고로 활성 안 됨");
    check(step(monitor, 0, false, last_sequence(newest), 103.5) == Change::kNone,
          "모름 -> 경고 없음은 해제 기록 없음");
    check(monitor.active() == std::optional<bool>(false), "경고 없음 확인");
    for (const std::uint32_t id : {kOther[0], kOther[2], kHealth, 0x0145335cU}) {
        check(monitor.observe(0, 500 + static_cast<int>(id & 7U), id, "other_event", 9500, false,
                              104.0) == Change::kNone,
              "관계없는 이벤트 무시");
    }
    check(monitor.active() == std::optional<bool>(false), "관계없는 이벤트 뒤 그대로");
    std::cout << "PASS stale and unrelated events\n";
}

// 해석된 이름만으로도 같은 규칙 (원시 ID가 다른 빌드라고 가정)
void decoded_name() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, false, std::nullopt, 100.0);
    check(monitor.observe(0, 10, 0x01000001U, "commander_arming_check_summary", 700, false,
                          100.0) == Change::kNone,
          "이름 요약");
    check(monitor.observe(0, 11, 0x01000002U, "check_estimator_mag_interference", 700, false,
                          100.0) == Change::kSet,
          "이름 경고 -> 활성");
    check(monitor.observe(0, 12, 0x01000003U, "commander_arming_check_summary", 800, false,
                          101.0) == Change::kNone,
          "이름 새 요약");
    check(step(monitor, 0, false, 12, 103.5) == Change::kCleared, "이름 묶음 해제");
    check(gw::MagInterferenceMonitor::is_mag_interference(kMag, ""), "원시 ID 경고");
    check(gw::MagInterferenceMonitor::is_arming_summary(kSummary, ""), "원시 ID 요약");
    check(!gw::MagInterferenceMonitor::is_mag_interference(kHealth, "commander_health_summary"),
          "건강 요약은 경고 아님");

    // 메타데이터 정의에서 이름 찾기 (0x01efc45f = 구성요소 1, 하위 15713375)
    gw::EventParser parser;
    parser.load_definitions(nlohmann::ordered_json::parse(R"({
        "version": 2,
        "components": {"1": {"namespace": "px4", "event_groups": {"arming_check": {"events": {
            "15713375": {"name": "check_estimator_mag_interference",
                         "message": "Strong magnetic interference"}}}}}}})"));
    check(parser.name(kMag) == std::optional<std::string>("check_estimator_mag_interference"),
          "정의 이름");
    check(!parser.name(kSummary).has_value(), "정의 없는 ID");
    std::cout << "PASS decoded name and raw id\n";
}

// epoch(재부팅, 순번 초기화, 재연결) -> 모름, 고정 해제
void epoch_reset() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, false, std::nullopt, 100.0);
    (void)feed(monitor, burst(123, 95615, true), false, 100.1);
    (void)step(monitor, 0, true, 129, 101.0);
    check(monitor.armed_with_warning(), "고정");
    (void)step(monitor, 1, true, std::nullopt, 102.0);
    check(!monitor.active().has_value() && !monitor.armed_with_warning(), "새 epoch -> 모름");
    check(monitor.status().at("active").is_null() && monitor.status().at("epoch") == 1,
          "상태 표시 모름");
    // 새 epoch에서 ARM 순간 모름이면 고정하지 않는다 (fail-open)
    (void)step(monitor, 1, false, std::nullopt, 103.0);
    (void)step(monitor, 1, true, std::nullopt, 104.0);
    check(!monitor.armed_with_warning(), "모름 상태 ARM은 고정 없음");
    (void)feed(monitor, burst(123, 95615, true), false, 105.0, 1);
    monitor.reset(1);
    check(!monitor.active().has_value(), "명시 초기화 -> 모름");
    std::cout << "PASS epoch reset\n";
}

// 묶음이 ms 경계를 넘는 경우: 요약 1 ms 뒤 경고도 같은 묶음 (sess278 경고 +1 ms 재현)
void straddle_ms_boundary() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, true, std::nullopt, 100.0);
    // 착륙 해제 직후 묶음이 해제 heartbeat보다 먼저 온다 (무장 중이라 바로 바꾸지 않음)
    const std::vector<Event> landing{
        {200, kSummary, 5000}, {201, kMag, 5001}, {202, kOther[0], 5001}, {203, kHealth, 5001}};
    (void)feed(monitor, landing, true, 100.5);
    (void)step(monitor, 0, false, 203, 101.0);
    check(step(monitor, 0, false, 203, 104.1) == Change::kSet, "+1 ms 경고도 묶음 구성원");
    // 무장 해제 중 같은 모양의 묶음: 경고 즉시 활성, 판정 뒤에도 해제되지 않음
    gw::MagInterferenceMonitor other;
    (void)step(other, 0, false, std::nullopt, 100.0);
    const std::vector<Event> rows{
        {10, kSummary, 6000}, {11, kOther[0], 6000}, {12, kMag, 6001}, {13, kHealth, 6001}};
    check(contains(feed(other, rows, false, 100.1), Change::kSet), "+1 ms 경고 즉시 활성");
    check(step(other, 0, false, 13, 104.0) == Change::kNone, "+1 ms 묶음 판정 뒤 활성 유지");
    check(other.active() == std::optional<bool>(true), "활성 유지");
    // 허용 폭(50 ms) 밖의 다음 이벤트는 묶음 끝
    (void)feed(other, {{20, kSummary, 7000}, {21, kOther[0], 7000}, {22, kMag, 7051}}, true, 105.0);
    check(step(other, 0, false, 22, 108.5) == Change::kCleared, "50 ms 밖 경고는 다른 묶음");
    std::cout << "PASS ms boundary straddle\n";
}

// ARM heartbeat가 1.9 s 늦어도 ARM 직후 경고 없는 묶음으로 해제하지 않고 이륙을 고정한다
void delayed_arm_heartbeat() {
    for (const double phase : {0.0, 0.25, 0.5, 0.75, 0.95}) {
        gw::MagInterferenceMonitor monitor;
        const double arm_at = 110.0;
        const double delay = 1.9;
        const auto first = burst(123, 95615, true);
        const auto at_arm = burst(131, 118108, false);
        std::optional<bool> heartbeat;
        double next_sent = 90.0 + phase;
        bool cleared = false;
        bool armed_event_sent = false;
        bool at_arm_sent = false;
        int latest = 0;
        for (int tick = 0; tick <= 1150; ++tick) {
            const double now = 95.0 + tick * .02;
            while (next_sent + delay <= now) {
                heartbeat = next_sent >= arm_at;
                monitor.observe_heartbeat(*heartbeat, next_sent + delay);
                next_sent += 1.0;
            }
            if (now >= 96.0 && latest == 0) {
                (void)feed(monitor, first, heartbeat, now);
                latest = last_sequence(first);
            }
            if (now >= arm_at + .005 && !armed_event_sent) {
                (void)monitor.observe(0, 130, kArmedBy, "", 118009, heartbeat, now);
                armed_event_sent = true;
            }
            if (now >= arm_at + .1 && !at_arm_sent) {
                (void)feed(monitor, at_arm, heartbeat, now);
                latest = last_sequence(at_arm);
                at_arm_sent = true;
            }
            cleared = cleared || monitor.update(0, heartbeat, latest, now) == Change::kCleared;
        }
        check(!cleared, "ARM 직후 묶음으로 해제 안 함 phase " + std::to_string(phase));
        check(monitor.armed_with_warning(), "이륙 고정 phase " + std::to_string(phase));
    }
    std::cout << "PASS delayed arm heartbeat 1.9 s\n";
}

// 지난 이력의 무장 이벤트 뒤 무장 heartbeat가 없으면 kArmedByTimeoutS 뒤 정상 판정
void stale_armed_by() {
    gw::MagInterferenceMonitor monitor;
    (void)step(monitor, 0, false, std::nullopt, 100.0);
    (void)monitor.observe(0, 50, kArmedBy, "", 1000, false, 100.0);
    (void)feed(monitor, burst(60, 2000, false), false, 100.1);
    check(step(monitor, 0, false, 65, 104.0) == Change::kNone, "무장 이벤트 대기 중 판정 안 함");
    check(!monitor.active().has_value(), "대기 중 모름");
    (void)step(monitor, 0, false, 65, 105.1);
    check(!monitor.armed_with_warning(), "대기 만료 뒤 고정 없음");
    (void)step(monitor, 0, false, 65, 108.2);
    check(monitor.active() == std::optional<bool>(false), "대기 만료 뒤 정상 판정");
    // 해제 이벤트는 고정을 바로 푼다
    (void)feed(monitor, {{70, kSummary, 3000}, {71, kMag, 3000}}, false, 109.0);
    (void)monitor.observe(0, 72, kArmedBy, "", 3500, false, 110.0);
    check(monitor.armed_with_warning(), "무장 이벤트 순간 경고 -> 고정");
    (void)monitor.observe(0, 73, gw::MagInterferenceMonitor::kDisarmedById, "", 3600, false, 110.5);
    check(!monitor.armed_with_warning(), "해제 이벤트 -> 고정 풀림");
    std::cout << "PASS stale armed_by and disarmed_by\n";
}

// 빠진 구성원을 더 받을 수 없으면 모름 (fail-open), 한 번만 알림
void incomplete_report() {
    std::vector<int> pending{141};
    gw::MagInterferenceMonitor monitor;
    monitor.set_pending_query([&pending](int sequence) {
        for (const int item : pending) {
            if (item == sequence) {
                return true;
            }
        }
        return false;
    });
    (void)step(monitor, 0, false, std::nullopt, 100.0);
    (void)feed(monitor, burst(123, 95615, true), false, 100.0);
    (void)feed(monitor, {{140, kSummary, 99000}, {142, kHealth, 99000}}, false, 101.0);
    check(step(monitor, 0, false, 142, 104.0) == Change::kNone, "재요청 중 판정 보류");
    check(monitor.active() == std::optional<bool>(true), "보류 중 활성 유지");
    pending.clear();
    check(step(monitor, 0, false, 142, 104.5) == Change::kUnknown, "재요청 끝남 -> 모름");
    check(!monitor.active().has_value() && monitor.status().at("reason") == "incomplete_report",
          "모름과 까닭 표시");
    check(step(monitor, 0, false, 142, 105.0) == Change::kNone, "같은 묶음은 한 번만");
    (void)feed(monitor, burst(143, 99500, false), false, 106.0);
    (void)step(monitor, 0, false, 148, 108.5);
    check(monitor.active() == std::optional<bool>(false) && monitor.status().at("reason").is_null(),
          "다음 완전한 묶음으로 다시 확인");
    // 재요청 정보가 없으면 kIncompleteTimeoutS 뒤 모름
    gw::MagInterferenceMonitor plain;
    (void)step(plain, 0, false, std::nullopt, 100.0);
    (void)feed(plain, burst(123, 95615, true), false, 100.0);
    (void)feed(plain, {{140, kSummary, 99000}, {142, kHealth, 99000}}, false, 101.0);
    check(step(plain, 0, false, 142, 110.9) == Change::kNone, "10 s 전 보류");
    check(step(plain, 0, false, 142, 111.1) == Change::kUnknown, "10 s 뒤 모름");
    std::cout << "PASS incomplete report fail-open\n";
}

class FakeLink final : public gw::ILink {
public:
    gw::Bytes read_available() override { return {}; }
    void write(const gw::Bytes& /*data*/) override {}
    void close() override {}
};

class TestLink final : public gw::FlightLink {
public:
    using gw::FlightLink::FlightLink;
    using gw::FlightLink::ingest;
    using gw::FlightLink::observed;
    using gw::FlightLink::takeoff_climb_stalled;
    using gw::FlightLink::update_mag_interference;
};

struct Rig {
    double now = 100.0;
    std::unique_ptr<TestLink> link;

    Rig() {
        gw::FlightLinkOptions options;
        options.expected_uid = kUid;
        options.event_metadata = "/nonexistent/fc-events.json";
        options.clock = [this] { return now; };
        link = std::make_unique<TestLink>(std::move(options));
        auto& fc = link->state();
        fc.link = std::make_unique<FakeLink>();
        fc.mav = std::make_unique<gw::MavlinkSender>(245, 191);
        fc.uid = link->expected_uid();
        fc.config_requested = true;
        for (const auto& [key, value] : gw::indoor_profile()) {
            fc.params.insert_or_assign(key, value);
        }
        fc.params.insert_or_assign("EKF2_OF_QMIN", 1);
        fc.params.insert_or_assign("COM_POS_FS_EPH", 5);
        fc.params.insert_or_assign("SENS_FLOW_MAXHGT", 2);
    }

    void seed(bool armed) {
        auto& fc = link->state();
        const auto set = [&](const char* name, gw::Json data) {
            fc.samples.insert_or_assign(name, gw::Sample{std::move(data), now});
        };
        const auto numbers = gw::mode_numbers("position").value_or(std::pair<int, int>{0, 0});
        set("HEARTBEAT", {{"base_mode", armed ? 128 : 0},
                          {"custom_mode", (numbers.first << 16) | (numbers.second << 24)}});
        set("EXTENDED_SYS_STATE", {{"landed_state", 1}});
        const std::int64_t bits = 3 | 268435456;
        set("SYS_STATUS", {{"onboard_control_sensors_present", bits},
                           {"onboard_control_sensors_enabled", bits},
                           {"onboard_control_sensors_health", bits},
                           {"voltage_battery", 16000},
                           {"current_battery", 100},
                           {"battery_remaining", 90}});
        set("ESTIMATOR_STATUS", {{"flags", 79}, {"pos_horiz_accuracy", .1}});
        set("LOCAL_POSITION_NED",
            {{"x", 0}, {"y", 0}, {"z", -.1}, {"vx", 0}, {"vy", 0}, {"vz", 0}});
        set("ATTITUDE", {{"roll", 0}, {"pitch", 0}, {"yaw", .2}});
        set("DISTANCE_SENSOR",
            {{"current_distance", 10}, {"min_distance", 8}, {"max_distance", 400}});
        set("OPTICAL_FLOW_RAD", {{"quality", 50}});
    }

    void event(const Event& row) {
        gw::InboundMessage message;
        message.kind = "EVENT";
        message.system = 1;
        message.component = 1;
        message.data = {{"sequence", row.sequence},        {"id", row.id},
                        {"event_time_boot_ms", row.fc_ms}, {"log_levels", 68},
                        {"arguments", gw::Json::array()},  {"destination_component", 0},
                        {"destination_system", 0}};
        link->ingest(message);
    }

    void events(const std::vector<Event>& rows) {
        for (const Event& row : rows) {
            event(row);
        }
    }

    // 시간을 옮기고 표본과 FC heartbeat를 새로 넣은 뒤 주기 판정
    void tick(double dt, bool armed) {
        now += dt;
        seed(armed);
        gw::InboundMessage message;
        message.kind = "HEARTBEAT";
        message.system = 1;
        message.component = 1;
        message.data = link->state().samples.at("HEARTBEAT").data;
        message.data.update(
            {{"type", 2}, {"autopilot", 12}, {"system_status", 4}, {"mavlink_version", 3}});
        link->ingest(message);
        link->update_mag_interference();
    }

    bool recorded(const std::string& text) const {
        for (const gw::Json& row : link->state().events) {
            if (row.at("text") == text) {
                return true;
            }
        }
        return false;
    }
};

void flight_link_paths() {
    Rig rig;
    rig.tick(0, false);
    const std::string arm_baseline = rig.link->reason("arm");
    check(arm_baseline.empty(), "보고 전 ARM 허용 (fail-open): " + arm_baseline);
    check(rig.link->snapshot().at("mag_interference").at("active").is_null(), "/state 모름 표시");
    const auto first = burst(123, 95615, true);
    rig.events(first);
    check(rig.link->reason("arm") == kArmRefusal, "경고 -> ARM 거부: " + rig.link->reason("arm"));
    check(rig.link->snapshot().at("reasons").at("arm") == kArmRefusal, "/state 거부 사유");
    check(rig.link->snapshot().at("mag_interference").at("active") == true, "/state 활성");
    check(rig.recorded("FC 강한 자기장 간섭 경고 감지"), "감지 기록");
    bool refused = false;
    try {
        (void)rig.link->submit("arm", "gui", nullptr);
    } catch (const gw::ValueError& error) {
        refused = std::string(error.what()) == kArmRefusal;
    }
    check(refused, "정비 GUI /command arm 거부");
    // 경고 없는 새 묶음 -> 해제 -> ARM 허용
    const auto second = burst(131, 118108, false);
    rig.events(second);
    rig.tick(1.0, false);
    check(rig.link->reason("arm") == kArmRefusal, "기다리는 동안 거부 유지");
    rig.tick(2.5, false);
    check(rig.link->reason("arm").empty(), "해제 뒤 ARM 허용: " + rig.link->reason("arm"));
    check(rig.recorded("FC 강한 자기장 간섭 경고 해제"), "해제 기록");
    // 해제 상태 ARM: 이륙 막지 않음
    rig.tick(.1, true);
    const std::string takeoff_baseline = rig.link->reason("takeoff");
    check(takeoff_baseline != kTakeoffRefusal, "경고 없는 ARM은 이륙 막지 않음");
    rig.tick(.1, false);
    // 경고 활성 중 RC 무장 -> 이륙 거부, 무장 해제까지 유지
    rig.events(burst(140, 130000, true));
    check(rig.link->mag_arm_reason() == kArmRefusal, "다시 활성");
    rig.tick(.5, true);
    check(rig.link->reason("takeoff") == kTakeoffRefusal,
          "무장 순간 경고 -> 이륙 거부: " + rig.link->reason("takeoff"));
    check(rig.link->snapshot().at("mag_interference").at("armed_with_warning") == true,
          "/state 고정 표시");
    rig.events(burst(147, 131000, false));
    rig.tick(5.0, true);
    check(rig.link->reason("takeoff") == kTakeoffRefusal, "무장 중 묶음으로 풀리지 않음");
    rig.tick(.5, false);
    check(rig.link->mag_takeoff_reason().empty(), "무장 해제 -> 이륙 고정 풀림");
    // 빠진 구성원이 끝내 안 오면 모름, 기록과 /state 까닭
    rig.events({{160, kSummary, 140000}, {162, kHealth, 140000}});
    rig.tick(10.5, false);
    check(rig.recorded("FC 자기장 경고 보고 불완전 - 상태 미확인, 재연결 권장"), "불완전 기록");
    check(rig.link->snapshot().at("mag_interference").at("reason") == "incomplete_report",
          "/state 불완전 까닭");
    check(rig.link->reason("arm").empty(), "모름은 ARM 막지 않음");
    // 연결 종료 -> 모름
    rig.link->close_link();
    check(rig.link->mag_interference().active() == std::nullopt, "연결 종료 -> 모름");
    std::cout << "PASS flight link arm/takeoff paths (arm baseline ok, takeoff baseline \""
              << takeoff_baseline << "\")\n";
}

}  // namespace

// 2026-11-20 이륙 완료 허용 오차 0.30 m (고도 변경 0.10 m는 그대로). 실외 10:29 이륙: 목표 4.0 m,
// 레이저 3.83~3.90 m에서 30초 동안 완료를 못 봐 착륙했다.
void takeoff_tolerance() {
    Rig rig;
    rig.seed(true);
    auto& fc = rig.link->state();
    const auto offboard = gw::mode_numbers("offboard").value_or(std::pair<int, int>{0, 0});
    const auto in_air = [&](double laser_cm) {
        fc.samples.insert_or_assign(
            "HEARTBEAT",
            gw::Sample{{{"base_mode", 128},
                        {"custom_mode", (offboard.first << 16) | (offboard.second << 24)}},
                       rig.now});
        fc.samples.insert_or_assign("EXTENDED_SYS_STATE",
                                    gw::Sample{{{"landed_state", 2}}, rig.now});
        fc.samples.insert_or_assign("DISTANCE_SENSOR", gw::Sample{{{"current_distance", laser_cm},
                                                                   {"min_distance", 8},
                                                                   {"max_distance", 1200}},
                                                                  rig.now});
    };
    const gw::Json job{{"action", "takeoff"}, {"ack", true}, {"height", 4.0}};
    in_air(385);
    check(rig.link->observed(job), "레이저 3.85 m, 목표 4.0 m -> 이륙 완료");
    in_air(371);
    check(rig.link->observed(job), "레이저 3.71 m -> 이륙 완료 (0.30 m 안)");
    in_air(365);
    check(!rig.link->observed(job), "레이저 3.65 m -> 아직 아님");
    gw::FlightLimits limits;
    check(limits.takeoff_tolerance == .30 && limits.target_tolerance == .10,
          "이륙 0.30 m, 고도 변경 0.10 m");
    std::cout << "PASS takeoff tolerance 0.30 m\n";
}

// 2026-11-24 이륙 상승 멈춤: 목표 setpoint에 닿은 뒤 떠 있고 |vz| < 0.1 m/s가 2초 이어지면 완료.
void takeoff_stall() {
    Rig rig;
    rig.seed(true);
    auto& fc = rig.link->state();
    const auto offboard = gw::mode_numbers("offboard").value_or(std::pair<int, int>{0, 0});
    const auto set = [&](double laser_cm, double vz, bool off = true, int landed = 2) {
        const auto numbers =
            off ? offboard : gw::mode_numbers("position").value_or(std::pair<int, int>{0, 0});
        fc.samples.insert_or_assign(
            "HEARTBEAT",
            gw::Sample{{{"base_mode", 128},
                        {"custom_mode", (numbers.first << 16) | (numbers.second << 24)}},
                       rig.now});
        fc.samples.insert_or_assign("EXTENDED_SYS_STATE",
                                    gw::Sample{{{"landed_state", landed}}, rig.now});
        fc.samples.insert_or_assign("DISTANCE_SENSOR", gw::Sample{{{"current_distance", laser_cm},
                                                                   {"min_distance", 8},
                                                                   {"max_distance", 1200}},
                                                                  rig.now});
        fc.samples.insert_or_assign(
            "LOCAL_POSITION_NED",
            gw::Sample{
                {{"x", 0}, {"y", 0}, {"z", -laser_cm / 100.}, {"vx", 0}, {"vy", 0}, {"vz", vz}},
                rig.now});
    };
    const auto start = [&](double climb_ago_s) {
        fc.pending = std::make_shared<gw::Json>(
            gw::Json{{"action", "takeoff"}, {"ack", true}, {"height", 1.5}, {"stage", "command"}});
        fc.local_target = std::make_shared<gw::Json>(gw::Json{{"x", 0},
                                                              {"y", 0},
                                                              {"start_z", -.1},
                                                              {"target_z", -1.5},
                                                              {"distance", 1.4},
                                                              {"climb_at", rig.now - climb_ago_s}});
    };
    // 목표 1.5 m, 레이저 1.10 m에서 멈춤 (허용 0.30 m 밖이라 observed는 아님)
    start(10);
    set(110, 0);
    check(!rig.link->observed(*fc.pending), "1.10 m는 목표 도달 아님");
    check(!rig.link->takeoff_climb_stalled(), "처음은 시작만 기록");
    rig.now += 1.0;
    set(110, .02);
    check(!rig.link->takeoff_climb_stalled(), "1초는 아직");
    rig.now += 1.1;
    set(110, -.03);
    check(rig.link->takeoff_climb_stalled(), "2초 멈춤 -> 완료");
    // 상승 중(vz 0.2)이면 다시 처음부터
    start(10);
    set(110, 0);
    (void)rig.link->takeoff_climb_stalled();
    rig.now += 1.5;
    set(110, -.2);
    check(!rig.link->takeoff_climb_stalled(), "상승 중이면 초기화");
    rig.now += 1.0;
    set(110, 0);
    check(!rig.link->takeoff_climb_stalled(), "초기화 뒤 다시 시작");
    // setpoint 상승이 아직 안 끝났으면(1.4 m / 0.25 m/s = 5.6 s) 아님
    start(3);
    set(110, 0);
    (void)rig.link->takeoff_climb_stalled();
    rig.now += 2.1;
    set(110, 0);
    check(!rig.link->takeoff_climb_stalled(), "상승 setpoint 진행 중");
    // 너무 낮으면(0.40 m < 0.5 m) 아님
    start(10);
    set(40, 0);
    (void)rig.link->takeoff_climb_stalled();
    rig.now += 2.1;
    set(40, 0);
    check(!rig.link->takeoff_climb_stalled(), "0.40 m는 하한 미만");
    // offboard가 아니면 아님
    start(10);
    set(110, 0, false);
    (void)rig.link->takeoff_climb_stalled();
    rig.now += 2.1;
    set(110, 0, false);
    check(!rig.link->takeoff_climb_stalled(), "offboard 아님");
    // 데모 범위(1.0~2.0 m)면 하한이 1.0 m
    rig.link->set_height_band_source([]() -> std::optional<gw::FlightLink::HeightBand> {
        return gw::FlightLink::HeightBand{1.0, 2.0, "demo"};
    });
    start(10);
    set(90, 0);
    (void)rig.link->takeoff_climb_stalled();
    rig.now += 2.1;
    set(90, 0);
    check(!rig.link->takeoff_climb_stalled(), "데모 0.90 m는 하한 미만");
    start(10);
    set(105, 0);
    (void)rig.link->takeoff_climb_stalled();
    rig.now += 2.1;
    set(105, 0);
    check(rig.link->takeoff_climb_stalled(), "데모 1.05 m 멈춤 -> 완료");
    rig.link->set_height_band_source({});
    gw::FlightLimits limits;
    check(limits.takeoff_stall_min_height == .5 && limits.takeoff_stall_speed == .1 &&
              limits.takeoff_stall_hold_s == 2.0,
          "멈춤 기준 0.5 m, 0.1 m/s, 2 s");
    std::cout << "PASS takeoff climb stall\n";
}

// 2026-11-24 정비 프로세스: FC 장치 없을 때 카메라/탐지만 종료 허용, 전체 재시작 상태 읽기
void maintenance_fc_absent() {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / ("maint_test_" + std::to_string(::getpid()));
    fs::create_directories(root / "run/connection");
    const fs::path device = root / "ttyACM_test";
    gw::MaintenanceOptions options;
    options.root = root;
    options.python = "python3";
    options.uart_device = device.string();
    const gw::Maintenance maintenance(options);
    check(maintenance.fc_link_absent(), "장치 파일 없음 -> FC 없음");
    std::ofstream(device) << "";
    check(!maintenance.fc_link_absent(), "장치 파일 있음 -> FC 있음");
    gw::MaintenanceOptions empty = options;
    empty.uart_device.clear();
    check(!gw::Maintenance(empty).fc_link_absent(), "장치 설정 없음 -> 판정 안 함");
    for (const char* name : {"argus-onboard", "i3_managed_capture", "argus_preview_bus",
                             "argus-frame-source", "argus_preview_udp_relay"}) {
        check(gw::Maintenance::perception_only(name), std::string("카메라/탐지 허용 ") + name);
    }
    for (const char* name : {"argus_follow_controller", "argus_follow_tracker", "IR FC 연결부",
                             "RGB/argus_follow_controller", "onboard(메인 관제 연결)"}) {
        check(!gw::Maintenance::perception_only(name), std::string("FC 관련 제외 ") + name);
    }
    check(maintenance.restart_status().is_null(), "재시작 기록 없음 -> null");
    std::ofstream(root / "run/connection/restart-all.json")
        << "{\"phase\": \"stopping\", \"log\": []}";
    check(maintenance.restart_status().at("phase") == "stopping", "재시작 상태 읽기");
    std::ofstream(root / "run/connection/restart-all.json") << "{broken";
    check(maintenance.restart_status().is_null(), "깨진 재시작 기록 -> null");
    fs::remove_all(root);
    std::cout << "PASS maintenance fc absent\n";
}

int main() {
    try {
        sess275_pre_arm();
        takeoff_tolerance();
        takeoff_stall();
        maintenance_fc_absent();
        armed_latch();
        disarm_burst_race();
        gap_and_late_member();
        stale_and_unrelated();
        decoded_name();
        epoch_reset();
        straddle_ms_boundary();
        delayed_arm_heartbeat();
        stale_armed_by();
        incomplete_report();
        flight_link_paths();
    } catch (const std::exception& error) {
        std::cerr << "FAIL exception " << error.what() << '\n';
        return 1;
    }
    std::cout << checks - failures << "/" << checks << " checks passed\n";
    return failures == 0 ? 0 : 1;
}
