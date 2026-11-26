// 공중 body-yaw 펄스 시험 (원본 manual_yaw.py ManualYaw). 기존 MAVLink 소유 스레드가 처리한다.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "argus/gateway/follow/include/follow_owner.hpp"
#include "argus/gateway/follow/include/scan_ramp.hpp"

namespace argus::gateway {

namespace {

constexpr double kDegreesPerRadian = M_PI / 180.0;  // 원본 math.radians/degrees의 계수
constexpr std::uint8_t kBodyNed = 8;
constexpr std::uint16_t kVelocityMask = 1479;
// Offboard 탐색 기본값·상한. 상한은 제안값. ICD 45002 기본 속도는 10°/s다.
// 2026-11-12 20°/s로 올렸다가, 2026-11-13 실내 비행에서 20°/s 스캔 중 플로우 위치 추정 오차가
// 커져(목표 대비 평균 0.13~0.20 m, 최대 0.44 m; 10°/s는 평균 0.1 m) 15°/s·10°/s로 낮췄다가,
// 같은 날 사용자 지시로 돌다 멈추기(scan_ramp.hpp)를 넣고, 다시 구간 20°·최고 15°/s와 밀림 멈춤
// (0.15 m 넘게 밀리면 멈추고 0.10 m·0.05 m/s 안으로 돌아오면 재개)을 더했다. 360°는 밀림 멈춤 없이
// 약 50초 걸리므로 기본 제한 시간을 60초에서 90초로 늘렸다.
// 2026-11-19 사용자 지시: 한 방향 회전(turn)은 오른쪽(시계 방향)으로 180°만, 중간 정지 없이 돈다.
// yaw 중 플로우가 회전각만큼 가짜 이동을 쌓아 기체가 밀린 실내 시험(2026-11-18) 때문에 회전각을
// 줄였다. 요청 범위가 180°를 넘으면(GCS 기본값 2π 포함) 180°로 줄이고, 요청 속도의 부호와 관계없이
// 오른쪽으로 돈다. 밀림 멈춤(scan_ramp.hpp)은 안전장치로 남긴다. 효과는 실기 확인 전이다.
constexpr double kScanRateDegS = 15.0;
constexpr double kScanSweepDeg = 180.0;
constexpr double kScanTurnMaxDeg = 180.0;
constexpr double kScanTimeoutS = 90.0;
constexpr double kScanMaxRateDegS = 30.0;
constexpr double kScanMaxTimeoutS = 120.0;

double radians(double degrees) { return degrees * kDegreesPerRadian; }

double degrees(double radians_value) { return radians_value / kDegreesPerRadian; }

// 원본 type(x) in (int, float) and math.isfinite(x)
bool finite_number(const Json& value) {
    return value.is_number() && std::isfinite(value.get<double>());
}

Json sample_field(const FlightLink& v, const char* kind, const char* key) {
    const Json* data = v.sample(kind);
    if (data == nullptr || !data->is_object()) {
        return nullptr;
    }
    const auto found = data->find(key);
    return found == data->end() ? Json() : *found;
}

double heartbeat_at(const FlightLink& v) {
    const auto found = v.state().samples.find("HEARTBEAT");
    if (found == v.state().samples.end()) {
        throw std::runtime_error("'HEARTBEAT'");  // 원본 KeyError
    }
    return found->second.at;
}

}  // namespace

ManualYaw::ManualYaw(FollowBOwner& owner, FlightLink& vehicle) : owner_(owner), v_(vehicle) {
    // 2026-11-18 탐색 방식 전환: ARGUS_SCAN_PATTERN=sector 이면 좌우 왕복, 없거나 turn이면 기존
    // 회전.
    const char* pattern = std::getenv("ARGUS_SCAN_PATTERN");  // NOLINT(concurrency-mt-unsafe)
    if (pattern != nullptr && *pattern != '\0') {
        const auto parsed = parse_scan_pattern(pattern);
        if (parsed) {
            default_scan_pattern_ = *parsed;
        } else {
            std::cerr << "ARGUS_SCAN_PATTERN must be turn or sector; using turn\n";
        }
    }
    const char* continuous =
        std::getenv("ARGUS_SCAN_SECTOR_CONTINUOUS");  // NOLINT(concurrency-mt-unsafe)
    if (continuous != nullptr && *continuous != '\0') {
        const std::string value = continuous;
        sector_continuous_ = !(value == "0" || value == "false" || value == "off");
    }
    const char* first = std::getenv("ARGUS_SCAN_SECTOR_FIRST");  // NOLINT(concurrency-mt-unsafe)
    if (first != nullptr && *first != '\0') {
        const std::string value = first;
        if (value == "left" || value == "right") {
            sector_first_left_ = value == "left";
        } else {
            std::cerr << "ARGUS_SCAN_SECTOR_FIRST must be left or right; using right\n";
        }
    }
    // 2026-11-18 왕복 기본 전체 폭(°). ARGUS_SCAN_SECTOR_WIDTH_DEG, 0 초과 180 이하. 없으면 120.
    const char* width =
        std::getenv("ARGUS_SCAN_SECTOR_WIDTH_DEG");  // NOLINT(concurrency-mt-unsafe)
    if (width != nullptr && *width != '\0') {
        char* end = nullptr;
        const double value = std::strtod(width, &end);
        if (end != width && *end == '\0' && std::isfinite(value) && value > 0 &&
            value <= kScanSectorMaxDeg) {
            sector_width_deg_ = value;
        } else {
            std::cerr << "ARGUS_SCAN_SECTOR_WIDTH_DEG must be in (0, 180]; using " << kScanSectorDeg
                      << '\n';
        }
    }
    std::cerr << "scan pattern: " << scan_pattern_name(default_scan_pattern_)
              << (default_scan_pattern_ == ScanPattern::Sector
                      ? std::string(sector_continuous_ ? " (continuous" : " (step pauses") +
                            (sector_first_left_ ? ", left first" : ", right first") + ", width " +
                            std::to_string(static_cast<int>(sector_width_deg_)) + " deg)"
                      : std::string())
              << '\n';
}

bool ManualYaw::sector_scan() const {
    return scanning() && state_.contains("scan_pattern") && state_.at("scan_pattern") == "sector";
}

// 왕복 탐색 한 틱: 상태(JSON) <-> SectorScanState 변환을 한곳에 둔다.
double ManualYaw::sector_rate(double now, double signed_progress, std::optional<ScanDrift> drift) {
    const double rate = state_number("rate_deg_s");
    const double width = state_number("sweep_deg");
    const int cycles = static_cast<int>(state_.at("sector_cycles").get<std::int64_t>());
    SectorScanState sector;
    sector.leg = static_cast<int>(state_.at("sector_leg").get<std::int64_t>());
    sector.leg_start_deg = state_number("leg_start_deg");
    sector.step = StepScanState{state_number("step_end_deg"), state_number("step_start_at"),
                                state_number("pause_until"), state_number("drift_since")};
    // 시작 회전 방향을 +로 둔 부호각
    const double along = std::copysign(1.0, rate) * signed_progress;
    const bool continuous =
        !state_.contains("sector_continuous") || python_truthy(state_.at("sector_continuous"));
    const double command =
        sector_scan_rate(rate, width, cycles, now, along, sector, drift, continuous);
    if (sector.leg != state_.at("sector_leg").get<std::int64_t>()) {
        v_.record_event("manual_yaw scan sector leg " + std::to_string(sector.leg) + "/" +
                        std::to_string(sector_leg_count(cycles)) + " at " + std::to_string(along) +
                        "deg");
    }
    json_set(state_, "sector_leg", sector.leg);
    json_set(state_, "leg_start_deg", sector.leg_start_deg);
    json_set(state_, "leg_rate_deg_s", sector_leg_rate(rate, cycles, width, sector));
    json_set(state_, "step_end_deg", sector.step.step_end_deg);
    json_set(state_, "step_start_at", sector.step.step_start_s);
    json_set(state_, "pause_until", sector.step.pause_until_s);
    json_set(state_, "drift_since", sector.step.drift_since_s);
    return command;
}

double ManualYaw::state_number(const char* key) const { return state_.at(key).get<double>(); }

std::string ManualYaw::fault() const {
    const auto& fc = v_.state();
    if (!fc.link || fc.uid != v_.expected_uid() ||
        !v_.fresh("HEARTBEAT", v_.limits().heartbeat_ttl)) {
        return "FC UID/heartbeat 확인 필요";
    }
    const auto& owner = v_.command_owner();
    if (active() && !owner.path().empty() && !owner.held()) {
        return "송신권 상실";
    }
    if (!v_.armed() || v_.ground() != false) {
        return "이륙 완료 후 공중 상태에서만 가능합니다";
    }
    if (!v_.fresh("ATTITUDE", .2) || !v_.fresh("LOCAL_POSITION_NED", .2)) {
        return "신선한 자세/위치 필요";
    }
    if (!finite_number(sample_field(v_, "ATTITUDE", "yaw"))) {
        return "유효한 방향 추정 필요";
    }
    const auto height = v_.height().first;
    if (!v_.local_ok() || !height || !v_.navigation_reason().empty()) {
        return "FC 위치·고도·항법 센서 확인 필요";
    }
    if (*height >= v_.limits().brake_height()) {
        return "고도 보호 한계 접근";
    }
    if (owner_.rc_override() && active()) {
        return "수동 조종 인계 상태";
    }
    if (!v_.gcs_active()) {
        return "관제 생존 신호 확인 필요";
    }
    return "";
}

std::string ManualYaw::ready_reason() const {
    std::string reason = fault();
    if (!reason.empty()) {
        return reason;
    }
    const auto& fc = v_.state();
    if (active() || owner_.token_active() || owner_.bench_active() || owner_.pending_active() ||
        fc.pending || !fc.jobs.empty() || fc.landing) {
        return "추종/탐색/다른 시험·명령을 먼저 중지하세요";
    }
    const auto current = v_.mode();
    if (current == std::optional<std::string>("hold")) {
        return "";
    }
    const auto& owner = v_.command_owner();
    if (current == std::optional<std::string>("offboard") && fc.external && fc.owned_flight &&
        fc.owned_mode_confirmed && valid_goal() && (owner.path().empty() || owner.held())) {
        return "";
    }
    return "이 GUI의 이륙 완료 상태 또는 실제 HOLD가 필요합니다";
}

Json ManualYaw::request(const Json& body) {
    const Json uid =
        body.is_object() && body.contains("expected_uid") ? body.at("expected_uid") : Json();
    if (uid != Json(v_.expected_uid())) {
        throw ValueError("FC UID 불일치");
    }
    const Json action = body.is_object() && body.contains("action") ? body.at("action") : Json();
    if (action == "manual_yaw_stop") {
        if (active()) {
            json_set(state_, "stop_requested", true);
            const Json why = body.contains("reason") ? body.at("reason") : Json();
            if (why.is_string() && !why.get<std::string>().empty()) {
                json_set(state_, "stop_reason", why);
            }
        }
        return {{"accepted", true}, {"reason", "yaw 중지 요청"}, {"manual_yaw", status()}};
    }
    if (action == "manual_yaw_scan") {
        return scan_request(body);
    }
    const Json direction = body.contains("direction") ? body.at("direction") : Json();
    const Json request_id = body.contains("request_id") ? body.at("request_id") : Json();
    if ((direction != "left" && direction != "right") || !request_id.is_string() ||
        utf8_length(request_id.get<std::string>()) == 0 ||
        utf8_length(request_id.get<std::string>()) > 128) {
        throw ValueError("좌/우 방향과 고유 요청 ID가 필요합니다");
    }
    const std::string id = request_id.get<std::string>();
    const std::string side = direction.get<std::string>();
    for (const auto& [known, value] : requests_) {
        if (known == id) {
            if (value != side) {
                throw ValueError("request_id_conflict");
            }
            return {{"accepted", true},
                    {"duplicate", true},
                    {"reason", "이미 처리한 요청 · 재송신하지 않음"}};
        }
    }
    const std::string reason = ready_reason();
    if (!reason.empty()) {
        throw ValueError(reason);
    }
    if (!v_.command_owner().acquire()) {
        throw ValueError("다른 송신자가 FC를 소유 중입니다");
    }
    const double now = v_.now();
    owner_.set_rc_override(false);  // 확인된 Hold/GUI 비행에서 명시 요청한 경우만
    requests_.emplace_back(id, side);
    while (requests_.size() > 256) {
        requests_.erase(requests_.begin());
    }
    state_ = Json{{"phase", "priming"},    {"reason", "prestream"},
                  {"request_id", id},      {"rate_deg_s", side == "left" ? -5.0 : 5.0},
                  {"duration_s", 2.0},     {"epoch", v_.state().pose_epoch},
                  {"phase_at", now},       {"deadline", now + 4.0},
                  {"last_tick", now},      {"last_send", -std::numeric_limits<double>::infinity()},
                  {"sent_count", 0},       {"acked", false},
                  {"initial_yaw", nullptr}};
    goal_.reset();
    if (v_.mode() == std::optional<std::string>("offboard")) {
        // 기존 LOCAL_NED 위치 송신만 유지하고 yaw 필드만 바꾼다. FC에 Offboard를 떠나라고 하지
        // 않는다.
        json_set(state_, "phase", "queued");
        json_set(state_, "reason", "position_yaw_pending");
        json_set(state_, "direct", true);
        json_set(state_, "deadline", now + .2);
        json_set(state_, "last_yaw", sample_field(v_, "ATTITUDE", "yaw"));
        goal_ = v_.state().local_target;
    }
    v_.record_event("manual_yaw requested " + side + " " + id);
    return {{"accepted", true},
            {"reason", "수동 yaw 시험 준비 · 5°/초 · 2초"},
            {"manual_yaw", status()}};
}

// 2026-11-09: GPS 없는 PX4는 HOLD를 거절하므로, Offboard 제자리 유지 중 위치·고도 목표는
// 그대로 두고 yaw 속도만 얹어 도는 탐색. 실제 yaw 누적 각도로 끝을 판정한다.
Json ManualYaw::scan_request(const Json& body) {
    const Json request_id = body.contains("request_id") ? body.at("request_id") : Json();
    if (!request_id.is_string() || utf8_length(request_id.get<std::string>()) == 0 ||
        utf8_length(request_id.get<std::string>()) > 128) {
        throw ValueError("탐색 요청 ID가 필요합니다");
    }
    const std::string id = request_id.get<std::string>();
    for (const auto& [known, value] : requests_) {
        if (known == id) {
            if (value != "scan") {
                throw ValueError("request_id_conflict");
            }
            return {{"accepted", true},
                    {"duplicate", true},
                    {"reason", "이미 처리한 요청 · 재송신하지 않음"}};
        }
    }
    const auto number = [&body](const char* key, double fallback) {
        if (!body.contains(key) || body.at(key).is_null()) {
            return fallback;
        }
        if (!finite_number(body.at(key))) {
            throw ValueError("탐색 범위·속도·제한 시간 오류");
        }
        return body.at(key).get<double>();
    };
    // 2026-11-18 탐색 방식: 요청의 pattern(turn|sector)이 있으면 그것, 없으면 ARGUS_SCAN_PATTERN
    // 기본값.
    ScanPattern pattern = default_scan_pattern_;
    if (body.contains("pattern") && !body.at("pattern").is_null()) {
        const auto parsed = body.at("pattern").is_string()
                                ? parse_scan_pattern(body.at("pattern").get<std::string>())
                                : std::nullopt;
        if (!parsed) {
            throw ValueError("탐색 방식은 turn 또는 sector");
        }
        pattern = *parsed;
    }
    const bool sector = pattern == ScanPattern::Sector;
    // 2026-11-18 왕복 시작 방향: 요청에 속도가 없으면 ARGUS_SCAN_SECTOR_FIRST(left|right) 방향으로
    // 시작. 요청 속도의 부호는 그대로 따른다. 기존 회전(turn)의 기본 방향은 바꾸지 않는다.
    double rate =
        number("rate_deg_s", sector && sector_first_left_ ? -kScanRateDegS : kScanRateDegS);
    double sweep = number("sweep_deg", sector ? sector_width_deg_ : kScanSweepDeg);
    const double timeout = number("timeout_s", kScanTimeoutS);
    if (sector && sweep >= kScanFullTurnRequestDeg) {
        // 왕복에서 한 바퀴(2π, GCS 기본값) 요청은 왕복 기본 폭으로 바꾼다.
        sweep = sector_width_deg_;
    }
    if (!sector) {
        // 2026-11-19 한 방향 회전은 오른쪽(양수)만, 범위는 180° 이하.
        // 0과 상한 초과 속도는 아래에서 거절한다.
        rate = std::fabs(rate);
        if (sweep > kScanTurnMaxDeg && sweep <= 360) {
            sweep = kScanTurnMaxDeg;
        }
    }
    if (rate == 0 || std::fabs(rate) > kScanMaxRateDegS || sweep <= 0 ||
        sweep > (sector ? kScanSectorMaxDeg : 360) || timeout <= 0 || timeout > kScanMaxTimeoutS) {
        throw ValueError("탐색 범위·속도·제한 시간 오류");
    }
    const std::string reason = ready_reason();
    if (!reason.empty()) {
        throw ValueError(reason);
    }
    if (v_.mode() != std::optional<std::string>("offboard")) {
        throw ValueError("탐색은 Offboard 제자리 유지 중에만 가능합니다");
    }
    if (!v_.command_owner().acquire()) {
        throw ValueError("다른 송신자가 FC를 소유 중입니다");
    }
    const double now = v_.now();
    // 2026-11-19 한 방향 회전은 구간 끝을 전체 범위로 두어 중간 정지 없이 돈다.
    // 마지막 구간으로 보아 끝에서 감속 정지만 한다.
    const double first_step_end = sector ? kScanStepDeg : sweep;
    requests_.emplace_back(id, "scan");
    while (requests_.size() > 256) {
        requests_.erase(requests_.begin());
    }
    state_ = Json{{"phase", "queued"},
                  {"reason", "position_scan_pending"},
                  {"request_id", id},
                  {"rate_deg_s", rate},
                  {"duration_s", timeout},
                  {"sweep_deg", sweep},
                  {"progress_deg", 0.0},
                  {"signed_progress_deg", 0.0},
                  // 돌다 멈추기 상태 (scan_ramp.hpp StepScanState)
                  {"step_end_deg", first_step_end},
                  {"step_start_at", now},
                  {"pause_until", -1.0},
                  {"drift_since", -1.0},
                  {"drift_holds", 0},
                  {"scan", true},
                  {"direct", true},
                  {"epoch", v_.state().pose_epoch},
                  {"phase_at", now},
                  {"deadline", now + .2},
                  {"last_tick", now},
                  {"last_send", -std::numeric_limits<double>::infinity()},
                  {"sent_count", 0},
                  {"acked", false},
                  {"initial_yaw", nullptr},
                  {"last_yaw", sample_field(v_, "ATTITUDE", "yaw")},
                  {"scan_pattern", scan_pattern_name(pattern)},
                  {"path_deg", 0.0}};
    if (sector) {
        json_set(state_, "sector_cycles", kScanSectorCycles);
        json_set(state_, "sector_legs", sector_leg_count(kScanSectorCycles));
        json_set(state_, "sector_leg", 0);
        json_set(state_, "leg_start_deg", 0.0);
        json_set(state_, "leg_rate_deg_s", rate);
        json_set(state_, "sector_continuous", sector_continuous_);
    }
    goal_ = v_.state().local_target;
    v_.record_event("manual_yaw scan requested " + id + " " + scan_pattern_name(pattern) + " " +
                    std::to_string(sweep) + "deg");
    return {{"accepted", true},
            {"reason",
             sector ? "탐색 시작 · Offboard 제자리 좌우 왕복" : "탐색 시작 · Offboard 제자리 회전"},
            {"manual_yaw", status()}};
}

bool ManualYaw::scanning() const {
    return active() && state_.contains("scan") && python_truthy(state_.at("scan"));
}

bool ManualYaw::valid_goal() const {
    const auto& goal = v_.state().local_target;
    if (!goal || !goal->is_object() || !goal->contains("climb_at") ||
        goal->at("climb_at").is_null()) {
        return false;
    }
    const std::array<const char*, 7> keys{"x",        "y",      "start_z", "target_z",
                                          "distance", "height", "yaw"};
    const Json& target = *goal;
    return std::all_of(keys.begin(), keys.end(), [&target](const char* key) {
        return target.contains(key) && finite_number(target.at(key));
    });
}

std::string ManualYaw::direct_reason() {
    const auto& fc = v_.state();
    const double now = v_.now();
    if (fc.pending || !fc.jobs.empty() || fc.landing || state_.at("epoch") != fc.pose_epoch) {
        return "operator_or_epoch_changed";
    }
    if (state_.contains("stop_requested") && python_truthy(state_.at("stop_requested"))) {
        return state_.contains("stop_reason") ? state_.at("stop_reason").get<std::string>()
                                              : "operator_stop";
    }
    if (v_.mode() != std::optional<std::string>("offboard") || !fc.external || !fc.owned_flight ||
        !fc.owned_mode_confirmed || fc.local_target != goal_ || !valid_goal()) {
        return "position_owner_changed";
    }
    std::string reason = fault();
    if (!reason.empty()) {
        return reason;
    }
    if (now < state_number("last_tick") || now - state_number("last_tick") > .2) {
        return "manual_dispatch_gap";
    }
    if (sector_scan()) {
        // 왕복: 모든 구간(+반폭, −반폭, 시작 방향 복귀)을 실제 yaw로 마쳤을 때 끝낸다.
        if (state_.at("sector_leg").get<std::int64_t>() >=
            state_.at("sector_legs").get<std::int64_t>()) {
            return "scan_complete";
        }
    } else if (scanning() && state_number("progress_deg") >= state_number("sweep_deg")) {
        return "scan_complete";
    }
    if (now >= state_number("deadline")) {
        if (state_.at("phase") != "rotating") {
            return "preparation_timeout";
        }
        return scanning() ? "scan_timeout" : "pulse_complete";
    }
    return "";
}

void ManualYaw::end_direct(const std::string& reason) {
    auto& fc = v_.state();
    // 회전 뒤 측정한 마지막 방향을 잡는다. 바뀐 목표나 다른 FC epoch의 목표는 덮어쓰지 않는다.
    if (fc.local_target == goal_ && fc.local_target && state_.at("epoch") == fc.pose_epoch &&
        python_truthy(state_.at("sent_count"))) {
        const Json yaw = sample_field(v_, "ATTITUDE", "yaw");
        if (v_.fresh("ATTITUDE", .2) && finite_number(yaw)) {
            json_set(state_, "last_yaw", yaw);
        }
        json_set(*fc.local_target, "yaw", state_.at("last_yaw"));
        zero_goal_ = fc.local_target;
    }
    finish(reason);
}

std::optional<double> ManualYaw::position_rate() {
    if (active() && state_.contains("direct") && python_truthy(state_.at("direct"))) {
        const std::string reason = direct_reason();
        if (!reason.empty()) {
            end_direct(reason);
        } else {
            const double now = v_.now();
            const Json yaw = sample_field(v_, "ATTITUDE", "yaw");
            if (state_.at("phase") == "queued") {
                json_set(state_, "phase", "rotating");
                json_set(state_, "reason",
                         scanning() ? "position_scan_active" : "position_yaw_active");
                json_set(state_, "phase_at", now);
                json_set(state_, "deadline", now + (scanning() ? state_number("duration_s") : 2.0));
                json_set(state_, "initial_yaw", yaw);
                if (scanning()) {
                    json_set(state_, "step_start_at", now);
                }
            } else if (scanning() && finite_number(yaw) && finite_number(state_.at("last_yaw"))) {
                // 실제 yaw 변화를 감아 누적한다. 시간은 회전 완료의 근거가 아니다.
                const double step = degrees(std::remainder(
                    yaw.get<double>() - state_.at("last_yaw").get<double>(), 2 * M_PI));
                const double signed_progress = state_number("signed_progress_deg") + step;
                const double path = state_number("path_deg") + std::fabs(step);
                json_set(state_, "signed_progress_deg", signed_progress);
                json_set(state_, "path_deg", path);
                // 왕복에서는 실제 돈 총 각(경로)을 진행각으로 알린다 (표적 판정의 "더 돈 각"이
                // 방향이 바뀌어도 줄지 않게). 기존 회전은 그대로 순 회전각의 절댓값이다.
                json_set(state_, "progress_deg", sector_scan() ? path : std::fabs(signed_progress));
            }
            json_set(state_, "last_yaw", yaw);
            json_set(state_, "last_send", now);
            json_set(state_, "sent_count", state_.at("sent_count").get<std::int64_t>() + 1);
            if (scanning()) {
                StepScanState step{state_number("step_end_deg"), state_number("step_start_at"),
                                   state_number("pause_until"), state_number("drift_since")};
                // 위치 유지 목표(goal_)에서 얼마나 밀렸는가. 신선한 위치가 없으면 판단하지 않는다.
                std::optional<ScanDrift> drift;
                const Json* position = v_.sample("LOCAL_POSITION_NED");
                if (position != nullptr && v_.fresh("LOCAL_POSITION_NED", .3) && goal_ &&
                    goal_->contains("x") && goal_->contains("y")) {
                    const Json px = sample_field(v_, "LOCAL_POSITION_NED", "x");
                    const Json py = sample_field(v_, "LOCAL_POSITION_NED", "y");
                    const Json vx = sample_field(v_, "LOCAL_POSITION_NED", "vx");
                    const Json vy = sample_field(v_, "LOCAL_POSITION_NED", "vy");
                    if (finite_number(px) && finite_number(py) && finite_number(vx) &&
                        finite_number(vy) && finite_number(goal_->at("x")) &&
                        finite_number(goal_->at("y"))) {
                        drift =
                            ScanDrift{std::hypot(px.get<double>() - goal_->at("x").get<double>(),
                                                 py.get<double>() - goal_->at("y").get<double>()),
                                      std::hypot(vx.get<double>(), vy.get<double>())};
                    }
                }
                const bool was_holding = step.drift_since_s >= 0;
                double rate = 0;
                if (sector_scan()) {
                    rate = sector_rate(now, state_number("signed_progress_deg"), drift);
                    step =
                        StepScanState{state_number("step_end_deg"), state_number("step_start_at"),
                                      state_number("pause_until"), state_number("drift_since")};
                } else {
                    rate = step_scan_rate(state_number("rate_deg_s"), state_number("sweep_deg"),
                                          now, state_number("progress_deg"), step, drift);
                }
                const bool holding = step.drift_since_s >= 0;
                if (holding && !was_holding) {
                    json_set(state_, "drift_holds",
                             state_.at("drift_holds").get<std::int64_t>() + 1);
                    v_.record_event("manual_yaw scan drift hold " +
                                    std::to_string(drift ? drift->error_m : -1.0) + "m at " +
                                    std::to_string(state_number("progress_deg")) + "deg");
                } else if (was_holding && !holding) {
                    v_.record_event("manual_yaw scan drift resume " +
                                    std::to_string(drift ? drift->error_m : -1.0) + "m");
                }
                json_set(state_, "step_end_deg", step.step_end_deg);
                json_set(state_, "step_start_at", step.step_start_s);
                json_set(state_, "pause_until", step.pause_until_s);
                json_set(state_, "drift_since", step.drift_since_s);
                json_set(state_, "drift_error_m", drift ? Json(drift->error_m) : Json());
                json_set(state_, "commanded_rate_deg_s", rate);
                return radians(rate);
            }
            return radians(state_number("rate_deg_s"));
        }
    }
    if (zero_goal_) {
        const std::shared_ptr<Json> goal = zero_goal_;
        zero_goal_.reset();
        if (v_.state().local_target == goal) {
            return 0.0;
        }
    }
    return std::nullopt;
}

void ManualYaw::send(double rate) {
    // 추종과 같은 BODY_NED 속도·yaw rate 패킷 (절대 yaw는 무시)
    v_.send_position_target(kBodyNed, kVelocityMask,
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, static_cast<float>(radians(rate))});
    json_set(state_, "last_send", v_.now());
    if (rate != 0) {
        json_set(state_, "sent_count", state_.at("sent_count").get<std::int64_t>() + 1);
    }
}

void ManualYaw::mode(const std::string& name) {
    json_set(state_, "acked", false);
    json_set(state_, "sent_at", v_.now());
    json_set(state_, "wanted_mode", name);
    v_.send_mode(name);
}

void ManualYaw::ingest(const std::string& kind, const Json& data) {
    if (active() && state_.contains("direct") && python_truthy(state_.at("direct"))) {
        return;  // 모드 명령을 보내지 않았다
    }
    const auto field = [&data](const char* key, const Json& fallback) {
        return data.contains(key) ? data.at(key) : fallback;
    };
    if (active() && kind == "COMMAND_ACK" && field("command", Json()) == 176) {
        const Json system = field("target_system", 0);
        const Json component = field("target_component", 0);
        if ((system != 0 && system != 245) || (component != 0 && component != 191)) {
            return;
        }
        const Json result = field("result", Json());
        if (result == 0) {
            json_set(state_, "acked", true);
        } else if (result != 5) {
            json_set(state_, "mode_rejected", true);
        }
    }
}

void ManualYaw::finish(const std::string& reason) {
    if (!active()) {
        return;
    }
    last_ = state_;
    json_set(last_, "phase", "finished");
    json_set(last_, "reason", reason);
    const Json yaw = sample_field(v_, "ATTITUDE", "yaw");
    const Json initial = state_.at("initial_yaw");
    if (!initial.is_null() && !yaw.is_null() && v_.fresh("ATTITUDE", .2)) {
        json_set(last_, "observed_yaw_delta_deg",
                 degrees(std::remainder(yaw.get<double>() - initial.get<double>(), 2 * M_PI)));
    }
    state_ = nullptr;
    goal_.reset();
    v_.record_event("manual_yaw finished " + last_.at("request_id").get<std::string>() + " " +
                    reason + " sent=" + std::to_string(last_.at("sent_count").get<std::int64_t>()));
    if (!(v_.state().external || v_.state().owned_flight)) {
        v_.command_owner().release();
    }
}

void ManualYaw::cancel(const std::string& reason) {
    if (active() && state_.contains("direct") && python_truthy(state_.at("direct"))) {
        end_direct(reason);
    } else {
        finish(reason);  // 조종자·RC·epoch 소유 변경: 모드를 덮어쓰지 않는다
    }
}

bool ManualYaw::can_stop() const {
    const auto& fc = v_.state();
    const auto& owner = v_.command_owner();
    return fc.link && fc.uid == v_.expected_uid() &&
           v_.fresh("HEARTBEAT", v_.limits().heartbeat_ttl) && v_.armed() && v_.ground() == false &&
           v_.mode() == std::optional<std::string>("offboard") &&
           (owner.path().empty() || owner.held()) && !owner_.rc_override();
}

void ManualYaw::stop(const std::string& reason) {
    if (!active() || state_.at("phase") == "stopping") {
        return;
    }
    if (state_.at("phase") == "await_hold") {
        finish(reason + "_existing_hold_retained");
        return;
    }
    if (can_stop()) {
        json_set(state_, "phase", "stopping");
        json_set(state_, "reason", reason);
        json_set(state_, "deadline", v_.now() + 3.0);
        send(0);
        mode("hold");
    } else {
        finish(reason);
    }
}

void ManualYaw::retain_offboard_hold(const std::string& reason) {
    // 나가기에 실패해도 FC가 Offboard인 동안 영속도 송신을 끊지 않는다. 기존 감독에 넘긴다.
    if (!can_stop()) {
        finish(reason + "_authority_lost");
        return;
    }
    send(0);
    auto& fc = v_.state();
    fc.local_target.reset();
    fc.external = fc.owned_flight = fc.owned_mode_confirmed = true;
    fc.holding = false;  // 기존 lease·항법·고도 감독은 유지
    fc.last_tx = v_.now();
    finish(reason + "_offboard_hold_retained");
}

void ManualYaw::tick() {
    if (!active()) {
        return;
    }
    if (state_.contains("direct") && python_truthy(state_.at("direct"))) {
        const std::string reason = direct_reason();
        if (!reason.empty()) {
            end_direct(reason);
        } else {
            json_set(state_, "last_tick", v_.now());
        }
        return;
    }
    const double now = v_.now();
    auto& fc = v_.state();
    if (fc.pending || !fc.jobs.empty() || fc.landing || state_.at("epoch") != fc.pose_epoch) {
        cancel("operator_or_epoch_changed");
        return;
    }
    const bool stop_requested =
        state_.contains("stop_requested") && python_truthy(state_.at("stop_requested"));
    state_.erase("stop_requested");
    if (stop_requested && state_.at("phase") != "stopping") {
        stop("operator_stop");
        return;
    }
    const bool rejected =
        state_.contains("mode_rejected") && python_truthy(state_.at("mode_rejected"));
    state_.erase("mode_rejected");
    if (state_.at("phase") == "await_hold" && rejected) {
        stop("mode_request_rejected");
        return;
    }
    const auto sent_at = [this](double fallback) {
        return state_.contains("sent_at") ? state_number("sent_at") : fallback;
    };
    if (state_.at("phase") == "stopping") {
        if (fc.uid == v_.expected_uid() && v_.fresh("HEARTBEAT", v_.limits().heartbeat_ttl) &&
            v_.mode() == std::optional<std::string>("hold") && heartbeat_at(v_) > sent_at(0)) {
            finish(state_.at("reason").get<std::string>());
        } else if (rejected || now >= state_number("deadline")) {
            retain_offboard_hold(rejected ? "hold_rejected" : "hold_unconfirmed");
        } else if (!can_stop()) {
            finish("hold_authority_lost");
        } else if (now - state_number("last_send") >= .05) {
            send(0);
        }
        return;
    }
    if (rejected) {
        stop("mode_request_rejected");
        return;
    }
    const std::string reason = fault();
    if (!reason.empty()) {
        stop(reason);
        return;
    }
    const auto current = v_.mode();
    if (current != std::optional<std::string>("hold") &&
        current != std::optional<std::string>("offboard")) {
        cancel("pilot_mode_departure");
        return;
    }
    if (state_.at("phase") == "rotating" && current != std::optional<std::string>("offboard")) {
        cancel("offboard_departure");
        return;
    }
    if (state_.at("phase") == "priming" && current != std::optional<std::string>("hold")) {
        stop("unexpected_offboard");
        return;
    }
    if (now < state_number("last_tick") || now - state_number("last_tick") > .2) {
        stop("manual_dispatch_gap");
        return;
    }
    json_set(state_, "last_tick", now);
    if (now >= state_number("deadline")) {
        stop(state_.at("phase") == "rotating" ? "pulse_complete" : "preparation_timeout");
        return;
    }
    if (state_.at("phase") == "await_hold" && !state_.contains("wanted_mode")) {
        mode("hold");
    }
    const auto heartbeat = fc.samples.find("HEARTBEAT");
    const bool fresh_mode = (heartbeat == fc.samples.end() ? 0.0 : heartbeat->second.at) >
                            sent_at(-std::numeric_limits<double>::infinity());
    if (state_.at("phase") == "await_hold" || state_.at("phase") == "stopping") {
        if (current == std::optional<std::string>("hold") && fresh_mode) {
            if (state_.at("phase") == "stopping") {
                finish(state_.at("reason").get<std::string>());
                return;
            }
            fc.external = fc.owned_flight = fc.owned_mode_confirmed = fc.holding = false;
            fc.local_target.reset();
            json_set(state_, "phase", "priming");
            json_set(state_, "phase_at", now);
            json_set(state_, "deadline", now + 4.0);
        }
    }
    if (state_.at("phase") == "await_hold") {
        return;  // 원래 제어기만 송신한다
    }
    if (state_.at("phase") == "priming" && now - state_number("phase_at") >= 1.3) {
        json_set(state_, "phase", "await_offboard");
        json_set(state_, "deadline", now + 3.0);
        mode("offboard");
    }
    if (state_.at("phase") == "await_offboard" &&
        current == std::optional<std::string>("offboard") && fresh_mode &&
        python_truthy(state_.at("acked"))) {
        json_set(state_, "phase", "rotating");
        json_set(state_, "phase_at", now);
        json_set(state_, "deadline", now + 2.0);
        json_set(state_, "initial_yaw", sample_field(v_, "ATTITUDE", "yaw"));
    }
    if (now - state_number("last_send") >= .05) {
        send(state_.at("phase") == "rotating" ? state_number("rate_deg_s") : 0.0);
    }
}

Json ManualYaw::status() const {
    const Json& source = active() ? state_ : last_;
    Json visible = Json::object();
    for (const char* key :
         {"phase", "reason", "request_id", "rate_deg_s", "duration_s", "sent_count",
          "observed_yaw_delta_deg", "scan", "sweep_deg", "progress_deg", "scan_pattern",
          "sector_leg", "sector_legs", "leg_rate_deg_s"}) {
        if (source.contains(key)) {
            json_set(visible, key, source.at(key));
        }
    }
    const std::string reason = ready_reason();
    json_set(visible, "supported", true);
    json_set(visible, "active", active());
    json_set(visible, "ready", reason.empty());
    json_set(visible, "ready_reason", reason);
    return visible;
}

}  // namespace argus::gateway
