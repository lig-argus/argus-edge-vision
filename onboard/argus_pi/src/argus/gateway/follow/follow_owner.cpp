// B 추종 송신부 (원본 follow_b_owner.py FollowBOwner).
#include "argus/gateway/follow/include/follow_owner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include "argus/gateway/bridge/include/follow_preferences.hpp"
#include "argus/gateway/common/include/python_compat.hpp"
#include "argus/gateway/follow/include/setpoint_stream_timing.hpp"
#include "argus/gateway/follow/include/tracks_protobuf.hpp"

namespace argus::gateway {

namespace {

// 원본 v._height()[0]: 없으면 None
Json height_value(const FlightLink& vehicle) {
    const auto height = vehicle.height().first;
    return height ? Json(*height) : Json();
}

constexpr std::int64_t kMaxInt = std::numeric_limits<std::int64_t>::max();
constexpr std::uint8_t kBodyNed = 8;
constexpr std::uint16_t kVelocityMask = 1479;
constexpr std::uint16_t kCmdReposition = 192;
// 2026-11-09 사용자 요구: 추종 중 GUI가 끊겨도 자율 추종을 이어 간다. 제안값이다.
// 이 시간을 넘기면 기존처럼 Offboard 고도 유지로 거둔다. 저전압 대응은 FC가 맡는다(사용자 지정).
constexpr double kGuiLossFollowSeconds = 60.0;
// 2026-11-18 데모 위치 고정 (사용자 확정 "위치 고정으로 변경"). 실내 demo는 BODY_NED 속도 대신
// FlightLink::local_setpoint의 yaw rate 경로와 같은 LOCAL_NED 위치 유지 + yaw rate를 보낸다.
// sess264 실측: 속도만 보내면 PX4 위치 제어가 꺼져 42초에 0.29 m(최대 0.42 m) 밀렸다.
constexpr std::uint8_t kFrameLocalNed = 1;
constexpr std::uint16_t kYawIgnore = 1024;
constexpr std::uint16_t kYawRateIgnore = 2048;
constexpr std::uint16_t kPositionMask = 8 | 16 | 32 | 64 | 128 | 256 | 2048;
// 위치 x,y,z와 yaw rate만 쓰고 속도, 가속도, 절대 yaw는 무시한다 (= 1528)
// 위 네 상수는 flight_link.cpp 익명 이름공간의 같은 이름 상수를 옮긴 것이다. 값이 서로 어긋나지
// 않도록 FlightLink::local_setpoint의 yaw rate 마스크 값(1528)을 여기서 고정 확인한다.
constexpr std::uint16_t kDemoHoldMask =
    static_cast<std::uint16_t>((kPositionMask & ~kYawRateIgnore) | kYawIgnore);
static_assert(kFrameLocalNed == 1 && kDemoHoldMask == 1528,
              "데모 위치 유지 마스크는 FlightLink::local_setpoint yaw rate 마스크와 같아야 한다");
// 기준점을 잡거나 밀림을 볼 때 위치 표본 허용 나이 s. 송신마다 보지만 fault()의 0.2 s 규칙
// (fc_position_stale)이 먼저 소유권을 끝내므로 이 값이 추가로 추종을 끊지는 않는다.
constexpr double kDemoPositionTtlS = 0.3;
// z 적분 한 번의 최대 시간 s (송신 간격이 길어도 vz를 그만큼만 적분한다)
constexpr double kDemoMaxStepS = 0.2;
// 기준점에서 이만큼 넘게 밀리면 다시 잡지 않고 안전 정지한다 m (제안값)
constexpr double kDemoAnchorDriftM = 1.0;

// 원본 _integer(d, k, minimum): type is int, minimum <= x <= 2^63-1
std::int64_t integer(const Wire& d, const std::string& key, std::int64_t minimum = 0) {
    const Wire value = wire_get(d, key);
    if (value.is_number_unsigned() &&
        value.get<std::uint64_t>() <= static_cast<std::uint64_t>(kMaxInt) &&
        static_cast<std::int64_t>(value.get<std::uint64_t>()) >= minimum) {
        return static_cast<std::int64_t>(value.get<std::uint64_t>());
    }
    if (value.is_number_integer() && !value.is_number_unsigned() &&
        value.get<std::int64_t>() >= minimum) {
        return value.get<std::int64_t>();
    }
    throw ValueError("invalid_" + key);
}

// d[key] (없으면 원본 KeyError 문구)
const Wire& at_key(const Wire& d, const std::string& key) {
    if (!d.is_object() || !d.contains(key)) {
        throw ValueError("'" + key + "'");
    }
    return d.at(key);
}

// 원본 정수 크기 비교 a < b (int64·uint64 모두 정확하게: 부호 먼저, 같으면 같은 형으로)
bool int_less(const Wire& a, const Wire& b) {
    const auto negative = [](const Wire& value) {
        return !value.is_number_unsigned() && value.get<std::int64_t>() < 0;
    };
    if (negative(a) != negative(b)) {
        return negative(a);
    }
    if (negative(a)) {
        return a.get<std::int64_t>() < b.get<std::int64_t>();
    }
    const auto magnitude = [](const Wire& value) {
        return value.is_number_unsigned() ? value.get<std::uint64_t>()
                                          : static_cast<std::uint64_t>(value.get<std::int64_t>());
    };
    return magnitude(a) < magnitude(b);
}

std::int64_t as_int(const Wire& value) {
    if (value.is_number_unsigned()) {
        return static_cast<std::int64_t>(value.get<std::uint64_t>());
    }
    return value.get<std::int64_t>();
}

bool finite_number(const Wire& value) {
    return value.is_number() && std::isfinite(value.get<double>());
}

// 원본 0 <= now - stamp <= 1.5s
bool status_fresh(std::int64_t now, const Wire& stamp) {
    const std::int64_t age = now - as_int(stamp);
    return 0 <= age && age <= 1'500'000'000;
}

bool all_zero(const Wire& d) {
    const std::array<const char*, 4> keys{"vx_m_s", "vy_m_s", "vz_m_s", "yaw_rate_rad_s"};
    return std::all_of(keys.begin(), keys.end(),
                       [&d](const char* key) { return python_equal(at_key(d, key), Wire(0)); });
}

// 원본 `x in container` (목록·문자열·사전 키)
bool python_in(const Wire& item, const Wire& container) {
    if (container.is_array()) {
        return std::any_of(container.begin(), container.end(),
                           [&item](const Wire& value) { return python_equal(item, value); });
    }
    if (container.is_string()) {
        if (!item.is_string()) {
            throw ValueError("'in <string>' requires string as left operand, not " +
                             python_type_name(item));
        }
        return container.get<std::string>().find(item.get<std::string>()) != std::string::npos;
    }
    if (container.is_object()) {
        return item.is_string() && container.contains(item.get<std::string>());
    }
    throw ValueError("argument of type '" + python_type_name(container) + "' is not iterable");
}

Wire sample_wire(const FlightLink& v, const char* kind) {
    const Json* data = v.sample(kind);
    return data == nullptr ? Wire::object() : to_wire(*data);
}

Wire sample_value(const FlightLink& v, const char* kind, const char* key, const Wire& fallback) {
    const Json* data = v.sample(kind);
    if (data == nullptr || !data->is_object() || !data->contains(key)) {
        return fallback;
    }
    return to_wire(data->at(key));
}

std::int64_t stamp_ns(const FlightLink& v, const char* kind) {
    const auto found = v.state().samples.find(kind);
    return found == v.state().samples.end() ? 0 : static_cast<std::int64_t>(found->second.at * 1e9);
}

std::string upper(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return text;
}

}  // namespace

Wire to_wire(const Json& value) {  // NOLINT(misc-no-recursion) - 상태 JSON 깊이만큼만 내려간다
    switch (value.type()) {
        case Json::value_t::object: {
            // 2026-11-16 perf: 원본 Json 객체는 키가 중복되지 않으므로 wire_set의 키
            // 검색(ordered_map 선형 탐색) 없이 같은 순서로 바로 덧붙인다. 결과(키 순서·값)는 이전과
            // 같다.
            Wire out = Wire::object();
            auto& object = out.get_ref<Wire::object_t&>();
            object.reserve(value.size());
            for (const auto& item : value.items()) {
                object.Container::emplace_back(item.key(), to_wire(item.value()));
            }
            return out;
        }
        case Json::value_t::array: {
            Wire out = Wire::array();
            for (const auto& item : value) {
                out.push_back(to_wire(item));
            }
            return out;
        }
        case Json::value_t::number_float:
            return value.get<double>();
        case Json::value_t::number_unsigned:
            return value.get<std::uint64_t>();
        case Json::value_t::number_integer:
            return value.get<std::int64_t>();
        case Json::value_t::boolean:
            return value.get<bool>();
        case Json::value_t::string:
            return value.get<std::string>();
        default:
            return nullptr;
    }
}

std::string python_type_name(const Wire& value) {
    switch (value.type()) {
        case Wire::value_t::null:
            return "NoneType";
        case Wire::value_t::boolean:
            return "bool";
        case Wire::value_t::number_integer:
        case Wire::value_t::number_unsigned:
            return "int";
        case Wire::value_t::number_float:
            return "float";
        case Wire::value_t::string:
            return "str";
        case Wire::value_t::binary:
            return "bytes";
        case Wire::value_t::array:
            return "list";
        default:
            return "dict";
    }
}

bool FollowBOwner::Context::operator==(const Context& other) const {
    return epoch == other.epoch && generation == other.generation &&
           python_equal(track, other.track);
}

FollowBOwner::FollowBOwner(FlightLink& vehicle, std::unique_ptr<IFollowTransport> transport,
                           const FollowOwnerOptions& options)
    : v_(vehicle),
      transport_(std::move(transport)),
      authority_(options.authority),
      cv_gap_enabled_(options.cv_gap_enabled),
      offboard_hover_enabled_(options.offboard_hover),
      session_(vehicle.new_id()),
      cv_journal_(std::make_unique<CvJournal>(options.journal_directory)),
      manual_yaw_(*this, vehicle),
      epoch_(vehicle.state().pose_epoch) {
    std::string event_error;
    setpoint_event_min_interval_ns_ = setpoint_event_min_interval_from_env(
        std::getenv("ARGUS_GW_SETPOINT_EVENT_MIN_MS"), &event_error);
    if (!event_error.empty()) {
        std::cerr << event_error << '\n';
    }
    std::cerr << "setpoint send: "
              << (setpoint_event_min_interval_ns_ > 0
                      ? "event (new command, min " +
                            std::to_string(setpoint_event_min_interval_ns_ / 1'000'000) +
                            "ms) + 50ms stream"
                      : std::string("50ms stream"))
              << '\n';
}

FollowBOwner::~FollowBOwner() {
    try {
        close();
    } catch (...) {  // NOLINT(bugprone-empty-catch) - 소멸 중에는 닫기 실패를 던지지 않는다
    }
}

void FollowBOwner::close() {
    if (closed_) {
        return;
    }
    closed_ = true;
    manual_yaw_.cancel("owner_closed");
    if (transport_) {
        transport_->close();
    }
    if (cv_journal_) {
        cv_journal_->close();
    }
}

std::int64_t FollowBOwner::now_ns() const {
    return static_cast<std::int64_t>(v_.now() * 1'000'000'000);
}

void FollowBOwner::cv_trace(const std::string& event, const Wire* command,
                            const std::string& reason, const std::string& proof) {
    try {
        const Wire data = command != nullptr && command->is_object() ? *command : Wire::object();
        Wire fields = Wire::object();
        wire_set(fields, "owner_monotonic_ns", now_ns());
        wire_set(fields, "fc_epoch", v_.state().pose_epoch);
        wire_set(fields, "owner_token_present", token_active());
        wire_set(fields, "cv_active", wire_get(data, "cv_gap_active") == true);
        wire_set(fields, "reason", reason);
        wire_set(fields, "proof", proof);
        for (const char* key :
             {"command_epoch", "generation", "track_id", "source_sample_monotonic_ns",
              "cv_origin_sample_ns", "cv_gap_deadline_monotonic_ns", "issued_monotonic_ns",
              "valid_until_monotonic_ns", "seq", "revision"}) {
            if (wire_get(data, key).is_number_integer()) {
                wire_set(fields, key, data.at(key));
            }
        }
        cv_journal_->record(event, fields);
    } catch (...) {  // NOLINT(bugprone-empty-catch) - 진단은 제어 승인·송신을 바꾸지 않는다
    }
}

void FollowBOwner::read_cv_observations() {
    // 원본은 cv_gap_enabled이고 추적 주소가 있을 때만 추적 SUB를 연다.
    if (!cv_gap_enabled_ || !transport_->has_tracks()) {
        return;
    }
    // FC 소유 스레드는 추적을 기다리며 막히지 않는다.
    for (int i = 0; i < 8; ++i) {
        const std::optional<Parts> parts = transport_->track_receive();
        if (!parts) {
            break;
        }
        try {
            // 2026-11-25 batch 2: [tracking/tracks][protobuf] (6375). 되살린 msgpack 은 예전과 같다
            Bytes packed;
            std::string error;
            const TracksFrame received = tracks_msgpack(*parts, packed, error);
            if (received == TracksFrame::NotTracks || received == TracksFrame::MissingPiSchema) {
                continue;
            }
            if (received == TracksFrame::Invalid) {
                throw ValueError(error);
            }
            const Wire frame = msgpack_unpack(packed, true);
            const auto& fc = v_.state();
            const bool verified = fc.uid == v_.expected_uid() && v_.fresh("HEARTBEAT", 1.5);
            cv_observations_.observe(frame, now_ns(), fc.pose_epoch, verified);
        } catch (const ValueError&) {
            continue;
        }
    }
}

std::string FollowBOwner::fault() const {
    if (authority_ != nullptr && authority_->emergency()) {
        return "operator_emergency_land";
    }
    const auto& fc = v_.state();
    if (!fc.link || fc.uid != v_.expected_uid()) {
        return "fc_identity_unverified";
    }
    if (!v_.fresh("HEARTBEAT", 1.5)) {
        return "fc_heartbeat_stale";
    }
    if (!v_.fresh("LOCAL_POSITION_NED", .2)) {
        return "fc_position_stale";
    }
    if (!v_.local_ok()) {
        return "fc_local_invalid";
    }
    if (rc_override_) {
        return "rc_override";
    }
    if (!v_.armed() || v_.ground() != false) {
        return "not_in_air";
    }
    return "";
}

void FollowBOwner::prepare_handoff() {
    // 승인만 한다: 기존 단일 소유자 고도 송신은 멈추지 않는다.
    const auto& fc = v_.state();
    const auto& owner = v_.command_owner();
    if (token_active() || pending_ || bench_ || manual_yaw_.active() || !fault().empty() ||
        !v_.gcs_active() || !fc.owned_mode_confirmed || !fc.external || !fc.owned_flight ||
        !manual_yaw_.valid_goal() || v_.mode() != std::optional<std::string>("offboard") ||
        (!owner.path().empty() && !owner.held())) {
        throw ValueError("기존 Offboard 고도 유지·송신권·안전 상태 확인 필요");
    }
    reset_demo_anchor("handoff_prepared");  // 새 인계는 기준점을 새로 잡는다
    handoff_ = Handoff{fc.local_target, fc.pose_epoch, now_ns() + 15'000'000'000};
}

bool FollowBOwner::handoff_valid() const {
    const auto& fc = v_.state();
    const auto& owner = v_.command_owner();
    return handoff_ && now_ns() < handoff_->until && handoff_->epoch == fc.pose_epoch &&
           handoff_->goal == fc.local_target && fc.external && fc.owned_flight && !fc.pending &&
           fc.jobs.empty() && !fc.landing && fc.owned_mode_confirmed &&
           v_.mode() == std::optional<std::string>("offboard") && manual_yaw_.valid_goal() &&
           fault().empty() && v_.gcs_active() && v_.navigation_reason().empty() &&
           v_.height().first.has_value() && (owner.path().empty() || owner.held());
}

bool FollowBOwner::hover_ready() const {
    const auto& fc = v_.state();
    const auto& owner = v_.command_owner();
    return offboard_hover_enabled_ && fc.holding && fc.external && fc.owned_flight &&
           fc.owned_mode_confirmed && v_.mode() == std::optional<std::string>("offboard") &&
           manual_yaw_.valid_goal() && fault().empty() &&
           (v_.gcs_active() || passive_hover_epoch_ == fc.pose_epoch) && v_.fresh("ATTITUDE", .2) &&
           v_.height().first.has_value() && v_.navigation_reason().empty() &&
           (owner.path().empty() || owner.held());
}

bool FollowBOwner::establish_hover(const std::string& reason, bool connection_lost) {
    // PX4 모드 변경 없이 움직임을 새 LOCAL_NED hover로 거둔다.
    auto& fc = v_.state();
    // GPS 없는 PX4는 HOLD(AUTO_LOITER)를 거절한다. GUI 연결이 끊긴 경우에도 HOLD 대신
    // Offboard 제자리 유지를 쓴다 (2026-11-09 실기: 연결 소실 → HOLD 거절 → failsafe 착륙).
    const bool passive =
        !v_.gcs_active() || (connection_lost && v_.now() - fc.lease_at > v_.limits().lease_ttl);
    std::string failure;
    if (!offboard_hover_enabled_) {
        failure = "hover_disabled";
    } else if (!fault().empty()) {
        failure = "fault";
    } else if (v_.mode() != std::optional<std::string>("offboard")) {
        failure = "not_offboard";
    } else if (!(token_active() || (fc.external && fc.owned_flight))) {
        failure = "no_follow_or_owned_flight";
    } else if (!v_.fresh("ATTITUDE", .2)) {
        failure = "attitude_stale";
    } else if (!v_.height().first) {
        failure = "height_invalid";
    } else if (const std::string navigation = v_.navigation_reason(); !navigation.empty()) {
        failure = "navigation:" + navigation;
    } else if (!v_.command_owner().acquire()) {
        failure = "command_owner_busy";
    }
    if (!failure.empty()) {
        last_hover_failure_ = failure;
        v_.record_event("Offboard 제자리 유지 불가 · " + reason + " · " + failure);
        return false;
    }
    const Wire position = sample_wire(v_, "LOCAL_POSITION_NED");
    const Wire attitude = sample_wire(v_, "ATTITUDE");
    for (const Wire& value : {wire_get(position, "x"), wire_get(position, "y"),
                              wire_get(position, "z"), wire_get(attitude, "yaw")}) {
        if (!finite_number(value)) {
            return false;
        }
    }
    // 인계가 준비 단계뿐이면 이전 고도를 유지한다.
    std::shared_ptr<Json> goal;
    if (handoff_pending_ && handoff_valid()) {
        goal = fc.local_target;
    } else {
        const Json* p = v_.sample("LOCAL_POSITION_NED");
        const Json* a = v_.sample("ATTITUDE");
        goal = std::make_shared<Json>(Json{{"x", p->at("x")},
                                           {"y", p->at("y")},
                                           {"start_z", p->at("z")},
                                           {"target_z", p->at("z")},
                                           {"distance", 0.0},
                                           {"height", height_value(v_)},
                                           {"yaw", a->at("yaw")},
                                           {"climb_at", v_.now()}});
    }
    const auto old_target = fc.local_target;
    const bool old_external = fc.external;
    const bool old_owned = fc.owned_flight;
    const bool old_confirmed = fc.owned_mode_confirmed;
    const bool old_holding = fc.holding;
    manual_yaw_.cancel(reason);
    fc.local_target = goal;
    try {
        v_.local_setpoint();
    } catch (...) {
        fc.local_target = old_target;
        fc.external = old_external;
        fc.owned_flight = old_owned;
        fc.owned_mode_confirmed = old_confirmed;
        fc.holding = old_holding;
        throw;
    }
    setpoint_.reset();
    prestream_at_.reset();
    last_stream_.reset();
    handoff_pending_ = false;
    handoff_.reset();
    reset_demo_anchor(reason);  // 고도 유지 목표가 송신을 넘겨받는다
    local_hover_active_ = true;
    passive_hover_epoch_ = passive ? std::optional<int>(fc.pose_epoch) : std::nullopt;
    fc.external = fc.owned_flight = fc.owned_mode_confirmed = fc.holding = true;
    v_.record_event("Offboard 고도 유지 · " + reason);
    return true;
}

void FollowBOwner::invalidate(const std::string& reason) {
    cv_trace("owner_invalidate", setpoint_ ? &*setpoint_ : nullptr, reason);
    cv_observations_.clear();
    reset_demo_anchor(reason);
    const bool preserve_hover = (handoff_pending_ && v_.state().external) || hover_ready();
    handoff_pending_ = false;
    local_hover_active_ = false;
    if (!preserve_hover) {
        passive_hover_epoch_.reset();
    }
    if (reason != "released") {
        handoff_.reset();
    }
    manual_yaw_.cancel(reason);
    if (token_active() && reason != "released") {
        mode_event(Wire::object(), "MODE_EXIT", reason);
    }
    last_reason_ = reason;
    token_.reset();
    client_.reset();
    command_context_.reset();
    last_real_command_source_.reset();
    real_command_sources_.clear();
    offboard_seen_ = false;
    setpoint_.reset();
    pending_.reset();
    prestream_at_.reset();
    last_stream_.reset();
    last_follow_status_.reset();
    follow_status_seq_ = Wire(-1);
    if (!preserve_hover) {
        v_.command_owner().release();
    }
}

bool FollowBOwner::bench_safe() const {
    const auto& fc = v_.state();
    return fc.link && fc.uid == v_.expected_uid() && v_.fresh("HEARTBEAT", 1.5) &&
           v_.fresh("EXTENDED_SYS_STATE", .5) && v_.ground() == true &&
           v_.mode() != std::optional<std::string>("offboard");
}

Json FollowBOwner::bench_request(const Json& body) {
    // 지상 전송 시험: B 비행 명령 규약 밖이다.
    const Wire data = to_wire(body);
    if (!python_equal(wire_get(data, "expected_uid"), Wire(v_.expected_uid()))) {
        throw ValueError("FC UID 불일치");
    }
    if (wire_get(data, "action") == "bench_stop") {
        bench_.reset();
        v_.command_owner().release();
        return {{"accepted", true}, {"pending", false}, {"reason", "bench_stopped"}};
    }
    const Wire rate = wire_get(data, "yaw_rate_deg_s");
    const Wire duration = wire_get(data, "duration_s");
    if (wire_get(data, "props_removed") != true) {
        throw ValueError("프로펠러 제거 확인 필요");
    }
    if (!finite_number(rate) || std::abs(rate.get<double>()) > 10) {
        throw ValueError("yaw rate ±10 deg/s");
    }
    const bool duration_ok =
        finite_number(duration) && 0 < duration.get<double>() && duration.get<double>() <= 5;
    if (!duration_ok) {
        throw ValueError("duration 0..5s");
    }
    const auto& fc = v_.state();
    if (!bench_safe() || token_active() || bench_ || manual_yaw_.active() || fc.external ||
        fc.pending || !fc.jobs.empty()) {
        throw ValueError("신선한 지상·idle 송신권 확인 필요");
    }
    if (!v_.command_owner().acquire()) {
        throw ValueError("another_setpoint_writer_owns_fc");
    }
    bench_ = Bench{now_ns() + static_cast<std::int64_t>(duration.get<double>() * 1e9),
                   rate.get<double>() * (M_PI / 180.0), 0};
    return {{"accepted", true},
            {"pending", true},
            {"transport", "gateway_b_ground_test"},
            {"fc_received", false}};
}

void FollowBOwner::ingest(const std::string& kind, const Json& data) {
    manual_yaw_.ingest(kind, data);
    if (kind == "STATUSTEXT" && data.contains("text") && data.at("text").is_string() &&
        data.at("text").get<std::string>().find("Pilot took over") != std::string::npos) {
        rc_override_ = true;
        invalidate("rc_override");
    }
    if (kind == "COMMAND_ACK" && pending_) {
        const auto field = [&data](const char* key, const Json& fallback) {
            return data.contains(key) ? data.at(key) : fallback;
        };
        const Json system = field("target_system", 0);
        const Json component = field("target_component", 0);
        if (field("command", Json()) == 176 && (system == 0 || system == 245) &&
            (component == 0 || component == 191)) {
            const Json result = field("result", Json());
            if (result == 0) {
                pending_->acked = true;
            } else if (result != 5) {
                wire_set((*pending_->reply), "status", "failed");
                wire_set((*pending_->reply), "reason", "mode_rejected");
                // ACK 진단만 붙인다. 승인·결과·흐름은 바꾸지 않는다.
                Wire diagnostics = Wire::object();
                if (result.is_number_integer()) {
                    wire_set(diagnostics, "mav_result", to_wire(result));
                }
                const Json param2 = field("result_param2", Json());
                if (param2.is_number_integer()) {
                    wire_set(diagnostics, "mav_result_param2", to_wire(param2));
                }
                for (const auto& item : diagnostics.items()) {
                    wire_set((*pending_->reply), item.key(), item.value());
                }
                mode_event(pending_->context, "MODE_FAILED", "mode_rejected", diagnostics);
                pending_.reset();
            }
        }
    }
}

std::int64_t FollowBOwner::validate(const Wire& d, bool velocity) {
    const std::int64_t now = now_ns();
    if (wire_get(d, "schema") != (velocity ? "argus.control.setpoint.v1" : "argus.follow.rpc.v1")) {
        throw ValueError("schema");
    }
    if (wire_get(d, "source") != "argus-follow-controller" ||
        wire_get(d, "clock_domain") != "host_monotonic") {
        throw ValueError("source_or_clock");
    }
    if (!python_equal(wire_get(d, "uid"), Wire(v_.expected_uid())) ||
        !python_equal(wire_get(d, "onboard_session"), Wire(session_)) ||
        !python_equal(wire_get(d, "fc_epoch"), Wire(v_.state().pose_epoch))) {
        throw ValueError("fc_context");
    }
    const Wire client = wire_get(d, "client_session");
    if (!client.is_string() || client.get_ref<const std::string&>().empty()) {
        throw ValueError("client_session");
    }
    const Wire request_id = wire_get(d, "request_id");
    if (!request_id.is_string() || request_id.get_ref<const std::string&>().empty()) {
        throw ValueError("request_id");
    }
    if (d.contains("command_id") && !python_equal(d.at("command_id"), request_id)) {
        throw ValueError("command_id_alias");
    }
    if (d.contains("session_id") && !python_equal(d.at("session_id"), client)) {
        throw ValueError("session_id_alias");
    }
    for (const char* key : {"fc_epoch", "seq", "revision", "command_epoch", "generation",
                            "source_sample_monotonic_ns", "issued_monotonic_ns"}) {
        integer(d, key);
    }
    if (!wire_get(d, "track_id").is_number_integer()) {
        throw ValueError("track_id");
    }
    const Wire bench = wire_get(d, "bench");
    if (!bench.is_boolean() || bench.get<bool>()) {
        throw ValueError("bench_not_supported");
    }
    const std::int64_t issued = as_int(d.at("issued_monotonic_ns"));
    const bool fresh = 0 <= now - issued && now - issued <= 250'000'000;
    if (!fresh) {
        throw ValueError("request_stale");
    }
    const Wire action = wire_get(d, "action");
    if (action != "acquire" && action != "release" && action != "revoke" &&
        action != "request_status") {
        const std::int64_t until = integer(d, "valid_until_monotonic_ns");
        const bool window_ok = issued < until && until <= issued + 250'000'000;
        if (!window_ok || now >= until) {
            throw ValueError("deadline");
        }
    }
    return now;
}

void FollowBOwner::authority(const Wire& d) const {
    if (!token_active() || !token_ || !python_equal(wire_get(d, "owner_token"), Wire(*token_)) ||
        !python_equal(at_key(d, "client_session"), client_ ? Wire(*client_) : Wire())) {
        throw ValueError("owner_token");
    }
    if (as_int(at_key(d, "revision")) < revision_) {
        throw ValueError("revision");
    }
}

FollowBOwner::CacheEntry* FollowBOwner::cached(const std::string& client, const Wire& request_id) {
    if (!request_id.is_string()) {
        return nullptr;
    }
    for (auto& entry : cache_) {
        if (entry.client == client && entry.request_id == request_id.get<std::string>()) {
            return &entry;
        }
    }
    return nullptr;
}

Wire FollowBOwner::request(const Wire& d) {
    if (!d.is_object()) {
        throw ValueError("'" + python_type_name(d) + "' object has no attribute 'get'");
    }
    auto reply = std::make_shared<Wire>(Wire::object());
    wire_set((*reply), "request_id", wire_get(d, "request_id"));
    wire_set((*reply), "client_session", wire_get(d, "client_session"));
    wire_set((*reply), "schema", "argus.follow.rpc.v1");
    wire_set((*reply), "onboard_session", session_);
    wire_set((*reply), "fc_epoch", v_.state().pose_epoch);
    wire_set((*reply), "status", "failed");
    try {
        const std::int64_t now = validate(d, false);
        const Bytes fingerprint = msgpack_pack(d);
        const std::string client = d.at("client_session").get<std::string>();
        const std::string request_id = d.at("request_id").get<std::string>();
        const Wire action_value = wire_get(d, "action");
        if (const CacheEntry* entry = cached(client, d.at("request_id"))) {
            if (entry->fingerprint != fingerprint) {
                throw ValueError("request_id_conflict");
            }
            if (action_value == "acquire" && !python_equal(wire_get(*entry->reply, "owner_token"),
                                                           token_ ? Wire(*token_) : Wire())) {
                throw ValueError("owner_revoked");
            }
            if (action_value != "acquire" && action_value != "release") {
                authority(d);
            }
            return *entry->reply;
        }
        const Wire action_wire = at_key(d, "action");
        const std::string action = action_wire.is_string() ? action_wire.get<std::string>() : "";
        auto& fc = v_.state();
        if (action == "acquire") {
            if (manual_yaw_.active()) {
                throw ValueError("manual_yaw_owns_fc");
            }
            const std::string why = fault();
            if (!why.empty()) {
                throw ValueError(why);
            }
            const bool direct = handoff_valid();
            const auto current = v_.mode();
            if (token_active() || fc.pending || !fc.jobs.empty() ||
                (!direct && (fc.external || (current != std::optional<std::string>("hold") &&
                                             current != std::optional<std::string>("position"))))) {
                throw ValueError("handoff_not_ready");
            }
            if (!v_.command_owner().acquire()) {
                throw ValueError("another_setpoint_writer_owns_fc");
            }
            local_hover_active_ = false;
            handoff_pending_ = direct;
            reset_demo_anchor("new_acquire");  // 소유권마다 기준점과 데모 판정을 새로 한다
            token_ = v_.new_id();
            client_ = client;
            last_mode_event_.reset();
            last_follow_status_ = now;
            follow_status_seq_ = Wire(-1);
            revision_ = as_int(d.at("revision"));
            seq_ = as_int(d.at("seq"));
            command_context_.reset();
            offboard_seen_ = false;
            rc_override_ = false;
            wire_set((*reply), "status", "accepted");
            wire_set((*reply), "owner_token", *token_);
            wire_set((*reply), "capabilities", Wire::array({"context_v1", "absolute_heading_rad"}));
        } else {
            authority(d);
            if (as_int(d.at("seq")) <= seq_) {
                throw ValueError("sequence");
            }
            seq_ = as_int(d.at("seq"));
            if (action == "release") {
                invalidate("released");
            } else if (action == "revoke") {
                revision_ = std::max(revision_, integer(d, "next_revision"));
                setpoint_.reset();
                if (pending_ && d.contains("cancel_request_ids") &&
                    python_in(pending_->request_id, d.at("cancel_request_ids"))) {
                    wire_set((*pending_->reply), "status", "failed");
                    wire_set((*pending_->reply), "reason", "revoked");
                    pending_.reset();
                }
            } else if (action == "request_status") {
                const Wire original = wire_get(d, "original_request_id");
                if (original.is_array() || original.is_object()) {
                    throw ValueError("unhashable type: '" + python_type_name(original) + "'");
                }
                const CacheEntry* old = client_ ? cached(*client_, original) : nullptr;
                if (old == nullptr) {
                    throw ValueError("request_unknown");
                }
                wire_set((*reply), "status", old->reply->at("status"));
                wire_set((*reply), "reason",
                         old->reply->contains("reason") ? old->reply->at("reason") : Wire(""));
                for (const char* key : {"mav_result", "mav_result_param2"}) {
                    if (wire_get(*old->reply, key).is_number_integer()) {
                        wire_set((*reply), key, old->reply->at(key));
                    }
                }
            } else if (action == "hold" || action == "start_offboard") {
                motion_valid(d, now, action == "hold");
                if (action == "hold" && offboard_hover_enabled_ &&
                    v_.mode() == std::optional<std::string>("offboard")) {
                    if (!establish_hover("follow_stop", false)) {
                        // 제자리 유지가 불가하면 HOLD가 아니라 같은 안전 경로(착륙)로 간다.
                        const std::string failure = last_hover_failure_;
                        safety_stop("follow_stop_hover_failed");
                        throw ValueError("offboard_hover_precondition_failed:" + failure);
                    }
                    mode_event(d, "MODE_COMPLETED", "offboard_hover_confirmed");
                    wire_set((*reply), "status", "accepted");
                    cache_.push_back({client, request_id, fingerprint, reply});
                    return *reply;
                }
                if (handoff_pending_) {
                    // 준비 실패로 GUI 고도 송신을 바꾸지 않는다.
                    invalidate("handoff_preparation_cancelled");
                    throw ValueError("existing_offboard_hover_preserved");
                }
                if (pending_ || fc.pending) {
                    throw ValueError("mode_busy");
                }
                if (action == "start_offboard" &&
                    (!prestream_at_ || now - *prestream_at_ < 1'000'000'000 ||
                     now - last_stream_.value_or(0) > 200'000'000)) {
                    throw ValueError("prestream_not_ready");
                }
                if (action == "hold") {
                    // Hold RPC가 앞선 PUB 영속도를 앞지를 수 있다. 이전 움직임은 여기서 거둔다.
                    setpoint_.reset();
                }
                v_.send_mode(action == "start_offboard" ? "offboard" : "hold");
                wire_set((*reply), "status", "pending");
                // 실행 기한은 송신 전에 검사했다. 수락한 모드의 확인 대기는 별도다.
                const std::int64_t completion = d.contains("mode")
                                                    ? now + 2'000'000'000
                                                    : as_int(d.at("valid_until_monotonic_ns"));
                pending_ = Pending{d.at("request_id"),
                                   reply,
                                   d,
                                   action == "start_offboard" ? "offboard" : "hold",
                                   false,
                                   now,
                                   completion};
            } else if (action == "heading") {
                motion_valid(d, now, wire_get(d, "stopping") == true);
                const Wire yaw = d.contains("yaw_rad") ? d.at("yaw_rad") : Wire(std::nan(""));
                if (v_.mode() != std::optional<std::string>("hold")) {
                    throw ValueError("heading_requires_hold");
                }
                if (!yaw.is_number() && !yaw.is_boolean()) {
                    throw ValueError("must be real number, not " + python_type_name(yaw));
                }
                double yaw_rad = 0.0;
                if (yaw.is_boolean()) {
                    yaw_rad = yaw.get<bool>() ? 1.0 : 0.0;
                } else {
                    yaw_rad = yaw.get<double>();
                }
                if (!std::isfinite(yaw_rad)) {
                    throw ValueError("heading_requires_hold");
                }
                // SDK 백엔드와 같은 PX4 DO_REPOSITION 절대 rad 규약
                const double nan = std::numeric_limits<double>::quiet_NaN();
                v_.send_command(kCmdReposition, {-1, 1, 0, yaw_rad, nan, nan, nan});
            } else {
                throw ValueError("unknown_action");
            }
        }
        if (reply->at("status") != "pending") {
            wire_set((*reply), "status", "accepted");
        }
        cache_.push_back({client, request_id, fingerprint, reply});
        while (cache_.size() > 256) {
            cache_.erase(cache_.begin());
        }
    } catch (const ValueError& error) {
        wire_set((*reply), "status", "failed");
        wire_set((*reply), "reason", error.what());
    }
    return *reply;
}

void FollowBOwner::motion_valid(const Wire& d, std::int64_t now, bool safety) {
    const std::string why = fault();
    if (!why.empty()) {
        throw ValueError(why);
    }
    const auto current = v_.mode();
    if (current != std::optional<std::string>("hold") &&
        current != std::optional<std::string>("position") &&
        current != std::optional<std::string>("offboard")) {
        throw ValueError("mode_not_allowed");
    }
    if (python_equal(at_key(d, "command_epoch"), Wire(0))) {
        throw ValueError("command_epoch");
    }
    const Context context{as_int(at_key(d, "command_epoch")), as_int(at_key(d, "generation")),
                          at_key(d, "track_id")};
    if (command_context_) {
        if (context.epoch < command_context_->epoch ||
            (context.epoch == command_context_->epoch && !(context == *command_context_))) {
            throw ValueError("command_context");
        }
    }
    command_context_ = context;
    const std::int64_t age = now - as_int(at_key(d, "source_sample_monotonic_ns"));
    const Wire cv = d.contains("cv_gap_active") ? d.at("cv_gap_active") : Wire(false);
    if (!cv.is_boolean()) {
        throw ValueError("cv_gap_active");
    }
    if (cv.get<bool>()) {
        if (!cv_gap_enabled_) {
            throw ValueError("cv_gap_disabled");
        }
        const std::int64_t origin = integer(d, "cv_origin_sample_ns");
        const std::int64_t deadline = integer(d, "cv_gap_deadline_monotonic_ns");
        read_cv_observations();
        const int epoch = v_.state().pose_epoch;
        const bool prior_command = std::any_of(
            real_command_sources_.begin(), real_command_sources_.end(),
            [&](const auto& item) { return item.first == context && item.second == origin; });
        const bool established =
            last_real_command_source_ && last_real_command_source_->first == context;
        const auto observed_identity = cv_observations_.identity(
            epoch, d.at("generation"), d.at("track_id"), Wire(origin), now);
        bool observed = false;
        if (established && observed_identity) {
            for (const auto& [candidate, stamp] : real_command_sources_) {
                if (candidate == context &&
                    cv_observations_.identity(epoch, d.at("generation"), d.at("track_id"),
                                              Wire(stamp), now) == observed_identity) {
                    observed = true;
                    break;
                }
            }
        }
        if (origin != as_int(d.at("source_sample_monotonic_ns")) || !(prior_command || observed) ||
            deadline != origin + 3'000'000'000 || age < 0 || now >= deadline ||
            as_int(at_key(d, "valid_until_monotonic_ns")) > deadline) {
            throw ValueError("cv_gap_provenance_or_deadline");
        }
    } else if (age < 0 || (!safety && age > 250'000'000)) {
        throw ValueError("source_stale");
    }
}

bool FollowBOwner::receive_setpoint(const Wire& data) {
    if (!data.is_object()) {
        last_reason_ = "invalid_setpoint_wire";  // 원본: dict(d) 실패 뒤 d.get에서 AttributeError
        return false;
    }
    Wire d = data;
    try {
        for (const auto& [external, internal] :
             std::initializer_list<std::pair<const char*, const char*>>{
                 {"forward", "vx_m_s"},
                 {"right", "vy_m_s"},
                 {"down", "vz_m_s"},
                 {"yaw_rate", "yaw_rate_rad_s"}}) {
            if (d.contains(external)) {
                const Wire value = d.at(external);
                if (!finite_number(value)) {
                    throw ValueError("setpoint_alias");
                }
                if (d.contains(internal) && !python_equal(d.at(internal), value)) {
                    throw ValueError("setpoint_alias_conflict");
                }
                wire_set(d, internal, value);
            }
        }
        const std::int64_t now = validate(d, true);
        authority(d);
        if (local_hover_active_) {
            throw ValueError("hover_requires_new_acquire");
        }
        if (at_key(d, "action") != "velocity" || wire_get(d, "coordinate_frame") != "body_frd") {
            throw ValueError("frame");
        }
        for (const char* key : {"vx_m_s", "vy_m_s", "vz_m_s", "yaw_rate_rad_s"}) {
            if (!finite_number(wire_get(d, key))) {
                throw ValueError("nonfinite");
            }
        }
        if (as_int(d.at("seq")) <= seq_) {
            throw ValueError("sequence");
        }
        const bool zero = all_zero(d);
        if (!zero && v_.mode() != std::optional<std::string>("offboard")) {
            throw ValueError("nonzero_requires_offboard");
        }
        motion_valid(d, now, zero);
        const std::int64_t source = as_int(d.at("source_sample_monotonic_ns"));
        if (!wire_truthy(d.contains("cv_gap_active") ? d.at("cv_gap_active") : Wire(false)) &&
            0 <= now - source && now - source <= 250'000'000) {
            const Context context{as_int(d.at("command_epoch")), as_int(d.at("generation")),
                                  d.at("track_id")};
            if (last_real_command_source_ && !(last_real_command_source_->first == context)) {
                real_command_sources_.clear();
            }
            last_real_command_source_ = std::make_pair(context, source);
            const bool present = std::any_of(
                real_command_sources_.begin(), real_command_sources_.end(),
                [&](const auto& item) { return item.first == context && item.second == source; });
            if (!present) {
                real_command_sources_.push_back(*last_real_command_source_);
            }
            while (real_command_sources_.size() > 128) {
                real_command_sources_.erase(real_command_sources_.begin());
            }
        }
        seq_ = as_int(d.at("seq"));
        revision_ = std::max(revision_, as_int(d.at("revision")));
        setpoint_ = d;
        setpoint_fresh_ = true;  // 2026-11-16 이벤트 송신: 아직 FC로 안 보낸 새 명령
        if (wire_get(d, "cv_gap_active") == true) {
            const std::int64_t origin = as_int(d.at("cv_origin_sample_ns"));
            const bool history =
                command_context_ && std::any_of(real_command_sources_.begin(),
                                                real_command_sources_.end(), [&](const auto& item) {
                                                    return item.first == *command_context_ &&
                                                           item.second == origin;
                                                });
            cv_trace("cv_setpoint_accepted", &d, "",
                     history ? "accepted_command_history" : "real_tracking_observation");
        }
        return true;
    } catch (const ValueError& error) {
        last_reason_ = error.what();
        if (wire_get(d, "cv_gap_active") == true) {
            cv_trace("cv_setpoint_rejected", &d, error.what());
        }
        return false;
    }
}

Wire FollowBOwner::mode_request(const Wire& wire) {
    // PR26 수락 응답과 실제 모드 완료 이벤트를 분리한다.
    if (!wire.is_object()) {
        throw ValueError("'" + python_type_name(wire) + "' object has no attribute 'get'");
    }
    Wire response = Wire::object();
    wire_set(response, "schema", "argus.control.mode_result.v1");
    wire_set(response, "source", "argus-onboard");
    wire_set(response, "command_id", wire_get(wire, "command_id"));
    wire_set(response, "session_id", session_);
    wire_set(response, "requester_session_id", wire_get(wire, "session_id"));
    wire_set(response, "client_session", wire_get(wire, "session_id"));
    wire_set(response, "request_id", wire_get(wire, "command_id"));
    wire_set(response, "onboard_session", session_);
    wire_set(response, "uid", v_.expected_uid());
    wire_set(response, "fc_epoch", v_.state().pose_epoch);
    wire_set(response, "result", "REJECTED");
    wire_set(response, "reason", "");
    try {
        if (wire_get(wire, "schema") != "argus.control.mode_request.v1") {
            throw ValueError("mode_schema");
        }
        const Wire mode = wire_get(wire, "mode");
        std::string action;
        if (mode == "OFFBOARD") {
            action = "start_offboard";
        } else if (mode == "HOLD") {
            action = "hold";
        } else if (mode == "HOLD_HEADING") {
            action = "heading";
        } else {
            throw ValueError("mode");
        }
        const Wire command = wire_get(wire, "command_id");
        const Wire session = wire_get(wire, "session_id");
        if (!python_equal(wire.contains("request_id") ? wire.at("request_id") : command, command)) {
            throw ValueError("command_id_alias");
        }
        if (!python_equal(wire.contains("client_session") ? wire.at("client_session") : session,
                          session)) {
            throw ValueError("session_id_alias");
        }
        if (!python_equal(wire.contains("action") ? wire.at("action") : Wire(action),
                          Wire(action))) {
            throw ValueError("mode_action_conflict");
        }
        static const std::vector<std::string> kKnown{"schema",
                                                     "source",
                                                     "request_id",
                                                     "client_session",
                                                     "command_id",
                                                     "session_id",
                                                     "action",
                                                     "uid",
                                                     "clock_domain",
                                                     "onboard_session",
                                                     "fc_epoch",
                                                     "owner_token",
                                                     "command_epoch",
                                                     "generation",
                                                     "track_id",
                                                     "source_sample_monotonic_ns",
                                                     "bench",
                                                     "issued_monotonic_ns",
                                                     "seq",
                                                     "revision",
                                                     "valid_until_monotonic_ns",
                                                     "mode",
                                                     "yaw_rad",
                                                     "stopping"};
        Wire request = Wire::object();
        for (const auto& item : wire.items()) {
            if (std::find(kKnown.begin(), kKnown.end(), item.key()) != kKnown.end()) {
                wire_set(request, item.key(), item.value());
            }
        }
        wire_set(request, "schema", "argus.follow.rpc.v1");
        wire_set(request, "action", action);
        wire_set(request, "request_id", command);
        wire_set(request, "client_session", session);
        const Wire reply = this->request(request);
        wire_set(response, "result",
                 (reply.at("status") == "pending" || reply.at("status") == "accepted")
                     ? "ACCEPTED"
                     : "REJECTED");
        wire_set(response, "reason",
                 reply.contains("reason") ? reply.at("reason") : Wire("request_admitted"));
    } catch (const ValueError& error) {
        wire_set(response, "reason", error.what());
    } catch (const std::runtime_error& error) {
        wire_set(response, "result", "FAILED");
        wire_set(response, "reason", std::string("mode_transport_error: ") + error.what());
    }
    pub_seq_ += 1;
    wire_set(response, "seq", pub_seq_);
    wire_set(response, "clock_domain", "host_monotonic");
    wire_set(response, "published_monotonic_ns", now_ns());
    return response;
}

void FollowBOwner::publish_message(const std::string& topic, Wire data) {
    publish_message_in_place(topic, data);
}

void FollowBOwner::publish_message_in_place(const std::string& topic, Wire& data) {
    pub_seq_ += 1;
    wire_set(data, "source", "argus-onboard");
    wire_set(data, "seq", pub_seq_);
    wire_set(data, "session_id", session_);
    transport_->publish({Bytes(topic.begin(), topic.end()), msgpack_pack(data)});
}

void FollowBOwner::mode_event(const Wire& context, const std::string& event,
                              const std::string& reason, const Wire& diagnostics) {
    const auto current = v_.mode();
    if (!current) {
        throw std::runtime_error(
            "'NoneType' object has no attribute 'upper'");  // 원본 AttributeError
    }
    Wire value = Wire::object();
    wire_set(value, "schema", "argus.fc.mode_event.v1");
    wire_set(value, "uid", v_.expected_uid());
    wire_set(value, "onboard_session", session_);
    wire_set(value, "requester_session_id", client_ ? Wire(*client_) : Wire(""));
    wire_set(value, "fc_epoch", v_.state().pose_epoch);
    wire_set(value, "clock_domain", "host_monotonic");
    wire_set(value, "received_monotonic_ns", now_ns());
    wire_set(value, "command_id",
             context.contains("request_id") ? context.at("request_id") : Wire(""));
    wire_set(value, "event", event);
    wire_set(value, "actual_mode", upper(*current));
    wire_set(value, "owner_token", token_ ? Wire(*token_) : Wire(""));
    wire_set(value, "revision",
             context.contains("revision") ? context.at("revision") : Wire(revision_));
    wire_set(value, "command_epoch",
             context.contains("command_epoch") ? context.at("command_epoch") : Wire(0));
    wire_set(value, "reason", reason);
    if (diagnostics.is_object()) {
        for (const auto& item : diagnostics.items()) {
            wire_set(value, item.key(), item.value());  // 진단은 FC 모드 처리를 바꾸지 않는다
        }
    }
    last_mode_event_ = value;
    publish_message("fc/mode_event", value);
    last_event_publish_ = now_ns();
}

bool FollowBOwner::receive_status(const Wire& data) {
    // 진단 생존 확인만 갱신한다. setpoint/heading 기한과 송신권은 연장하지 않는다.
    if (!data.is_object()) {
        last_reason_ = "invalid_setpoint_wire";
        return false;
    }
    const std::int64_t now = now_ns();
    const Wire stamp = wire_get(data, "received_monotonic_ns");
    const Wire seq = wire_get(data, "seq");
    const Wire client = client_ ? Wire(*client_) : Wire();
    const Wire epoch = wire_get(data, "fc_epoch");
    if (!token_active() || wire_get(data, "schema") != "argus.follow.status.v1" ||
        wire_get(data, "source") != "argus-follow-controller" ||
        wire_get(data, "clock_domain") != "host_monotonic" ||
        !python_equal(wire_get(data, "uid"), Wire(v_.expected_uid())) ||
        !python_equal(wire_get(data, "client_session"), client) ||
        !python_equal(data.contains("session_id") ? data.at("session_id") : client, client) ||
        !python_equal(wire_get(data, "onboard_session"), Wire(session_)) ||
        !epoch.is_number_integer() || !python_equal(epoch, Wire(v_.state().pose_epoch)) ||
        !stamp.is_number_integer() || !status_fresh(now, stamp) || !seq.is_number_integer() ||
        !int_less(follow_status_seq_, seq)) {
        return false;
    }
    last_follow_status_ = as_int(stamp);
    follow_status_seq_ = seq;
    return true;
}

void FollowBOwner::zero_body_setpoint(double yaw_rate) {
    v_.send_position_target(kBodyNed, kVelocityMask,
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, static_cast<float>(yaw_rate)});
}

void FollowBOwner::reset_demo_anchor(const std::string& reason) {
    if (demo_anchor_) {
        v_.record_event("데모 위치 고정 해제 (" + reason + ")");
    }
    demo_anchor_.reset();
    demo_mode_.reset();
}

std::optional<bool> FollowBOwner::demo_send_decision() {
    // 첫 송신에서 정한 방식을 소유권 동안 유지한다. 상태를 잠깐 모르게 돼도(비어 있음) 바꾸지
    // 않는다. 확인된 값이 정한 방식과 다르면 어느 쪽으로든 위치 송신과 속도 송신을 섞지 않고
    // 거부한다.
    const std::optional<bool> demo = v_.demo_follow();
    if (demo_mode_) {
        if (demo && *demo != *demo_mode_) {
            reset_demo_anchor(*demo ? "demo_profile_started" : "demo_profile_inactive");
            throw ValueError("demo_profile_changed");
        }
        return demo_mode_;
    }
    if (demo) {
        return demo;
    }
    // 인계 대기 중 모르면 이번 송신을 미룬다. 송신부의 고도 유지 목표가 계속 나가고, 인계는
    // 확정하지 않는다 (인계 준비가 만료되면 기존 경로로 무효가 된다). 그 밖은 기존 속도 송신이다.
    if (handoff_pending_) {
        return std::nullopt;
    }
    return false;
}

void FollowBOwner::send_demo_hold(const Wire& d, std::int64_t now) {
    // 데모는 회전 추종만 한다. 수평 이동 명령은 버리지 않고 거부해 기존 안전 정지 경로로 보낸다.
    if (d.at("vx_m_s").get<double>() != 0 || d.at("vy_m_s").get<double>() != 0) {
        throw ValueError("demo_translation_rejected");
    }
    const auto& fc = v_.state();
    // EKF 재설정으로 local 좌표가 바뀌면 예전 기준점을 쫓지 않는다.
    if (demo_anchor_ && demo_anchor_->epoch != fc.pose_epoch) {
        reset_demo_anchor("fc_epoch_changed");
        throw ValueError("demo_anchor_epoch_changed");
    }
    if (!v_.fresh("LOCAL_POSITION_NED", kDemoPositionTtlS)) {
        throw ValueError("demo_anchor_position_stale");
    }
    const Wire position = sample_wire(v_, "LOCAL_POSITION_NED");
    const Wire px = wire_get(position, "x");
    const Wire py = wire_get(position, "y");
    const Wire pz = wire_get(position, "z");
    if (!finite_number(px) || !finite_number(py) || !finite_number(pz)) {
        throw ValueError("demo_anchor_position_invalid");
    }
    if (!demo_anchor_) {
        DemoAnchor anchor{px.get<double>(), py.get<double>(), pz.get<double>(), fc.pose_epoch,
                          std::nullopt};
        // 인계 확정 전이면 기존 고도 유지 목표 자리(x, y, 현재 z 목표)를 그대로 잇는다.
        const auto& goal = fc.local_target;
        if (handoff_pending_ && goal && goal->is_object() && goal->contains("x") &&
            goal->contains("y") && goal->at("x").is_number() && goal->at("y").is_number()) {
            anchor.x = goal->at("x").get<double>();
            anchor.y = goal->at("y").get<double>();
            anchor.z = v_.ramped_z();
        }
        if (!std::isfinite(anchor.x) || !std::isfinite(anchor.y) || !std::isfinite(anchor.z)) {
            throw ValueError("demo_anchor_invalid");
        }
        demo_anchor_ = anchor;
        v_.record_event("데모 위치 고정 x=" + python_format_fixed(anchor.x, 2) +
                        " y=" + python_format_fixed(anchor.y, 2) +
                        " z=" + python_format_fixed(anchor.z, 2));
    }
    DemoAnchor& anchor = *demo_anchor_;
    // 기준점에서 크게 밀렸으면 조용히 다시 잡지 않는다.
    if (std::hypot(px.get<double>() - anchor.x, py.get<double>() - anchor.y) > kDemoAnchorDriftM) {
        throw ValueError("demo_anchor_drift");
    }
    // z 목표: 제어기의 vz(1.5 m 고도 유지)를 송신 간격만큼 적분하고, 데모 이륙 범위와 같은
    // 지면 기준 높이(FlightLink::height)로 1.0~2.0 m 안에 자른다. 높이를 모르면 그대로 둔다.
    double z = anchor.z;
    const std::optional<double> height = v_.height().first;
    if (height) {
        const double dt =
            anchor.last_send
                ? std::clamp(static_cast<double>(now - *anchor.last_send) / 1e9, 0.0, kDemoMaxStepS)
                : 0.0;
        z += d.at("vz_m_s").get<double>() * dt;
        const double ground_z = pz.get<double>() + *height;  // NED: 아래가 +
        z = std::clamp(z, ground_z - kDemoMaxHeightM, ground_z - kDemoMinHeightM);
        anchor.height_unknown_logged = false;
    } else if (!anchor.height_unknown_logged) {
        // 높이를 모르면 z 목표를 그대로 둔다 (적분, 범위 자르기 모두 안 함). 한 번 남긴다.
        anchor.height_unknown_logged = true;
        v_.record_event("데모 고도 유지 멈춤 (지면 높이 모름, z 목표 유지)");
    }
    v_.send_position_target(
        kFrameLocalNed, kDemoHoldMask,
        {static_cast<float>(anchor.x), static_cast<float>(anchor.y), static_cast<float>(z), 0, 0, 0,
         0, 0, 0, 0, static_cast<float>(d.at("yaw_rate_rad_s").get<double>())});
    anchor.z = z;
    anchor.last_send = now;
}

Wire FollowBOwner::demo_hold_status() const {
    Wire status = Wire::object();
    wire_set(status, "demo_position_hold",
             demo_mode_ == std::optional<bool>(true) && demo_anchor_.has_value());
    wire_set(status, "anchor_x", demo_anchor_ ? Wire(demo_anchor_->x) : Wire());
    wire_set(status, "anchor_y", demo_anchor_ ? Wire(demo_anchor_->y) : Wire());
    wire_set(status, "anchor_z", demo_anchor_ ? Wire(demo_anchor_->z) : Wire());
    wire_set(status, "anchor_epoch", demo_anchor_ ? Wire(demo_anchor_->epoch) : Wire());
    return status;
}

void FollowBOwner::safety_stop(const std::string& reason) {
    // RC/모드 인계 뒤에는 영속도/Hold도 덮어쓰지 않는다. 검사 뒤 소유권을 폐기한다.
    if (offboard_hover_enabled_ && establish_hover(reason, reason == "gui_link_expired")) {
        invalidate(reason);
        return;
    }
    if (offboard_hover_enabled_ && reason == "gui_link_expired") {
        if (token_active() && fault().empty() &&
            v_.mode() == std::optional<std::string>("offboard")) {
            v_.state().jobs.push_front(std::make_shared<Json>(
                Json{{"id", v_.new_id()},
                     {"action", "land"},
                     {"status", "queued"},
                     {"message", "GUI 연결 만료 · 안전 고도 유지 조건 미충족"}}));
        }
        invalidate(reason);
        return;
    }
    if (handoff_pending_) {
        invalidate(reason);
        return;
    }
    const auto& owner = v_.command_owner();
    if (token_active() && (owner.path().empty() || owner.held()) && fault().empty() &&
        v_.mode() == std::optional<std::string>("offboard")) {
        // 제자리 유지가 불가하면 HOLD(무GPS에서 거절됨)가 아니라 착륙한다.
        zero_body_setpoint(0);
        v_.state().jobs.push_front(std::make_shared<Json>(
            Json{{"id", v_.new_id()},
                 {"action", "land"},
                 {"status", "queued"},
                 {"message", "추종 중지 · 제자리 유지 조건 미충족 (" + reason + ")"}}));
    }
    invalidate(reason);
}

Wire FollowBOwner::common_wire() const {
    const auto& fc = v_.state();
    Wire common = Wire::object();
    wire_set(common, "uid", fc.uid && !fc.uid->empty() ? *fc.uid : v_.expected_uid());
    wire_set(common, "onboard_session", session_);
    wire_set(common, "fc_epoch", fc.pose_epoch);
    wire_set(common, "clock_domain", "host_monotonic");
    return common;
}

Wire FollowBOwner::attitude_wire(const Wire& common) const {
    return attitude_wire(common, v_.sample("ATTITUDE"), stamp_ns(v_, "ATTITUDE"));
}

Wire FollowBOwner::attitude_wire(const Wire& common, const Json* data, std::int64_t stamp) {
    const Wire attitude = data == nullptr ? Wire::object() : to_wire(*data);
    Wire attitude_message = common;
    wire_set(attitude_message, "schema", "argus.fc.attitude.v1");
    wire_set(attitude_message, "received_monotonic_ns", stamp);
    for (const char* key : {"roll", "pitch", "yaw"}) {
        wire_set(attitude_message, std::string(key) + "_rad",
                 attitude.contains(key) ? attitude.at(key) : Wire(0.0));
    }
    wire_set(attitude_message, "fc_time_boot_ms", wire_get(attitude, "time_boot_ms"));
    wire_set(attitude_message, "fc_time_monotonic_ns", nullptr);
    wire_set(attitude_message, "timesync_valid", false);
    for (const char* key : {"roll", "pitch", "yaw"}) {
        wire_set(attitude_message, std::string(key) + "_rad_s",
                 wire_get(attitude, std::string(key) + "speed"));
    }
    return attitude_message;
}

// 2026-11-09 실기: 30ms 주기 송신이 가끔 60ms 넘게 비어, 추적기의 자세 보간(앞뒤 표본 50ms)이
// 프레임 뒤쪽 표본을 받지 못하고 거부했다. 새 ATTITUDE는 받은 루프에서 바로 보낸다.
// 내용·수신 시각은 30ms 송신과 같고, 같은 표본을 두 번 새로 보내지 않는다.
// 2026-11-24 사용자 지시: 100 Hz ATTITUDE에서 10 ms 루프 한 번에 표본이 2개 겹치면 앞의 것이
// fc/attitude로 나가지 않았다(지상 실측 100.8 Hz 수신 중 약 88 Hz만 송신). 마지막 송신 뒤 이력에
// 들어온 표본을 수신 순서대로 모두 보낸다. 가장 새 표본은 같은 루프의 마지막에 나가므로 지연은
// 그대로다. 처음(마지막 송신 없음)에는 지난 이력을 몰아 보내지 않는다.
std::size_t FollowBOwner::publish_attitudes_since_last(const Wire& common) {
    constexpr std::size_t kMaxBurst = 16;
    const auto& fc = v_.state();
    const auto found = fc.pose_history.find("ATTITUDE");
    if (found == fc.pose_history.end() || last_attitude_stamp_ <= 0) {
        return 0;
    }
    const auto& rows = found->second;
    const auto stamp_of = [](const Json& row) {
        return row.at("received_monotonic_ns").get<std::int64_t>();
    };
    std::size_t first = rows.size();
    while (first > 0 && rows.size() - first < kMaxBurst &&
           stamp_of(rows.at(first - 1)) > last_attitude_stamp_) {
        --first;
    }
    std::size_t sent = 0;
    for (std::size_t i = first; i < rows.size(); ++i) {
        const Json& row = rows.at(i);
        const std::int64_t stamp = stamp_of(row);
        publish_message("fc/attitude", attitude_wire(common, &row.at("data"), stamp));
        last_attitude_stamp_ = stamp;
        ++sent;
    }
    return sent;
}

void FollowBOwner::publish_new_attitude() {
    const std::int64_t stamp = stamp_ns(v_, "ATTITUDE");
    if (stamp <= 0 || stamp <= last_attitude_stamp_) {
        return;
    }
    const Wire common = common_wire();
    publish_attitudes_since_last(common);
    // 이력에 없는 최신 표본(첫 송신, 유한하지 않은 값)은 기존처럼 최신 표본으로 보낸다
    if (stamp > last_attitude_stamp_) {
        last_attitude_stamp_ = stamp;
        publish_message("fc/attitude", attitude_wire(common));
    }
}

void FollowBOwner::publish() {
    const auto& fc = v_.state();
    const Wire common = common_wire();
    const auto current = v_.mode();
    Wire state = common;
    wire_set(state, "schema", "argus.fc.state.v1");
    wire_set(state, "received_monotonic_ns", stamp_ns(v_, "LOCAL_POSITION_NED"));
    wire_set(state, "fc_link_connected",
             fc.link && fc.uid == v_.expected_uid() && v_.fresh("HEARTBEAT", 2.));
    wire_set(state, "connected",
             fc.link && fc.uid == v_.expected_uid() && v_.fresh("HEARTBEAT", 1.5));
    wire_set(state, "armed", v_.armed());
    wire_set(state, "in_air", v_.ground() == false);
    wire_set(state, "offboard", current == std::optional<std::string>("offboard"));
    wire_set(state, "hold", current == std::optional<std::string>("hold"));
    wire_set(state, "position", current == std::optional<std::string>("position"));
    wire_set(state, "rc_override", rc_override_);
    wire_set(state, "offboard_hover", hover_ready());
    wire_set(state, "ground_known", v_.ground().has_value());
    wire_set(state, "landed", v_.ground() == true);
    wire_set(state, "local_ok", v_.local_ok());
    wire_set(state, "flow_valid", v_.flow_ok());
    wire_set(state, "gps_valid", v_.global_ok());
    const auto where = v_.environment();
    wire_set(state, "flight_environment", where ? *where : std::string("unknown"));
    std::map<std::string, std::vector<std::optional<std::pair<std::int64_t, std::int64_t>>>>
        history_keys;  // 2026-11-16 perf: 보낸 뒤 행을 저장소로 되돌릴 열쇠
    wire_set(state, "flow_quality", sample_value(v_, "OPTICAL_FLOW_RAD", "quality", Wire(0)));
    wire_set(state, "position_monotonic_ns", stamp_ns(v_, "LOCAL_POSITION_NED"));
    wire_set(state, "distance_monotonic_ns", stamp_ns(v_, "DISTANCE_SENSOR"));
    wire_set(state, "fault_reason", last_reason_);
    const Wire position = sample_wire(v_, "LOCAL_POSITION_NED");
    const Wire distance = sample_wire(v_, "DISTANCE_SENSOR");
    wire_set(state, "position_time_boot_ms", wire_get(position, "time_boot_ms"));
    wire_set(state, "distance_time_boot_ms", wire_get(distance, "time_boot_ms"));
    if (where == std::optional<std::string>("indoor") ||
        where == std::optional<std::string>("outdoor")) {
        // 약 33Hz 중계에서도 원래 고속 표본을 보존한다. 이 묶음 반복은 원래 수신 시각을 갱신하지
        // 않는다.
        Wire history = Wire::object();
        // 2026-11-16 perf: 이미 변환한 행은 다시 변환하지도 복사하지도 않는다. 저장해 둔 행을
        // 상태로 옮겨 보내고, 보낸 뒤 다시 저장소로 옮겨 온다 (내용·순서·바이트는 이전과 같다).
        if (history_cache_epoch_ != fc.pose_epoch) {
            history_cache_.clear();
            history_cache_epoch_ = fc.pose_epoch;
        }
        for (const auto& [kind, rows] : fc.pose_history) {
            Wire list = Wire::array();
            auto& cache = history_cache_[kind];
            auto& keys = history_keys[kind];
            const std::size_t first = rows.size() > 16 ? rows.size() - 16 : 0;
            for (std::size_t i = first; i < rows.size(); ++i) {
                const Json& row = rows.at(i);
                const Json* stamp = row.is_object() && row.contains("received_monotonic_ns")
                                        ? &row.at("received_monotonic_ns")
                                        : nullptr;
                const Json* data =
                    row.is_object() && row.contains("data") ? &row.at("data") : nullptr;
                const bool has_boot =
                    data != nullptr && data->is_object() && data->contains("time_boot_ms");
                const Json* boot = has_boot ? &data->at("time_boot_ms") : nullptr;
                if (stamp == nullptr || !stamp->is_number_integer() ||
                    (boot != nullptr && !boot->is_number())) {
                    list.push_back(to_wire(row));  // 예상 밖 모양은 재사용하지 않는다
                    keys.emplace_back(std::nullopt);
                    continue;
                }
                const std::pair<std::int64_t, std::int64_t> key{
                    stamp->get<std::int64_t>(), boot != nullptr ? boot->get<std::int64_t>() : -1};
                const auto found = cache.find(key);
                if (found != cache.end()) {
                    list.push_back(std::move(found->second));
                    cache.erase(found);
                } else {
                    list.push_back(to_wire(row));
                }
                keys.emplace_back(key);
            }
            wire_set(history, kind, std::move(list));
        }
        wire_set(state, "cmc_pose_history", std::move(history));
    }
    for (const auto& [out, raw] :
         std::initializer_list<std::pair<const char*, const char*>>{{"north_m", "x"},
                                                                    {"east_m", "y"},
                                                                    {"down_m", "z"},
                                                                    {"vn_m_s", "vx"},
                                                                    {"ve_m_s", "vy"},
                                                                    {"vd_m_s", "vz"}}) {
        wire_set(state, out, position.contains(raw) ? position.at(raw) : Wire(0.0));
    }
    for (const auto& [out, raw] : std::initializer_list<std::pair<const char*, const char*>>{
             {"distance_cm", "current_distance"},
             {"min_distance_cm", "min_distance"},
             {"max_distance_cm", "max_distance"},
             {"distance_orientation", "orientation"}}) {
        wire_set(state, out, distance.contains(raw) ? distance.at(raw) : Wire(0));
    }
    for (const auto& [out, raw] : std::initializer_list<std::pair<const char*, const char*>>{
             {"distance_m", "distance_cm"},
             {"min_distance_m", "min_distance_cm"},
             {"max_distance_m", "max_distance_cm"}}) {
        const Wire value = state.at(raw);
        wire_set(state, out, value.is_number() ? Wire(value.get<double>() / 100.) : Wire());
    }
    publish_message_in_place("fc/state", state);
    // 2026-11-16 perf: 보낸 행을 다음 발행에 쓰도록 저장소로 되돌린다 (복사 없이 이동).
    if (!history_keys.empty() && state.contains("cmc_pose_history")) {
        Wire& sent = state.at("cmc_pose_history");
        for (auto& [kind, keys] : history_keys) {
            auto& cache = history_cache_[kind];
            cache.clear();
            if (!sent.contains(kind)) {
                continue;
            }
            Wire& rows = sent.at(kind);
            for (std::size_t i = 0; i < keys.size() && i < rows.size(); ++i) {
                if (keys[i]) {
                    cache.emplace(*keys[i], std::move(rows.at(i)));
                }
            }
        }
    }
    // 2026-11-24: 30 ms 주기 송신 전에도 아직 안 보낸 표본을 먼저 차례로 보낸다
    // 2026-11-25 리뷰: 묶음의 마지막 표본과 같은 시각은 같은 루프에서 다시 보내지 않는다.
    // 새 표본이 없는 30 ms 주기 재송신과 이력에 없는 최신 표본 송신은 그대로 둔다
    const std::size_t burst = publish_attitudes_since_last(common);
    const std::int64_t latest = stamp_ns(v_, "ATTITUDE");
    if (burst == 0 || latest > last_attitude_stamp_) {
        publish_message("fc/attitude", attitude_wire(common));
    }
    last_attitude_stamp_ = std::max(last_attitude_stamp_, latest);
}

void FollowBOwner::tick() {
    read_cv_observations();
    std::int64_t now = now_ns();
    manual_yaw_.tick();
    if (!token_active()) {
        last_reason_ = fault();
    }
    if (bench_) {
        if (!bench_safe() || now >= bench_->until) {
            if (bench_safe()) {
                zero_body_setpoint(0);
            }
            bench_.reset();
            v_.command_owner().release();
        } else if (now - bench_->last >= 40'000'000) {
            zero_body_setpoint(bench_->rate);
            bench_->last = now;
        }
    }
    if (epoch_ != v_.state().pose_epoch) {
        invalidate("fc_epoch_changed");
        epoch_ = v_.state().pose_epoch;
    }
    // 2026-11-06 실기: RC 인계 표시가 착륙·DISARM 뒤에도 남아 재이륙·추종 시작이 막혔다.
    // 신선한 FC 신원으로 지상·DISARM이 확인되고 진행 중인 송신이 없으면 표시를 내린다.
    if (rc_override_ && !token_active() && !pending_ && !bench_ && !manual_yaw_.active() &&
        v_.state().link && v_.state().uid == v_.expected_uid() && v_.fresh("HEARTBEAT", 1.5) &&
        !v_.armed() && v_.ground() == true) {
        rc_override_ = false;
    }
    const auto is_offboard = [this] { return v_.mode() == std::optional<std::string>("offboard"); };
    if (token_active()) {
        if (offboard_seen_ && !is_offboard() && !(pending_ && pending_->mode == "hold")) {
            rc_override_ = true;
            invalidate("offboard_departure_requires_explicit_handoff");
        } else if (is_offboard()) {
            offboard_seen_ = true;
        }
    }
    if (token_active()) {
        const std::string why = fault();
        const auto current = v_.mode();
        if (!why.empty() || (current != std::optional<std::string>("hold") &&
                             current != std::optional<std::string>("position") && !is_offboard())) {
            invalidate(!why.empty() ? why : "mode_departure");
        }
    }
    if (pending_) {
        if (pending_->mode == "hold" && v_.mode() == std::optional<std::string>("hold")) {
            // 실제 Hold가 ACK보다 먼저 올 수 있다. 영속도 중계는 멈추되 같은 요청은
            // ACK·기한·개입까지 둔다.
            setpoint_.reset();
        }
        const auto heartbeat = v_.state().samples.find("HEARTBEAT");
        const auto mode_at = static_cast<std::int64_t>(
            (heartbeat == v_.state().samples.end() ? 0.0 : heartbeat->second.at) * 1e9);
        if (pending_->acked && mode_at > pending_->sent_at &&
            v_.mode() == std::optional<std::string>(pending_->mode)) {
            if (pending_->mode == "hold") {
                offboard_seen_ = false;
            }
            mode_event(pending_->context, "MODE_COMPLETED", "actual_mode_confirmed");
            wire_set((*pending_->reply), "status", "accepted");
            pending_.reset();
        } else if (now >= pending_->deadline) {
            mode_event(pending_->context, "MODE_FAILED", "mode_timeout");
            wire_set((*pending_->reply), "status", "failed");
            wire_set((*pending_->reply), "reason", "mode_timeout");
            pending_.reset();
        }
    }
    for (int i = 0; i < 16; ++i) {
        const std::optional<Parts> parts = transport_->rpc_receive();
        if (!parts) {
            break;
        }
        // 2026-11-25 batch 7: 6371 은 팀 [control/mode_request][protobuf] 하나다 (예전 한 프레임
        // msgpack RPC 와 control/mode_result 응답 이름은 없앴다). 전송이 본문을 msgpack 값으로
        // 되살려 [control/mode_request][msgpack] 으로 넘긴다. 모드 요청과 제어권 RPC 는 값의 schema
        // 로 나눈다 (argus.control.mode_request.v1 이면 mode_request, 아니면 request 가 예전처럼
        // argus.follow.rpc.v1 인지 본다). 응답 첫 프레임은 늘 요청 이름이다
        static const std::string kModeRequest = "control/mode_request";
        Wire reply;
        try {
            if (parts->size() != 2 ||
                std::string(parts->at(0).begin(), parts->at(0).end()) != kModeRequest) {
                throw ValueError("rpc_multipart_shape");
            }
            const Wire wire = msgpack_unpack(parts->at(1));
            if (wire.is_object() && wire.contains("schema") &&
                wire.at("schema") == "argus.control.mode_request.v1") {
                reply = mode_request(wire);
            } else {
                reply = request(wire);
            }
        } catch (const std::exception& error) {
            reply = Wire::object();
            wire_set(reply, "schema", "argus.follow.rpc.v1");
            wire_set(reply, "status", "failed");
            wire_set(reply, "reason", error.what());
        }
        transport_->rpc_send(
            {Bytes(kModeRequest.begin(), kModeRequest.end()), msgpack_pack(reply)});
    }
    for (int i = 0; i < 32; ++i) {
        const std::optional<Parts> parts = transport_->command_receive();
        if (!parts) {
            break;
        }
        if (parts->size() == 2) {
            try {
                const Wire data = msgpack_unpack(parts->at(1));
                const std::string topic(parts->at(0).begin(), parts->at(0).end());
                if (topic == "follow/status") {
                    receive_status(data);
                } else if (topic == "control/setpoint") {
                    receive_setpoint(data);
                }
            } catch (const std::exception&) {
                last_reason_ = "invalid_setpoint_wire";
            }
        }
    }
    // RPC/SUB 처리 중 tick 시작 뒤의 표본을 받았을 수 있다. 실제 송신 시각에 다시 검사한다.
    now = now_ns();
    auto& fc = v_.state();
    if (offboard_hover_enabled_ && token_active() && !local_hover_active_ && !v_.gcs_active() &&
        v_.now() - fc.lease_at > v_.limits().lease_ttl) {
        // 2026-11-09: GUI가 끊겨도 추종기가 보내는 추종은 바로 거두지 않는다. FC 상태가 정상이면
        // kGuiLossFollowSeconds까지 이어 가고, 그 밖에는 기존처럼 고도 유지한다.
        const bool started = !gui_loss_since_;
        if (started) {
            gui_loss_since_ = v_.now();
        }
        if (!fault().empty() || v_.now() - *gui_loss_since_ > kGuiLossFollowSeconds) {
            gui_loss_since_.reset();
            safety_stop("gui_link_expired");
        } else if (started) {
            v_.record_event("GUI 연결 끊김 · 자율 추종 유지 (최대 60초)");
        }
    } else {
        gui_loss_since_.reset();
    }
    if (token_active() && !local_hover_active_ && last_follow_status_ &&
        now - *last_follow_status_ >= 3'000'000'000) {
        safety_stop("follow_status_timeout");
    }
    if (handoff_pending_ && !handoff_valid()) {
        invalidate("handoff_preparation_expired_or_changed");
    }
    if (setpoint_) {
        const Wire d = *setpoint_;
        try {
            authority(d);
            motion_valid(d, now, all_zero(d));
            if (!all_zero(d) && v_.mode() != std::optional<std::string>("offboard")) {
                throw ValueError("nonzero_requires_offboard");
            }
            if (now >= as_int(d.at("valid_until_monotonic_ns"))) {
                throw ValueError("setpoint_expired");
            }
            if (setpoint_stream_due(last_stream_, now, setpoint_fresh_,
                                    setpoint_event_min_interval_ns_)) {
                // 2026-11-18 데모 판정을 모르는 인계 대기 송신은 미룬다 (다음 tick에 다시 본다).
                const std::optional<bool> demo_send = demo_send_decision();
                if (demo_send) {
                    if (!last_stream_ || now - *last_stream_ > 200'000'000) {
                        prestream_at_ = now;
                    }
                    if (handoff_pending_ && !handoff_valid()) {
                        throw ValueError("handoff_precondition_changed");
                    }
                    if (*demo_send) {
                        // 2026-11-18 실내 demo: LOCAL_NED 위치 유지 + yaw rate (영명령도 같은 경로)
                        send_demo_hold(d, now);
                    } else {
                        v_.send_position_target(
                            kBodyNed, kVelocityMask,
                            {0, 0, 0, static_cast<float>(d.at("vx_m_s").get<double>()),
                             static_cast<float>(d.at("vy_m_s").get<double>()),
                             static_cast<float>(d.at("vz_m_s").get<double>()), 0, 0, 0, 0,
                             static_cast<float>(d.at("yaw_rate_rad_s").get<double>())});
                    }
                    demo_mode_ = *demo_send;  // 첫 송신이 성공한 뒤 이 소유권의 방식으로 정한다
                    if (wire_get(d, "cv_gap_active") == true) {
                        cv_trace("cv_mavlink_send", &d);
                    }
                    if (handoff_pending_) {
                        // 같은 소유 스레드에서 송신이 성공한 뒤에만 확정한다.
                        fc.external = fc.owned_flight = false;
                        fc.local_target.reset();
                        handoff_pending_ = false;
                        handoff_.reset();
                        offboard_seen_ = true;
                        v_.record_event("Offboard 추종 인계 완료 · 모드 전환 없음");
                    }
                    last_stream_ = now;
                    setpoint_fresh_ = false;
                }
            }
        } catch (const ValueError& error) {
            cv_trace("setpoint_egress_rejected", &d, error.what());
            setpoint_.reset();
            last_reason_ = error.what();
            // 오래된 속도를 다시 보내지 않는다. 권한이 있을 때만 Hold를 한 번 요청한다.
            if (token_active()) {
                safety_stop(error.what());
            }
        }
    }
    if (now - last_publish_ >= 30'000'000) {
        last_publish_ = now;
        publish();
    } else {
        publish_new_attitude();
    }
    // 소켓 재연결로 이벤트 하나를 놓쳐도 복구되게 짧게 반복한다. 원래 완료 시각을 지키며
    // 새 요청·명령·lease를 만들지 않는다.
    if (last_mode_event_ && now - last_event_publish_ >= 100'000'000) {
        const std::int64_t received = as_int(last_mode_event_->at("received_monotonic_ns"));
        if (0 <= now - received && now - received < 2'000'000'000) {
            publish_message("fc/mode_event", *last_mode_event_);
            last_event_publish_ = now;
        }
    }
}

Wire FollowBOwner::debug_state() const {
    Wire state = Wire::object();
    wire_set(state, "token", token_ ? Wire(*token_) : Wire());
    wire_set(state, "client", client_ ? Wire(*client_) : Wire());
    wire_set(state, "revision", revision_);
    wire_set(state, "seq", seq_);
    wire_set(state, "last_reason", last_reason_);
    wire_set(state, "rc_override", rc_override_);
    wire_set(state, "local_hover_active", local_hover_active_);
    wire_set(state, "handoff_pending", handoff_pending_);
    wire_set(state, "handoff", handoff_.has_value());
    wire_set(state, "offboard_seen", offboard_seen_);
    wire_set(state, "setpoint", setpoint_.has_value());
    wire_set(state, "pending", pending_ ? Wire(pending_->mode) : Wire());
    wire_set(state, "cache", cache_.size());
    wire_set(state, "prestream_at", prestream_at_ ? Wire(*prestream_at_) : Wire());
    wire_set(state, "last_stream", last_stream_ ? Wire(*last_stream_) : Wire());
    wire_set(state, "setpoint_event_min_ms", Wire(setpoint_event_min_interval_ns_ / 1'000'000));
    wire_set(state, "real_sources", real_command_sources_.size());
    wire_set(state, "bench", bench_.has_value());
    wire_set(state, "passive_hover_epoch",
             passive_hover_epoch_ ? Wire(*passive_hover_epoch_) : Wire());
    wire_set(state, "follow_status_seq", follow_status_seq_);
    wire_set(state, "ledger", cv_observations_.debug_state());
    return state;
}

}  // namespace argus::gateway
