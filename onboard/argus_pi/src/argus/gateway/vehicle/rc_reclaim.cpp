#include "argus/gateway/vehicle/include/rc_reclaim.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace argus::gateway {
namespace {
using namespace std::chrono_literals;
constexpr double kRcRetrySpacingS = .25;
constexpr double kRcRetryPerNameS = 2.0;

const std::vector<std::string>& rc_parameters() {
    static const auto kValues = [] {
        std::vector<std::string> result{"RC_MAP_ROLL", "RC_MAP_PITCH", "RC_MAP_YAW",
                                        "RC_MAP_THROTTLE"};
        for (int channel = 1; channel <= 18; ++channel) {
            for (const char* suffix : {"MIN", "MAX", "TRIM", "DZ", "REV"}) {
                result.push_back("RC" + std::to_string(channel) + "_" + suffix);
            }
        }
        return result;
    }();
    return kValues;
}

bool true_field(const Json& data, const char* key) {
    const auto value = data.find(key);
    return value != data.end() && *value == true;
}

Json field_or_null(const Json& data, const char* key) {
    if (!data.is_object()) {
        return nullptr;
    }
    const auto value = data.find(key);
    return value == data.end() ? Json() : *value;
}

// Python str(value)
std::string python_text(const Json& value) {
    if (value.is_string()) {
        return value.get<std::string>();
    }
    if (value.is_null()) {
        return "None";
    }
    if (value.is_boolean()) {
        return value.get<bool>() ? "True" : "False";
    }
    if (value.is_number_float()) {
        return python_float_repr(value.get<double>());
    }
    return value.dump();
}
}  // namespace

// 원본 main_track_adapter.completion: 원래 명령을 다시 보내지 않고 최종 결과만 읽는다.
StopCompletion stop_completion(const Json& status, const std::string& command_id,
                               const Json& controller_session, const Json& onboard_session,
                               const Json& fc_epoch) {
    if (!status.is_object() || field_or_null(status, "accepted") != true ||
        field_or_null(status, "controller_session") != controller_session ||
        field_or_null(status, "onboard_session") != onboard_session ||
        field_or_null(status, "fc_epoch") != fc_epoch) {
        throw ValueError("추종 제어 세션 변경 또는 응답 미확인");
    }
    const Json rows = field_or_null(status, "command_results");
    if (!rows.is_array()) {
        throw ValueError("명령 결과 목록 미확인");
    }
    const Json* match = nullptr;
    int matches = 0;
    for (const auto& row : rows) {
        if (row.is_object() && field_or_null(row, "command_id") == Json(command_id)) {
            match = &row;
            ++matches;
        }
    }
    if (matches != 1) {
        throw ValueError("명령의 최종 결과를 유일하게 확인할 수 없음");
    }
    if (field_or_null(*match, "pending") == true) {
        return {true, false, ""};
    }
    if (field_or_null(*match, "pending") != false ||
        !field_or_null(*match, "accepted").is_boolean()) {
        throw ValueError("명령 완료 상태 형식 오류");
    }
    const Json reason = field_or_null(*match, "reason");
    return {false, match->at("accepted").get<bool>(),
            match->contains("reason") ? python_text(reason) : ""};
}

bool ReclaimFlightLink::pause(double seconds) {
    if (controller_.wait) {
        return controller_.wait(seconds);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    return ending();
}

ReclaimFlightLink::ReclaimFlightLink(FlightLinkOptions options, ReclaimController controller,
                                     OperatorAuthority* authority)
    : FlightLink(std::move(options)), controller_(std::move(controller)), authority_(authority) {
    if (!controller_.present || !controller_.request) {
        throw ValueError("RC 복귀 추종기 어댑터가 필요합니다");
    }
}

ReclaimFlightLink::~ReclaimFlightLink() {
    closing_ = true;
    stop_worker();
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool ReclaimFlightLink::ending() const { return closing_.load() || stopping(); }

bool ReclaimFlightLink::rc_override() const {
    return state().follow_b && state().follow_b->rc_override();
}

void ReclaimFlightLink::check_epoch() {
    if (rc_epoch_ != state().pose_epoch) {
        rc_epoch_ = state().pose_epoch;
        calibration_.clear();
        reset_rc_retry();
        neutral_.reset("FC 문맥 변경");
    }
}

void ReclaimFlightLink::configure() {
    FlightLink::configure();
    calibration_.clear();
    rc_epoch_ = state().pose_epoch;
    neutral_.reset("FC RC 보정값 수신 대기");
    reset_rc_retry();
    const double current = now();
    for (const auto& name : rc_parameters()) {
        write(state().mav->param_request_read(1, 1, name, -1));
        rc_retry_at_[name] = current;
    }
    next_rc_retry_ = current + kRcRetrySpacingS;
}

void ReclaimFlightLink::reset_rc_retry() {
    rc_retry_at_.clear();
    next_rc_retry_ = -std::numeric_limits<double>::infinity();
}

// 축 매핑이 없으면 매핑부터, 매핑이 있으면 그 채널의 보정값만 찾는다.
// 값을 받았지만 범위 밖인 경우는 FC 실제 설정이므로 다시 묻지 않는다.
std::vector<std::string> ReclaimFlightLink::missing_rc_calibration() const {
    std::vector<std::string> missing;
    std::vector<int> channels;
    for (const char* axis : {"ROLL", "PITCH", "YAW", "THROTTLE"}) {
        const std::string name = std::string("RC_MAP_") + axis;
        const auto found = calibration_.find(name);
        if (found == calibration_.end() || !found->is_number()) {
            missing.push_back(name);
            continue;
        }
        const double value = found->get<double>();
        if (value == std::floor(value) && value >= 1 && value <= 18) {
            const int channel = static_cast<int>(value);
            if (std::find(channels.begin(), channels.end(), channel) == channels.end()) {
                channels.push_back(channel);
            }
        }
    }
    for (const int channel : channels) {
        for (const char* suffix : {"MIN", "MAX", "TRIM", "DZ", "REV"}) {
            const std::string name = "RC" + std::to_string(channel) + "_" + suffix;
            const auto found = calibration_.find(name);
            if (found == calibration_.end() || !found->is_number()) {
                missing.push_back(name);
            }
        }
    }
    return missing;
}

// 비행 중에도 RC → GUI 복귀가 가능해야 하므로 비행 상태로 막지 않는다.
// 한 번에 한 개, 같은 이름은 kRcRetryPerNameS 간격으로만 요청한다.
void ReclaimFlightLink::retry_rc_calibration() {
    auto& fc = state();
    if (!fc.config_requested || !fc.link || !fc.mav || fc.uid != expected_uid() ||
        !fresh("HEARTBEAT", 1.5)) {
        return;
    }
    const double current = now();
    if (current < next_rc_retry_) {
        return;
    }
    const auto missing = missing_rc_calibration();
    if (missing.empty()) {
        return;
    }
    const auto retry_at = [this](const std::string& name) {
        const auto found = rc_retry_at_.find(name);
        return found == rc_retry_at_.end() ? -std::numeric_limits<double>::infinity()
                                           : found->second;
    };
    std::string name = missing.front();
    for (const auto& candidate : missing) {
        if (retry_at(candidate) < retry_at(name)) {
            name = candidate;
        }
    }
    if (current - retry_at(name) < kRcRetryPerNameS) {
        return;
    }
    write(fc.mav->param_request_read(1, 1, name, -1));
    rc_retry_at_[name] = current;
    next_rc_retry_ = current + kRcRetrySpacingS;
}

void ReclaimFlightLink::ingest(const InboundMessage& message) {
    FlightLink::ingest(message);
    check_epoch();
    if (message.system != 1 || message.component != 1) {
        return;
    }
    if ((context_ || workflow_) && message.kind == "STATUSTEXT" &&
        message.data.value("text", std::string()).find("Pilot took over") != std::string::npos) {
        neutral_.reset("RC 우선권 개입");
    }
    const auto* value = sample("PARAM_VALUE");
    if (message.kind != "PARAM_VALUE" || value == nullptr || *value != message.data) {
        return;
    }
    const auto name = value->value("param_id", std::string());
    const auto& names = rc_parameters();
    if (std::find(names.begin(), names.end(), name) == names.end()) {
        return;
    }
    auto number = python_number(value->at("param_value"));
    const auto kind = value->value("param_type", 0);
    constexpr int kUint32 = 5;
    constexpr int kInt32 = 6;
    if (state().param_bytewise && (kind == kInt32 || kind == kUint32)) {
        const float raw = value->at("param_value").is_null()
                              ? std::numeric_limits<float>::quiet_NaN()
                              : value->at("param_value").get<float>();
        if (kind == kInt32) {
            std::int32_t decoded{};
            std::memcpy(&decoded, &raw, sizeof(decoded));
            number = decoded;
        } else {
            std::uint32_t decoded{};
            std::memcpy(&decoded, &raw, sizeof(decoded));
            number = decoded;
        }
    }
    if (number) {
        json_set(calibration_, name, *number);
    }
}

void ReclaimFlightLink::observe_rc() {
    check_epoch();
    const auto channels = state().samples.find("RC_CHANNELS");
    const auto* health = sample("SYS_STATUS");
    constexpr std::uint32_t kRcReceiver = 65536;
    bool healthy = health != nullptr && fresh("SYS_STATUS", 2.0);
    if (healthy && health != nullptr) {
        for (const auto* key :
             {"onboard_control_sensors_present", "onboard_control_sensors_enabled",
              "onboard_control_sensors_health"}) {
            healthy = healthy && (health->value(key, std::uint32_t{0}) & kRcReceiver) != 0;
        }
    }
    const bool found = channels != state().samples.end();
    observation_ = neutral_.update(
        now(), found ? channels->second.at : -std::numeric_limits<double>::infinity(),
        found ? channels->second.data : Json::object(), calibration_, healthy);
}

std::string ReclaimFlightLink::reclaim_reason(bool allow_workflow, bool ignore_emergency) const {
    if (authority_ != nullptr && authority_->emergency() && !ignore_emergency) {
        return "긴급 착륙 후 지상·무장 해제 및 제어권 확인 필요";
    }
    if (workflow_ && !allow_workflow) {
        return "RC → GUI 복귀 절차 진행 중";
    }
    auto why = reason("offboard");
    if (!why.empty()) {
        return why;
    }
    if (mode() != "hold" && mode() != "position") {
        return "실제 Hold 또는 Position에서만 GUI 복귀 가능";
    }
    const auto& fc = state();
    if (fc.pending || !fc.jobs.empty() || fc.landing || fc.external || fc.owned_flight) {
        return "기존 비행 명령·외부제어가 종료되어야 합니다";
    }
    if (fc.follow_b && (fc.follow_b->token_active() || fc.follow_b->bench_active() ||
                        fc.follow_b->manual_yaw().active())) {
        return "추종·yaw 송신 중에는 복귀할 수 없습니다";
    }
    if (!command_owner().status().value("available", false)) {
        return "다른 setpoint 송신자가 제어권을 사용 중";
    }
    if (!true_field(observation_, "ready")) {
        return observation_.value("reason", std::string("RC 중립 확인 필요"));
    }
    const auto* position = sample("LOCAL_POSITION_NED");
    const auto* attitude = sample("ATTITUDE");
    if (position == nullptr || attitude == nullptr || !fresh("LOCAL_POSITION_NED", .2) ||
        !fresh("ATTITUDE", .5)) {
        return "신선한 위치·방향 추정 필요";
    }
    for (const auto* key : {"x", "y", "z"}) {
        if (!python_number(position->value(key, Json()))) {
            return "유효한 위치·방향 추정 필요";
        }
    }
    if (!python_number(attitude->value("yaw", Json()))) {
        return "유효한 위치·방향 추정 필요";
    }
    if (height().first.value_or(limits().brake_height()) >= limits().brake_height()) {
        return "고도 보호 상한 접근 중";
    }
    return {};
}

Json ReclaimFlightLink::reclaim(const Json& client) {
    const std::scoped_lock lock(mutex());
    observe_rc();
    const auto why = reclaim_reason();
    if (ending() || !why.empty()) {
        throw ValueError(why.empty() ? "RC 복귀 연결부 종료" : why);
    }
    keepalive(client);
    if (worker_.joinable()) {
        worker_.join();
    }
    auto workflow = std::make_shared<Workflow>(
        Workflow{new_id(), client.get<std::string>(), state().pose_epoch, rc_override()});
    workflow->offboard_first =
        indoor_offboard_reclaim_ && environment() == std::optional<std::string>("indoor");
    workflow_ = workflow;
    try {
        worker_ = std::thread([this, workflow] { reclaim_sequence(workflow); });
    } catch (...) {
        workflow_.reset();
        throw;
    }
    return {{"accepted", true}, {"job_id", workflow->id}, {"pending", true}};
}

Json ReclaimFlightLink::submit(const Json& action, const Json& client, const Json& height_value) {
    const std::scoped_lock lock(mutex());
    if (workflow_ && action != "land") {
        throw ValueError("RC → GUI 복귀 절차 진행 중");
    }
    return FlightLink::submit(action, client, height_value);
}

void ReclaimFlightLink::workflow_guard(const Workflow& workflow) {
    if (ending() || workflow_.get() != &workflow || workflow.cancelled ||
        workflow.epoch != state().pose_epoch || state().owner != workflow.client ||
        now() - state().lease_at > limits().lease_ttl ||
        (authority_ != nullptr && authority_->owner() != workflow.client)) {
        throw ValueError("RC 복귀 세션·제어권·관제 연결 변경");
    }
    observe_rc();
    if (workflow.offboard_first) {
        // 실내: 복귀 작업이 Offboard를 직접 소유하므로 offboard 사전 조건 대신 모드·중립만 본다.
        // 위치·센서 만료와 스틱 움직임은 supervise()의 기존 취소 조건이 처리한다.
        if (!true_field(observation_, "ready") || (mode() != "position" && mode() != "offboard") ||
            (authority_ != nullptr && authority_->emergency())) {
            throw ValueError("RC 중립·Position/Offboard 유지 확인 유실");
        }
        return;
    }
    const auto why = reason("offboard");
    if (!why.empty() || !true_field(observation_, "ready") ||
        (mode() != "hold" && mode() != "position") ||
        (authority_ != nullptr && authority_->emergency())) {
        throw ValueError(why.empty() ? "RC 중립·실제 유지 모드 확인 유실" : why);
    }
}

void ReclaimFlightLink::confirm_controller_stop(const Workflow& workflow) {
    if (!controller_.present()) {
        return;
    }
    const double deadline = now() + 4;
    for (;;) {
        {
            const std::scoped_lock lock(mutex());
            workflow_guard(workflow);
            if (workflow.offboard_first ? mode() != "offboard" : mode() != "hold") {
                throw ValueError(workflow.offboard_first ? "Offboard 제자리 유지 유실"
                                                         : "실제 HOLD 유실");
            }
        }
        const auto id = new_id();
        const auto reply = controller_.request(
            {{"action", "stop"}, {"expected_uid", expected_uid()}, {"request_id", id}}, 500);
        if (!true_field(reply, "accepted")) {
            const auto why = reply.contains("reason") ? python_text(reply.at("reason"))
                                                      : std::string("추종기 STOP 거부");
            if (now() > deadline ||
                why.find("recovery_requires_fresh_actual_hold_without_rc_override") ==
                    std::string::npos) {
                throw ValueError(why);
            }
            pause(.1);
            continue;
        }
        if (!true_field(reply, "pending")) {
            return;
        }
        for (;;) {
            {
                const std::scoped_lock lock(mutex());
                workflow_guard(workflow);
            }
            const auto status =
                controller_.request({{"action", "status"}, {"expected_uid", expected_uid()}}, 500);
            // 원본 response[k]: 첫 STOP 응답에 세션 값이 없으면 KeyError
            for (const char* key : {"controller_session", "onboard_session", "fc_epoch"}) {
                if (!reply.contains(key)) {
                    throw ValueError(std::string("'") + key + "'");
                }
            }
            const auto completed =
                stop_completion(status, id, reply.at("controller_session"),
                                reply.at("onboard_session"), reply.at("fc_epoch"));
            if (!completed.pending) {
                if (!completed.accepted) {
                    throw ValueError(completed.reason);
                }
                return;
            }
            if (now() > deadline) {
                throw ValueError("추종기 STOP 완료 확인 시간 초과");
            }
            pause(.05);
        }
    }
}

void ReclaimFlightLink::reclaim_sequence(const std::shared_ptr<Workflow>& workflow) {
    try {
        if (workflow->offboard_first) {
            // 1. HOLD 없이 지금 위치를 목표로 Offboard 진입(기존 2단계와 같은 작업).
            {
                const std::scoped_lock lock(mutex());
                workflow_guard(*workflow);
                const auto why = reclaim_reason(true);
                if (!why.empty()) {
                    throw ValueError(why);
                }
                auto job = std::make_shared<Json>(
                    Json{{"id", workflow->id},
                         {"action", "offboard"},
                         {"reclaim", true},
                         {"reclaim_client", workflow->client},
                         {"reclaim_epoch", workflow->epoch},
                         {"status", "queued"},
                         {"message", "RC→GUI 현재 위치 목표 포착 · 실내 Offboard 제자리 유지"}});
                state().jobs.push_back(job);
                state().last_job = job;
            }
            // 2. 실제 Offboard 제자리 유지 확인 (supervise가 holding·RC 개입 해제를 정한다).
            const double deadline = now() + 12;
            for (;;) {
                if (pause(.05)) {
                    throw ValueError("RC 복귀 연결부 종료");
                }
                const std::scoped_lock lock(mutex());
                workflow_guard(*workflow);
                const auto job = state().last_job;
                if (job && job->value("id", std::string()) == workflow->id) {
                    const auto status = job->value("status", std::string());
                    if (status == "failed" || status == "rejected" || status == "cancelled" ||
                        status == "unknown") {
                        throw ValueError(job->value("message", std::string("Offboard 진입 실패")));
                    }
                }
                if (context_ && context_->holding && mode() == "offboard" && !state().pending) {
                    break;
                }
                if (now() > deadline) {
                    throw ValueError("RC 복귀 Offboard 제자리 유지 확인 시간 초과");
                }
            }
            // 3. 추종기가 인정하는 Offboard hover로 넘긴다 (HOLD 대체와 같은 함수).
            {
                const std::scoped_lock lock(mutex());
                workflow_guard(*workflow);
                auto& follow = state().follow_b;
                if (follow && follow->offboard_hover_enabled() &&
                    !follow->establish_hover("rc_gui_reclaim", false)) {
                    throw ValueError("Offboard 제자리 유지 인계 실패");
                }
            }
            // 4. 추종기 STOP·복구 확인 (Offboard 제자리 유지가 조건).
            confirm_controller_stop(*workflow);
            const std::scoped_lock lock(mutex());
            workflow_guard(*workflow);
            state().last_job = std::make_shared<Json>(
                Json{{"id", workflow->id},
                     {"action", "rc_gui_handoff"},
                     {"status", "done"},
                     {"message", "RC → GUI 복귀 완료 · Offboard 제자리 유지"}});
            record_event("RC → GUI 복귀 완료 · Offboard 제자리 유지");
            workflow_.reset();
            if (controller_.finished) {
                controller_.finished();
            }
            return;
        }
        const auto hold_id = workflow->id + "-hold";
        {
            const std::scoped_lock lock(mutex());
            workflow_guard(*workflow);
            auto job = std::make_shared<Json>(Json{{"id", hold_id},
                                                   {"action", "hold"},
                                                   {"status", "queued"},
                                                   {"rc_reclaim_hold", true},
                                                   {"message", "RC 복귀: 실제 HOLD 확인"}});
            state().jobs.push_back(job);
            state().last_job = job;
        }
        const double deadline = now() + 9;
        for (;;) {
            if (pause(.05)) {
                throw ValueError("RC 복귀 연결부 종료");
            }
            const std::scoped_lock lock(mutex());
            workflow_guard(*workflow);
            const auto job = state().last_job;
            if (job && job->value("id", std::string()) == hold_id) {
                const auto status = job->value("status", std::string());
                if (status == "done" && mode() == "hold") {
                    if (state().follow_b) {
                        state().follow_b->set_rc_override(false);
                    }
                    break;
                }
                if (status == "failed" || status == "rejected" || status == "cancelled" ||
                    status == "unknown") {
                    throw ValueError(job->value("message", std::string("실제 HOLD 확인 실패")));
                }
            }
            if (now() > deadline) {
                throw ValueError("RC 복귀 HOLD 확인 시간 초과");
            }
        }
        confirm_controller_stop(*workflow);
        const std::scoped_lock lock(mutex());
        workflow_guard(*workflow);
        const auto why = reclaim_reason(true);
        if (!why.empty()) {
            throw ValueError(why);
        }
        auto job = std::make_shared<Json>(Json{{"id", workflow->id},
                                               {"action", "offboard"},
                                               {"reclaim", true},
                                               {"reclaim_client", workflow->client},
                                               {"reclaim_epoch", workflow->epoch},
                                               {"status", "queued"},
                                               {"message", "RC→GUI 현재 위치 목표 포착"}});
        state().jobs.push_back(job);
        state().last_job = job;
        workflow_.reset();
    } catch (const std::exception& error) {
        const std::scoped_lock lock(mutex());
        const std::string message = error.what();
        if (state().pending && true_field(*state().pending, "rc_reclaim_hold")) {
            cancel("RC 복귀 중단: " + message);
        }
        const bool gui_holding = workflow->offboard_first && context_ && context_->holding &&
                                 mode() == std::optional<std::string>("offboard");
        if (state().follow_b && workflow->prior_rc && !gui_holding) {
            state().follow_b->set_rc_override(true);
        }
        if (!state().pending && state().jobs.empty()) {
            state().last_job = std::make_shared<Json>(Json{{"id", workflow->id},
                                                           {"action", "rc_gui_handoff"},
                                                           {"status", "failed"},
                                                           {"message", message}});
        }
        record_event("RC → GUI 복귀 실패: " + message);
        workflow_.reset();
    }
    if (controller_.finished) {
        controller_.finished();
    }
}

void ReclaimFlightLink::dispatch(const Job& job) {
    if (!true_field(*job, "reclaim")) {
        context_.reset();
        if (workflow_ && !true_field(*job, "rc_reclaim_hold")) {
            workflow_->cancelled = true;
        }
        FlightLink::dispatch(job);
        return;
    }
    observe_rc();
    auto why = reclaim_reason(workflow_ && workflow_->offboard_first &&
                              job->value("id", std::string()) == workflow_->id);
    const auto client = job->at("reclaim_client").get<std::string>();
    auto& fc = state();
    if (fc.owner != client || fc.pose_epoch != job->at("reclaim_epoch").get<int>() ||
        now() - fc.lease_at > limits().lease_ttl ||
        (authority_ != nullptr && authority_->owner() != client)) {
        why = "위치 목표 포착 전 RC 복귀 세션·제어권 변경";
    }
    const auto height_value = height().first;
    const auto* position = sample("LOCAL_POSITION_NED");
    const auto* attitude = sample("ATTITUDE");
    if (why.empty() && (!height_value || position == nullptr || attitude == nullptr)) {
        why = "신선한 위치·방향 추정 필요";
    }
    if (!why.empty() || !command_owner().acquire()) {
        fc.last_job = std::make_shared<Json>(*job);
        json_set(*fc.last_job, "status", "rejected");
        json_set(*fc.last_job, "message", why.empty() ? "송신권 획득 실패" : why);
        record_event(fc.last_job->at("message").get<std::string>());
        return;
    }
    fc.local_target = std::make_shared<Json>(Json{{"x", position->at("x")},
                                                  {"y", position->at("y")},
                                                  {"start_z", position->at("z")},
                                                  {"target_z", position->at("z")},
                                                  {"distance", 0},
                                                  {"height", height_value.value_or(0.0)},
                                                  {"yaw", attitude->at("yaw")},
                                                  {"climb_at", nullptr}});
    context_ = Context{fc.pose_epoch, mode()};
    fc.pending = job;
    job->update(Json{{"status", "running"},
                     {"stage", "priming"},
                     {"prime_until", now() + 1.3},
                     {"sent_at", now()},
                     {"deadline", now() + 8},
                     {"ack", false}});
    fc.holding = false;
    fc.owned_flight = true;
    fc.owned_mode_confirmed = false;
    local_setpoint();
}

void ReclaimFlightLink::clear_reclaim_target() {
    auto& fc = state();
    fc.external = fc.owned_flight = fc.owned_mode_confirmed = false;
    fc.local_target.reset();
    context_.reset();
    command_owner().release();
}

void ReclaimFlightLink::supervise() {
    retry_rc_calibration();
    observe_rc();
    auto& fc = state();
    if (authority_ != nullptr && fc.uid == expected_uid() && fresh("HEARTBEAT", 1.5) &&
        ground() == true && !armed()) {
        authority_->set_emergency(false);
    }
    if (fc.landing && fc.landing_mode_seen && fresh("HEARTBEAT", 1.5) && mode() != "land" &&
        ground() == false && fc.follow_b) {
        fc.follow_b->set_rc_override(true);
    }
    if (context_) {
        const auto current_mode = mode();
        const bool mode_ok =
            context_->holding
                ? current_mode == "offboard"
                : (current_mode == context_->origin_mode ||
                   (fc.pending && fc.pending->value("stage", std::string()) == "command" &&
                    current_mode == "offboard"));
        if (context_->epoch != fc.pose_epoch || !mode_ok || !true_field(observation_, "ready") ||
            !fresh("LOCAL_POSITION_NED", .2) || !fresh("ATTITUDE", .5) ||
            fc.uid != expected_uid() || !fresh("HEARTBEAT", 1.5)) {
            cancel("RC 입력/FC 상태 변경 — GUI 복귀·위치 송신 종료");
            if (fc.follow_b) {
                fc.follow_b->set_rc_override(true);
                fc.follow_b->invalidate("rc_reclaim_cancelled");
            }
            clear_reclaim_target();
        } else if (fc.owned_mode_confirmed && current_mode != "offboard") {
            context_.reset();
        }
    }
    FlightLink::supervise();
    if (context_ && fc.owned_mode_confirmed && mode() == "offboard") {
        context_->holding = true;
        if (fc.follow_b) {
            fc.follow_b->set_rc_override(false);
        }
    }
    if (context_ && !fc.pending && !fc.external && !fc.owned_flight) {
        context_.reset();
    }
}

void ReclaimFlightLink::finish(const std::string& status, std::string message) {
    const bool reclaiming = state().pending && true_field(*state().pending, "reclaim");
    FlightLink::finish(status, std::move(message));
    if (reclaiming && status != "done") {
        clear_reclaim_target();
    }
}

Json ReclaimFlightLink::yaw_handoff() {
    const std::scoped_lock lock(mutex());
    if (rc_override()) {
        throw ValueError("RC 인계 후에는 RC → GUI 복귀를 먼저 완료하세요");
    }
    auto result = FlightLink::yaw_handoff();
    if (true_field(result, "accepted")) {
        context_.reset();
    }
    return result;
}

void ReclaimFlightLink::close_link() {
    context_.reset();
    calibration_.clear();
    reset_rc_retry();
    neutral_.reset("FC 연결 종료");
    FlightLink::close_link();
}

Json ReclaimFlightLink::snapshot() {
    const std::scoped_lock lock(mutex());
    auto view = FlightLink::snapshot();
    if (workflow_) {
        json_set(view, "busy", true);
        auto& controls = view.at("controls");
        for (auto it = controls.begin(); it != controls.end(); ++it) {
            if (it.key() != "land") {
                *it = false;
            }
        }
    }
    const auto why = reclaim_reason(false, rc_override());
    std::string status = rc_override() ? "RC 우선" : "복귀 대기";
    if (context_) {
        status = "인계 확인 중";
    }
    if (workflow_) {
        status = workflow_->offboard_first ? "Offboard 제자리 유지·추종기 STOP 확인 중"
                                           : "실제 HOLD·추종기 STOP 확인 중";
    }
    if (context_ && context_->holding) {
        status = "GUI 위치 유지";
    }
    json_set(view, "rc_gui_handoff",
             {{"supported", true}, {"allowed", why.empty()}, {"status", status}, {"reason", why}});
    return view;
}
}  // namespace argus::gateway
