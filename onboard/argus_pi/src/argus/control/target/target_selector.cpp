// 명시적 선택·시작을 유지하며 opt-in 표적 기억을 같은 잠금 아래 갱신한다.
#include "argus/control/target/include/target_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "argus/control/contracts/include/v2_1_features.hpp"
#include "argus/control/target/include/selected_class_filter.hpp"

namespace follow {
bool TargetSelector::reset_input_context() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ || pending_) {
        return false;
    }
    latest_.reset();
    latest_received_at_ = Clock::time_point{};
    target_manager_.stop();
    command_cache_.clear();
    ++command_epoch_;
    return true;
}
namespace {
double iou(const TrackRecordV1& a, const TrackRecordV1& b) {
    const double left = std::max(a.x1, b.x1), top = std::max(a.y1, b.y1);
    const double right = std::min(a.x2, b.x2), bottom = std::min(a.y2, b.y2);
    const double intersection = std::max(0.0, right - left) * std::max(0.0, bottom - top);
    const double area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    const double area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    const double union_area = area_a + area_b - intersection;
    return union_area > 0 ? intersection / union_area : 0;
}
bool same_request(const TargetCommand& a, const TargetCommand& b) {
    return a.request_id == b.request_id && a.action == b.action && a.generation == b.generation &&
           a.track_id == b.track_id && a.expected_uid == b.expected_uid &&
           a.class_id == b.class_id && a.label == b.label && a.sensor == b.sensor;
}
}  // namespace

TargetSelector::TargetSelector(TargetSelectorConfig config, ReacquisitionTrace* trace)
    : config_(config), target_manager_(config.recovery, trace) {
    const bool invalid_age =
        config_.max_tracks_age.count() <= 0 || config_.max_tracks_age > std::chrono::seconds(5);
    const bool invalid_lifetime =
        !std::isfinite(config_.lost_track_ttl_s) || config_.lost_track_ttl_s <= 0;
    const bool invalid_overlap = !std::isfinite(config_.overlap_iou_reject) ||
                                 config_.overlap_iou_reject <= 0 || config_.overlap_iou_reject > 1;
    const bool invalid_capacity = config_.dedup_capacity == 0 || config_.dedup_capacity > 4096;
    if (invalid_age || invalid_lifetime || invalid_overlap || invalid_capacity) {
        throw std::invalid_argument("invalid target selector settings");
    }
}

bool TargetSelector::update(const TracksV1& tracks, Clock::time_point received_at) {
    if (received_at == Clock::time_point{} || !valid_tracks_envelope(tracks)) {
        return false;
    }
    if (ground_tracks_schema(tracks.schema) || config_.require_local_monotonic_sample) {
        const auto sampled =
            Clock::time_point{std::chrono::nanoseconds{tracks.sample_monotonic_ns}};
        if (tracks.clock_domain != "host_monotonic" || received_at < sampled ||
            received_at - sampled > config_.max_tracks_age) {
            return false;
        }
    }
    std::vector<std::int64_t> ids;
    ids.reserve(tracks.tracks.size());
    for (const auto& item : tracks.tracks) {
        if (!valid_track_record(item) || !track_fits_frame(item, tracks) ||
            item.last_real_detection_monotonic_ns < 0) {
            return false;
        }
        ids.push_back(item.track_id);
    }
    std::sort(ids.begin(), ids.end());
    if (std::adjacent_find(ids.begin(), ids.end()) != ids.end()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_) {
        const bool context_changed = tracks.source != latest_->source ||
                                     camera_only_tracks(tracks) != camera_only_tracks(*latest_) ||
                                     tracks.fc_epoch != latest_->fc_epoch ||
                                     tracks.clock_domain != latest_->clock_domain ||
                                     tracks.schema != latest_->schema;
        const bool clock_reversed = tracks.frame_id > latest_->frame_id &&
                                    tracks.sample_monotonic_ns < latest_->sample_monotonic_ns;
        if (context_changed || clock_reversed) {
            const bool old_target_lost =
                config_.recovery.enabled &&
                target_manager_.snapshot(received_at, config_.max_tracks_age).phase ==
                    TargetPhase::Lost;
            const bool idle_camera_switch =
                config_.allow_idle_camera_context_switch && (!active_ || old_target_lost) &&
                !clock_reversed &&
                (camera_only_tracks(tracks) != camera_only_tracks(*latest_) ||
                 tracks.fc_epoch != latest_->fc_epoch) &&
                tracks.source == latest_->source && tracks.clock_domain == latest_->clock_domain &&
                tracks.generation > latest_->generation &&
                tracks.sample_monotonic_ns > latest_->sample_monotonic_ns;
            if (idle_camera_switch) {
                // Observation only: discard selection, never carry it across coordinate bases.
                pending_.reset();
                active_.reset();
                target_manager_.stop();
                command_cache_.clear();
                ++command_epoch_;
                latest_.reset();
            } else {
                if (config_.recovery.enabled && active_) {
                    target_manager_.invalidate(context_changed ? "tracking_context_changed"
                                                               : "sample_clock_reversed");
                }
                return false;
            }
        }
        if (latest_ && (tracks.frame_id <= latest_->frame_id ||
                        tracks.sample_monotonic_ns < latest_->sample_monotonic_ns)) {
            return false;
        }
        // 정지/역행한 센서 시간을 새 host 표본으로 재발행해 신선도를 늘리지 않는다.
        if (latest_ && tracks.motion_sample_ns && latest_->motion_sample_ns &&
            *tracks.motion_sample_ns <= *latest_->motion_sample_ns) {
            return false;
        }
        // 같은 표본을 새 frame_id로 재발행해도 수신 신선도나 확인 횟수를 늘리지 않는다.
        if (latest_ && config_.recovery.enabled &&
            tracks.sample_monotonic_ns == latest_->sample_monotonic_ns) {
            return false;
        }
    }
    latest_ = tracks;
    latest_received_at_ = received_at;
    if (config_.recovery.enabled && active_) {
        const auto memory = target_manager_.snapshot(received_at, config_.max_tracks_age).memory;
        const auto* current = memory ? find_target(tracks, memory->binding) : nullptr;
        if (!config_.recovery.trust_selected_id && current &&
            (current->class_id != memory->last_real_track.class_id ||
             current->label != memory->last_real_track.label)) {
            current = nullptr;
        }
        if (!config_.recovery.trust_selected_id && current && current->real_detection &&
            overlapping_candidate(tracks, *current)) {
            target_manager_.pause_ambiguous("overlapping_candidate_ambiguous");
        } else {
            // Keep the full frame for selection/display and filter only the
            // manager input. Missing selected-class observations stay empty.
            const auto filtered =
                memory && !config_.recovery.trust_selected_id
                    ? selected_class_tracks(tracks, memory->last_real_track.class_id)
                    : tracks;
            target_manager_.observe(filtered, received_at, config_.max_tracks_age);
        }
    }
    return true;
}

bool TargetSelector::tracks_fresh(Clock::time_point now) const {
    if (!latest_ || latest_received_at_ == Clock::time_point{} || now < latest_received_at_ ||
        now - latest_received_at_ > config_.max_tracks_age) {
        return false;
    }
    if (ground_tracks_schema(latest_->schema)) {
        const auto sampled =
            Clock::time_point{std::chrono::nanoseconds{latest_->sample_monotonic_ns}};
        return now >= sampled && now - sampled <= config_.max_tracks_age;
    }
    return true;
}

const TrackRecordV1* TargetSelector::find_target(const TracksV1& message,
                                                 const SelectedTargetKey& key) const {
    if (message.generation != key.generation) {
        return nullptr;
    }
    const auto it = std::find_if(message.tracks.begin(), message.tracks.end(),
                                 [&](const auto& track) { return track.track_id == key.track_id; });
    return it == message.tracks.end() ? nullptr : &*it;
}

bool TargetSelector::overlapping_candidate(const TracksV1& message,
                                           const TrackRecordV1& selected) const {
    return std::any_of(message.tracks.begin(), message.tracks.end(), [&](const auto& other) {
        return other.track_id != selected.track_id && other.class_id == selected.class_id &&
               other.state == "CONFIRMED" && iou(selected, other) >= config_.overlap_iou_reject;
    });
}

TargetCommandResult TargetSelector::apply(const TargetCommand& command, Clock::time_point now) {
    auto reject = [&](const char* reason) {
        return TargetCommandResult{command.request_id, false, "REJECTED", reason};
    };
    if (command.action == TargetCommandAction::Stop) {
        pending_.reset();
        active_.reset();
        if (config_.recovery.enabled) {
            target_manager_.stop();
        }
        ++command_epoch_;
        return {command.request_id, true, "STOPPED", "operator_stop"};
    }
    if (!tracks_fresh(now)) {
        return reject("tracks_stale_or_missing");
    }
    if ((command.action == TargetCommandAction::Start ||
         command.action == TargetCommandAction::SelectAndStart) &&
        camera_only_tracks(*latest_)) {
        return reject("camera_only_tracking_not_flight_ready");
    }
    if (command.action == TargetCommandAction::SelectAndStart &&
        (!command.class_id || !command.label || !command.sensor)) {
        return reject("atomic_start_requires_full_target_identity");
    }
    if (command.action == TargetCommandAction::Select ||
        command.action == TargetCommandAction::SelectAndStart) {
        if (active_) {
            return reject("stop_follow_before_reselect");
        }
        SelectedTargetKey key{command.generation, command.track_id};
        if (command.sensor && *command.sensor != latest_->source) {
            return reject("candidate_sensor_identity_mismatch");
        }
        const auto* candidate = find_target(*latest_, key);
        if (!candidate || candidate->state != "CONFIRMED" || !candidate->real_detection) {
            return reject("candidate_not_current_real_detection");
        }
        if (command.class_id.has_value() != command.label.has_value() ||
            (command.class_id &&
             (*command.class_id != candidate->class_id || *command.label != candidate->label))) {
            return reject("candidate_class_identity_mismatch");
        }
        if (overlapping_candidate(*latest_, *candidate)) {
            return reject("overlapping_candidate_ambiguous");
        }
        pending_ = key;
        if (config_.recovery.enabled) {
            target_manager_.select(config_.recovery.trust_selected_id
                                       ? *latest_
                                       : selected_class_tracks(*latest_, candidate->class_id),
                                   *candidate, latest_received_at_);
        }
        if (command.action == TargetCommandAction::Select) {
            return {command.request_id, true, "SELECTED_PENDING", "operator_selected"};
        }
    }
    if (command.action == TargetCommandAction::Start ||
        command.action == TargetCommandAction::SelectAndStart) {
        if (!pending_) {
            return reject("no_pending_target_selection");
        }
        const auto* candidate = find_target(*latest_, *pending_);
        if (!candidate || candidate->state != "CONFIRMED" || !candidate->real_detection ||
            overlapping_candidate(*latest_, *candidate)) {
            return reject("selected_target_not_confirmed_now");
        }
        active_ = pending_;
        if (config_.recovery.enabled) {
            target_manager_.start(config_.recovery.trust_selected_id
                                      ? *latest_
                                      : selected_class_tracks(*latest_, candidate->class_id),
                                  *candidate, latest_received_at_);
        }
        ++command_epoch_;
        return {command.request_id, true, "FOLLOWING", "fresh_selected_target"};
    }
    return reject("unsupported_command");
}

TargetCommandResult TargetSelector::execute(const TargetCommand& command, Clock::time_point now) {
    if (command.request_id.empty() || command.request_id.size() > 128) {
        return {command.request_id, false, "REJECTED", "invalid_request_id"};
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto cached = command_cache_.find(command.request_id);
    if (cached != command_cache_.end()) {
        if (same_request(cached->second.command, command)) {
            return cached->second.result;
        }
        return {command.request_id, false, "REJECTED", "request_id_reused_with_different_command"};
    }
    if (config_.recovery.enabled) {
        now = std::max(now, latest_received_at_);
    }
    auto result = apply(command, now);
    command_cache_.emplace(command.request_id, CachedCommand{command, result});
    if (command_cache_.size() > config_.dedup_capacity) {
        command_cache_.erase(command_cache_.begin());
    }
    return result;
}

void TargetSelector::stop_for_shutdown(Clock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 내부 종료는 외부 request_id 캐시를 통하지 않고 기존 Stop 적용만 재사용한다.
    (void)apply({"", TargetCommandAction::Stop}, now);
}

TargetSelectionSnapshot TargetSelector::snapshot(Clock::time_point now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    // tick 시작 뒤 worker가 갱신했을 수 있다. 읽기는 포함된 로컬 수신보다 과거가 될 수 없다.
    // 원본 표본 clock이나 단절 기한은 바꾸지 않는다.
    if (config_.recovery.enabled) {
        now = std::max(now, latest_received_at_);
    }
    TargetSelectionSnapshot result;
    result.cv_ten_frame_policy = config_.recovery.v32_cv_ten_frame;
    result.snapshot_at = now;
    result.command_epoch = command_epoch_;
    result.target = active_ ? active_ : pending_;
    if (result.target && latest_) {
        if (const auto* identity = find_target(*latest_, *result.target)) {
            result.identity_class_id = identity->class_id;
            result.identity_label = identity->label;
            result.identity_source = latest_->source;
        }
    }
    const bool fresh = tracks_fresh(now);
    if (fresh) {
        result.latest_frame = *latest_;
    }
    if (active_) {
        result.phase = TargetSelectionSnapshot::Phase::Following;
    } else if (pending_) {
        result.phase = TargetSelectionSnapshot::Phase::Pending;
    } else {
        result.phase = TargetSelectionSnapshot::Phase::Stopped;
    }
    if (config_.recovery.enabled && (active_ || pending_)) {
        const auto recovery = target_manager_.snapshot(now, config_.max_tracks_age);
        result.target_phase = recovery.phase;
        result.target_revision = recovery.revision;
        result.recovery_reason = recovery.reason;
        result.recovery_pause = active_.has_value() && recovery.pause_commands;
        if (recovery.memory) {
            result.mission_target_id = recovery.memory->mission_target_id;
            result.target = recovery.memory->binding;
            result.identity_class_id = recovery.memory->last_real_track.class_id;
            result.identity_label = recovery.memory->last_real_track.label;
            result.identity_source = recovery.memory->source;
            result.box_motion_suspect = recovery.memory->box_motion_suspect;
            result.cv_velocity_north_m_s = recovery.memory->cv_velocity_north_m_s;
            result.cv_velocity_east_m_s = recovery.memory->cv_velocity_east_m_s;
            result.cv_velocity_unconfirmed = recovery.memory->cv_velocity_unconfirmed;
            result.cv_velocity_fit_sample_ns = recovery.memory->cv_velocity_fit_sample_ns;
            result.ground_motion_valid = recovery.memory->ground_motion_valid;
            result.velocity_north_m_s = recovery.memory->velocity_north_m_s;
            result.velocity_east_m_s = recovery.memory->velocity_east_m_s;
            result.ground_acceleration_valid = recovery.memory->ground_acceleration_valid;
            result.acceleration_north_m_s2 = recovery.memory->acceleration_north_m_s2;
            result.acceleration_east_m_s2 = recovery.memory->acceleration_east_m_s2;
            result.short_motion_enabled = config_.recovery.short_motion.enabled;
            result.short_motion = recovery.memory->short_motion;
            result.age_since_real_s =
                std::chrono::duration<double>(now - recovery.memory->last_real_received_at).count();
        }
        if (!active_ || !recovery.memory || recovery.phase == TargetPhase::Lost) {
            return result;
        }
        if (result.recovery_pause) {
            result.state = TargetState::Predicted;
            result.prediction_valid_until =
                recovery.memory->last_real_received_at + config_.recovery.max_gap;
            const auto origin_ns = recovery.memory->last_real_sample_ns;
            const auto origin = Clock::time_point{std::chrono::nanoseconds{origin_ns}};
            const auto deadline = origin + std::chrono::seconds{3};
            // 2026-11-16 사용자 지시 "3초 가라니깐 왜 1초만 감", "그 3초 동안 계속 비교판정하도록
            // 실패 한번 나와도": 3초 기한(마지막 실관측 + 3초) 안에서는 표적을 아직 받아들이지
            // 못한 모든 공백 단계(GapHold·Reacquire·Confirm)에서 GAP_CV를 이어 가고, 판정은 매
            // 프레임 다시 한다. 14:58 실내: 1초 공백 뒤 ID 20이 지면 예측 밖으로 돌아온 프레임이
            // 후보 확인(Confirm)으로 넘어가며 GAP_CV가 꺼졌고, 다음 프레임은 열 복귀 검증으로
            // 보류됐다. 다른 차량과 구분하지 못한 모호한 공백(pause_ambiguous)만 기존처럼 멈춘다.
            const bool ambiguous = recovery.reason == "known_competitor_identity_unresolved" ||
                                   recovery.reason == "candidate_is_known_other_vehicle" ||
                                   recovery.reason == "locked_ground_candidates_ambiguous" ||
                                   recovery.reason == "reacquisition_ambiguous";
            const bool unresolved_gap = recovery.phase == TargetPhase::GapHold ||
                                        recovery.phase == TargetPhase::Reacquire ||
                                        recovery.phase == TargetPhase::Confirm;
            if (v21_enabled("ARGUS_V21_CV_GAP") &&
                recovery.memory->clock_domain == "host_monotonic" && origin_ns > 0 &&
                now >= origin) {
                result.cv_origin_sample_ns = origin_ns;
                result.cv_gap_deadline = deadline;
                result.cv_visibility_reference = recovery.memory->cv_visibility_reference;
                result.cv_gap_active = now < deadline && unresolved_gap && !ambiguous &&
                                       (!v21_enabled("ARGUS_V23_CV_MOVING_ONLY") ||
                                        recovery.memory->cv_last_observed_moving) &&
                                       std::isfinite(result.cv_velocity_north_m_s) &&
                                       std::isfinite(result.cv_velocity_east_m_s);
                if (config_.recovery.v32_cv_ten_frame) {
                    // 60프레임 전 기록이 있으면 그 속도를 쓴다. 2026-11-09 사용자 결정(11-03 공백
                    // CV 3초): 기록이 없으면 마지막 저장 속도로 CV 이동한다.
                    result.cv_gap_active =
                        result.cv_gap_active && recovery.memory->cv_last_observed_moving;
                    const auto old = recovery.memory->cv_ten_frame.selected(
                        recovery.memory->last_real_frame.frame_id);
                    if (old && !old->unconfirmed && old->source_ns < origin_ns &&
                        origin_ns - old->source_ns <= kV21CvWindowNs) {
                        result.cv_velocity_north_m_s = old->north;
                        result.cv_velocity_east_m_s = old->east;
                        result.cv_velocity_fit_sample_ns = old->fit_ns;
                        result.cv_velocity_unconfirmed = old->unconfirmed;
                        result.cv_history_frame_id = old->frame_id;
                        result.cv_history_source_ns = old->source_ns;
                    }
                }
            }
            return result;
        }
        if (!fresh) {
            return result;
        }
        const auto* current = find_target(*latest_, recovery.memory->binding);
        if (!config_.recovery.trust_selected_id && current &&
            (current->class_id != recovery.memory->last_real_track.class_id ||
             current->label != recovery.memory->last_real_track.label)) {
            current = nullptr;
        }
        if (!current && latest_->tracks.empty() && recovery.phase == TargetPhase::Locked &&
            recovery.memory->last_real_track.ground) {
            result.latest_frame = recovery.memory->last_real_frame;
            result.selected_track = recovery.memory->last_real_track;
            result.state = TargetState::Confirmed;
            result.fresh_real_detection =
                true;  // 원래 실관측이 아직 TTL 안이다. 새 관측을 합성하지 않는다.
            result.observation_valid_until =
                recovery.memory->last_real_received_at + config_.max_tracks_age;
            return result;
        }
        if (current && current->real_detection &&
            current->last_real_detection_monotonic_ns == latest_->sample_monotonic_ns) {
            result.selected_track = *current;
            if (config_.use_fitted_ground_position && recovery.memory->ground_acceleration_valid &&
                result.selected_track->ground) {
                // Same observed reference and sample clock; fitting does not extend the observation
                // TTL.
                (*result.selected_track->ground)[0] = recovery.memory->fitted_north_m;
                (*result.selected_track->ground)[1] = recovery.memory->fitted_east_m;
            }
            if (config_.use_fitted_ground_position && result.short_motion_enabled &&
                result.short_motion.valid &&
                result.short_motion.phase == ObservedMotionPhase::Stopped &&
                result.selected_track->ground) {
                (*result.selected_track->ground)[0] = result.short_motion.north_m;
                (*result.selected_track->ground)[1] = result.short_motion.east_m;
            }
            result.state = TargetState::Confirmed;
            result.fresh_real_detection = true;
            result.observation_valid_until =
                recovery.memory->last_real_received_at + config_.max_tracks_age;
        }
        return result;
    }
    if (!active_ || !fresh) {
        return result;
    }
    const auto* track = find_target(*latest_, *active_);
    if (!track) {
        return result;
    }
    result.selected_track = *track;
    if (!config_.recovery.trust_selected_id && overlapping_candidate(*latest_, *track)) {
        result.state = TargetState::Ambiguous;
        result.fresh_real_detection = false;
        return result;
    }
    const auto age_ns = latest_->sample_monotonic_ns - track->last_real_detection_monotonic_ns;
    result.age_since_real_s = static_cast<double>(age_ns) / 1e9;
    if (track->state == "CONFIRMED" && track->real_detection && age_ns == 0) {
        result.state = TargetState::Confirmed;
        result.fresh_real_detection = true;
    } else if (track->state == "PREDICTED" && result.age_since_real_s <= config_.lost_track_ttl_s) {
        result.state = TargetState::Predicted;
        // Legacy prediction age is a source-time DIFFERENCE, mapped to receipt time.
        // Cap by input freshness; repeated snapshots cannot extend either deadline.
        const auto remaining = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(config_.lost_track_ttl_s - result.age_since_real_s));
        result.prediction_valid_until =
            std::min(latest_received_at_ + config_.max_tracks_age, latest_received_at_ + remaining);
    }
    return result;
}

void TargetSelector::advance(Clock::time_point now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_.recovery.enabled && active_) {
        target_manager_.advance(std::max(now, latest_received_at_), config_.max_tracks_age);
    }
}

void TargetSelector::invalidate_recovery(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_.recovery.enabled && active_) {
        target_manager_.invalidate(reason.c_str());
    }
}
}  // namespace follow
