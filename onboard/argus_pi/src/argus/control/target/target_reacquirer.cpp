// 같은 클래스·예상 위치·크기의 유일한 실관측만 재획득 후보로 제안한다.
#include "argus/control/target/include/target_reacquirer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace follow {

double TargetReacquirer::ground_gate(double gap_s) const {
    if (!(config_.ground_gate_growth_m_s > 0) || !std::isfinite(gap_s)) {
        return config_.ground_gate_m;
    }
    return std::min(config_.ground_gate_max_m,
                    config_.ground_gate_m + config_.ground_gate_growth_m_s * std::max(0., gap_s));
}

std::optional<TargetReacquirer::GroundPrediction> TargetReacquirer::predicted_ground_distance(
    const TargetMemory& memory, const TracksV1& frame, const TrackRecordV1& track) const {
    const auto& previous = memory.last_real_track;
    if (!previous.ground || !track.ground) {
        return std::nullopt;
    }
    // find_candidate와 같은 속도·시각 기준을 쓴다.
    const auto& identity = memory.identity_short_motion;
    const bool identity_velocity = !config_.trust_selected_id && config_.prefer_selected_continuity;
    const double velocity_north = identity_velocity
                                      ? (identity.valid ? identity.velocity_north_m_s : 0.)
                                      : memory.velocity_north_m_s;
    const double velocity_east = identity_velocity
                                     ? (identity.valid ? identity.velocity_east_m_s : 0.)
                                     : memory.velocity_east_m_s;
    const double dt_s =
        static_cast<double>(frame.sample_monotonic_ns - memory.last_real_sample_ns) / 1e9;
    const double ground_dt_s = frame.motion_sample_ns && memory.last_real_frame.motion_sample_ns
                                   ? static_cast<double>(*frame.motion_sample_ns -
                                                         *memory.last_real_frame.motion_sample_ns) /
                                         1e9
                                   : dt_s;
    if (dt_s < 0 || ground_dt_s < 0) {
        return std::nullopt;
    }
    const auto& before = *previous.ground;
    const auto& current = *track.ground;
    return GroundPrediction{std::hypot(current[0] - before[0] - velocity_north * ground_dt_s,
                                       current[1] - before[1] - velocity_east * ground_dt_s),
                            ground_dt_s};
}

ReacquisitionResult TargetReacquirer::find_candidate(
    const TargetMemory& memory, const TracksV1& frame,
    const TrackRecordV1* confirmation_track) const {
    ReacquisitionResult result;
    const auto& previous = memory.last_real_track;
    // Recovery scoring consumes identity evidence, not box-gated control velocity.
    const auto& identity = memory.identity_short_motion;
    const double velocity_north = (!config_.trust_selected_id && config_.prefer_selected_continuity)
                                      ? (identity.valid ? identity.velocity_north_m_s : 0.)
                                      : memory.velocity_north_m_s;
    const double velocity_east = (!config_.trust_selected_id && config_.prefer_selected_continuity)
                                     ? (identity.valid ? identity.velocity_east_m_s : 0.)
                                     : memory.velocity_east_m_s;

    const double width = previous.x2 - previous.x1;
    const double height = previous.y2 - previous.y1;
    const double dt_s =
        static_cast<double>(frame.sample_monotonic_ns - memory.last_real_sample_ns) / 1e9;
    const double ground_dt_s = frame.motion_sample_ns && memory.last_real_frame.motion_sample_ns
                                   ? static_cast<double>(*frame.motion_sample_ns -
                                                         *memory.last_real_frame.motion_sample_ns) /
                                         1e9
                                   : dt_s;
    if (dt_s < 0 || ground_dt_s < 0 || width <= 0 || height <= 0) {
        return result;
    }

    // 일정 이미지 속도 모델이다. 큰 카메라 운동의 실패가 확인되면 자세 정렬 예측을 확장한다.
    const double predicted_x = (previous.x1 + previous.x2) * 0.5 + memory.velocity_x_px_s * dt_s;
    const double predicted_y = (previous.y1 + previous.y2) * 0.5 + memory.velocity_y_px_s * dt_s;
    const double max_distance = std::hypot(width, height) * config_.max_center_distance_diagonals;
    double best_score = std::numeric_limits<double>::infinity();
    double second_score = best_score;

    ReacquisitionEvaluation evaluation;
    struct Capture {
        ReacquisitionTrace* sink;
        ReacquisitionEvaluation& value;
        const ReacquisitionResult& result;
        const double& best;
        const double& second;
        ~Capture() {
            if (!sink) {
                return;
            }
            value.result = result.kind;
            if (std::isfinite(best)) {
                value.best_score = best;
            }
            if (std::isfinite(second)) {
                value.second_score = second;
            }
            sink->push_back(std::move(value));
        }
    } capture{trace_, evaluation, result, best_score, second_score};
    if (trace_) {
        evaluation.previous = previous;
        evaluation.dt_s = dt_s;
        evaluation.ground_dt_s = ground_dt_s;
        evaluation.velocity_north = velocity_north;
        evaluation.velocity_east = velocity_east;
    }
    for (const auto& track : frame.tracks) {
        CandidateEvaluation* diagnostic = nullptr;
        if (trace_) {
            evaluation.candidates.push_back({track});
            diagnostic = &evaluation.candidates.back();
        }
        const auto reject = [&](const char* reason) {
            if (diagnostic) {
                diagnostic->reason = reason;
            }
        };
        if (!valid_track_record(track) || !track_fits_frame(track, frame) ||
            !track.real_detection ||
            track.last_real_detection_monotonic_ns != frame.sample_monotonic_ns ||
            track.class_id != previous.class_id || track.label != previous.label) {
            reject("invalid_or_not_current_same_class_detection");
            continue;
        }
        // 가림 뒤 커지는 같은 후보의 크기만 연속 비교한다. 지면 신원 예측은 원기억 그대로다.
        const auto& shape =
            previous.ground && confirmation_track && confirmation_track->track_id == track.track_id
                ? *confirmation_track
                : previous;
        const double shape_width = shape.x2 - shape.x1, shape_height = shape.y2 - shape.y1;
        double area_ratio =
            (track.x2 - track.x1) * (track.y2 - track.y1) / (shape_width * shape_height);
        if (config_.ground_contact_anchor) {
            if (!track.ground || track.x1 < 2 || track.x2 > frame.image_width - 2 || track.y2 < 2 ||
                track.y2 > frame.image_height - 2 || track.y2 - track.y1 < 8) {
                reject("ground_anchor_or_image_boundary");
                continue;
            }
            // 상단이 잘린 높이를 전체 차 크기로 비교하면 같은 native ID도 거부된다.
            // 관측된 폭의 제곱 비율과 기존 지면/경쟁차 gate를 함께 유지한다.
            if (shape.y1 < 2 || track.y1 < 2) {
                area_ratio = std::pow((track.x2 - track.x1) / shape_width, 2);
            }
        }
        double distance = std::hypot((track.x1 + track.x2) * 0.5 - predicted_x,
                                     (track.y1 + track.y2) * 0.5 - predicted_y);
        double gate = max_distance;
        double score = 0;
        if (previous.ground) {
            if (!track.ground) {
                reject("missing_ground");
                continue;
            }
            const auto& before = *previous.ground;
            const auto& current = *track.ground;
            distance = std::hypot(current[0] - before[0] - velocity_north * ground_dt_s,
                                  current[1] - before[1] - velocity_east * ground_dt_s);
            gate = ground_gate(ground_dt_s);
            const double dn = current[0] - before[0] - velocity_north * ground_dt_s;
            const double de = current[1] - before[1] - velocity_east * ground_dt_s;
            const double nn = current[2] + before[2], ne = current[3] + before[3],
                         ee = current[4] + before[4];
            const double determinant = nn * ee - ne * ne;
            if (determinant <= 0) {
                reject("nonpositive_covariance_determinant");
                continue;
            }
            score = (ee * dn * dn - 2 * ne * dn * de + nn * de * de) / determinant;
        }
        if (diagnostic) {
            diagnostic->area_ratio = area_ratio;
            diagnostic->distance = distance;
            diagnostic->score = score;
        }
        double maximum_area_ratio = config_.max_area_ratio;
        if (config_.prefer_selected_continuity && previous.ground && track.ground &&
            frame.generation == memory.binding.generation &&
            track.track_id == memory.binding.track_id && track.track_id == previous.track_id &&
            track.confidence >= .3) {
            // 동일 ID에서만 위치·경쟁 후보 검사를 유지하며 상자 확대를 허용한다.
            maximum_area_ratio = std::max(maximum_area_ratio, 4.0);
        }
        if (area_ratio < config_.min_area_ratio || area_ratio > maximum_area_ratio ||
            distance > gate) {
            reject(distance > gate ? "distance_gate" : "area_ratio_gate");
            continue;
        }
        reject("eligible");
        if (!previous.ground && result.candidate) {
            result = {ReacquisitionResult::Kind::Ambiguous, std::nullopt};
            return result;
        }
        if (score >= best_score) {
            second_score = std::min(second_score, score);
            continue;
        }
        second_score = best_score;
        best_score = score;
        result.kind = ReacquisitionResult::Kind::Unique;
        result.candidate = track;
    }
    // 넓은 metre gate 안의 모든 차량을 동등하게 보지 않는다. 불확실성상 경쟁하면 HOLD한다.
    if (previous.ground && second_score - best_score < 4.0) {
        result = {ReacquisitionResult::Kind::Ambiguous, std::nullopt};
        return result;
    }
    return result;
}

}  // namespace follow
