// 표적 기억과 현재 실관측 후보를 비교한다. 기억 수정과 기체 명령은 하지 않는다.
#pragma once

#include <optional>

#include "argus/control/target/include/target_memory.hpp"

namespace follow {

struct ReacquisitionResult {
    enum class Kind { None, Unique, Ambiguous } kind{Kind::None};
    std::optional<TrackRecordV1> candidate;
};

struct CandidateEvaluation {
    TrackRecordV1 track;
    std::optional<double> area_ratio, distance, score;
    std::string reason{"not_evaluated"};
};
struct ReacquisitionEvaluation {
    TrackRecordV1 previous;
    double dt_s{0}, ground_dt_s{0};
    double velocity_north{0}, velocity_east{0};
    std::vector<CandidateEvaluation> candidates;
    std::optional<double> best_score, second_score;
    ReacquisitionResult::Kind result{ReacquisitionResult::Kind::None};
};
using ReacquisitionTrace = std::vector<ReacquisitionEvaluation>;

class TargetReacquirer final {
public:
    explicit TargetReacquirer(TargetRecoveryConfig config, ReacquisitionTrace* trace = nullptr)
        : config_(config), trace_(trace) {}
    ReacquisitionResult find_candidate(const TargetMemory& memory, const TracksV1& frame,
                                       const TrackRecordV1* confirmation_track = nullptr) const;
    // 2026-11-10 공백 시간(초)에 따른 지면 게이트(m). 증가율 0이면 ground_gate_m 그대로.
    double ground_gate(double gap_s) const;
    // 기억의 CV 예측 지면 위치와 track 지면 위치의 거리(m)와 그때의 공백(초).
    // 어느 한쪽에 지면 정보가 없거나 시각이 거꾸로면 nullopt.
    struct GroundPrediction {
        double distance_m;
        double gap_s;
    };
    std::optional<GroundPrediction> predicted_ground_distance(const TargetMemory& memory,
                                                              const TracksV1& frame,
                                                              const TrackRecordV1& track) const;

private:
    TargetRecoveryConfig config_;
    ReacquisitionTrace* trace_;
};

}  // namespace follow
