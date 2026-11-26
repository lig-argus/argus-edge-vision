// 단일 박스 시험에서 쓰는 표적 상태와 마지막 실관측의 시뮬레이션 시각을 정의한다.
// Confirmed는 실관측, Predicted는 잠깐의 미검출, Lost는 유실, Ambiguous는 모호한 입력이다.
#pragma once
namespace follow {
enum class TargetState { Confirmed, Predicted, Lost, Ambiguous };
struct TargetObservation {
    TargetState state{TargetState::Lost};
    double last_real_sim_s{0};
};
}  // namespace follow
