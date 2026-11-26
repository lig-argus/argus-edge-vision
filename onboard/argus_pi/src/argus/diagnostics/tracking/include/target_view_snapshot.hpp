// GUI/Overlay 이전의 읽기 모델. 같은 선택기 snapshot에 들어 있는 frame과 ID만 연결한다.
#pragma once
#include <vector>

#include "argus/control/target/include/target_selector.hpp"
namespace follow {
struct TargetObjectIdentity {
    std::size_t track_index{0};
    std::optional<std::uint64_t> target_id;
    bool is_selected{false};
    bool binding_confirmed{false};
};
struct TargetViewSnapshot {
    TargetSelectionSnapshot selection;
    Clock::time_point published_at{};
    bool frame_available{false};
    std::optional<double> source_age_s;
    std::vector<TargetObjectIdentity> objects;
};
TargetViewSnapshot build_target_view_snapshot(
    TargetSelectionSnapshot selection, Clock::time_point now,
    std::chrono::milliseconds max_age = std::chrono::milliseconds(250));
}  // namespace follow
