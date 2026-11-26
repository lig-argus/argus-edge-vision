// 새 객체뷰 topic만 추가한다. 원래 tracks/status/명령 wire는 바꾸지 않는다.
#pragma once
#include <string>

#include "argus/diagnostics/tracking/include/target_view_snapshot.hpp"
namespace follow {
inline constexpr char kTargetViewTopic[] = "tracking/target_view";
std::string encode_target_view(const TargetViewSnapshot& view);
}  // namespace follow
