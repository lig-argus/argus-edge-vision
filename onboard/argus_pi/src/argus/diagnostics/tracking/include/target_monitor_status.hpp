// 공개 감시 상태는 비행 상태가 아니다. native/mission ID와 원본 실관측 시각을 유지한다.
#pragma once
#include <string>

#include "argus/control/target/include/target_selector.hpp"
namespace follow {
inline constexpr char kTargetMonitorTopic[] = "tracking/target_status";
std::string encode_target_monitor_status(const TargetSelectionSnapshot&, Clock::time_point now);
}  // namespace follow
