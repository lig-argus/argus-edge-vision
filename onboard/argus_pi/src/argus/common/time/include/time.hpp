// 영상 시험 경로에서 수신 경과 시간을 재는 공통 시계와 시간 리터럴을 제공한다.
// steady_clock은 단조 시계이며 Gazebo 메시지의 시뮬레이션 시간과 별개다.
#pragma once
#include <chrono>
namespace follow {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
}  // namespace follow
