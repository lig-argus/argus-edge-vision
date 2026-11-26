#include "argus/tracking/motion/include/short_motion_estimator.hpp"

#include <cassert>
#include <cmath>
#include <limits>

using namespace follow;
int main() {
    ShortMotionConfig config;
    config.enabled = true;
    ShortMotionEstimator motion(config);
    ShortMotionEstimate result;
    constexpr std::int64_t begin = 1000000000LL;
    for (int i = 0; i <= 30; ++i) {
        result = motion.update(begin + i * 50000000LL, {8 + .6 * i * .05, 0});
    }
    assert(result.valid && result.phase == ObservedMotionPhase::Moving &&
           std::abs(result.velocity_north_m_s - .6) < 1e-8);
    assert(result.direction_valid);
    assert(result.feedforward_valid);
    double confirmed = -1;
    for (int i = 31; i <= 50; ++i) {
        result =
            motion.update(begin + i * 50000000LL, {8.9 + .005 * std::sin(i), .005 * std::cos(i)});
        if (result.phase == ObservedMotionPhase::Stopped && confirmed < 0) {
            confirmed = (i - 30) * .05;
        }
    }
    assert(confirmed > 0 && confirmed <= 1 && result.phase == ObservedMotionPhase::Stopped);
    assert(!result.direction_valid);
    const auto repeated = motion.update(begin + 50 * 50000000LL, {100, 100});
    assert(repeated.phase == result.phase && repeated.north_m == result.north_m);
    for (int i = 51; i <= 75; ++i) {
        result = motion.update(begin + i * 50000000LL, {8.9 - .6 * (i - 50) * .05, 0});
    }
    assert(result.phase == ObservedMotionPhase::Moving &&
           result.velocity_north_m_s < -.55);  // 실제 후진은 부호로 버리지 않는다.
    assert(result.direction_valid);
    result = motion.update(begin + 100 * 50000000LL, {9, 0});
    assert(!result.valid && result.phase == ObservedMotionPhase::Unknown);
    result = motion.update(begin + 101 * 50000000LL, {std::numeric_limits<double>::quiet_NaN(), 0});
    assert(!result.valid);
    motion.reset();
    for (int i = 0; i <= 30; ++i) {
        result = motion.update(begin + i * 50000000LL, {8 + .02 * std::sin(i), .02 * std::cos(i)});
    }
    assert(result.phase == ObservedMotionPhase::Stopped);
    // 정지 라벨의 확인 대기와 새 실관측 출발은 동시에 표현한다.
    motion.reset();
    for (int i = 0; i <= 30; ++i) {
        motion.update(begin + i * 50000000LL, {8, 0});
    }
    bool departure = false;
    for (int i = 31; i <= 48; ++i) {
        result = motion.update(begin + i * 50000000LL, {8 - .6 * (i - 30) * .05, 0});
        if (result.departure_pending) {
            departure = true;
            assert(result.valid && result.position_changed_since_stop);
            assert(result.phase == ObservedMotionPhase::Stopped &&
                   result.velocity_north_m_s <= -config.resume_speed_m_s);
        }
    }
    assert(departure && result.phase == ObservedMotionPhase::Moving && !result.departure_pending);
    motion.reset();
    for (int i = 0; i <= 30; ++i) {
        motion.update(begin + i * 50000000LL, {8, 0});
    }
    for (int i = 31; i <= 65; ++i) {
        result = motion.update(begin + i * 50000000LL, {8 + .2 * (i - 30) * .05, 0});
    }
    assert(result.phase == ObservedMotionPhase::Moving &&
           result.velocity_north_m_s > .18);  // 느린 실제 출발도 고정 정지에 갇히지 않는다.
    assert(
        result.direction_valid &&
        !result.feedforward_valid);  // 저속 위치/방향은 유지하지만 불명확한 저속 전향은 억제한다.
    motion.reset();
    for (int i = 0; i <= 30; ++i) {
        motion.update(begin + i * 50000000LL, {8, 0});
    }
    result = motion.update(begin + 31 * 50000000LL, {8.10, 0});
    assert(result.valid && result.phase == ObservedMotionPhase::Stopped &&
           result.position_changed_since_stop);
    assert(!result.feedforward_valid);  // 위치 보정은 깨어나도 확정 속도를 꾸며내지 않는다.
    // 낮은 residual gate를 넘는 실제 좌표 잡음은 운동 증거로 쓰지 않는다.
    motion.reset();
    for (int i = 31; i <= 60; ++i) {
        result = motion.update(begin + i * 50000000LL, {8 + (i % 2 ? .3 : -.3), 0});
    }
    assert(!result.valid && result.phase == ObservedMotionPhase::Unknown);
    for (const double window : {0., 2.}) {
        bool rejected = false;
        try {
            auto invalid = config;
            invalid.window_s = window;
            ShortMotionEstimator bad(invalid);
        } catch (...) {
            rejected = true;
        }
        assert(rejected);
    }
}
