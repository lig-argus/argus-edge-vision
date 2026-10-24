// 시각을 읽는 인터페이스
// 규약: docs/design/uml/class_onboard.md 1절
// 구현: SteadyClock (core), 테스트용 FakeClock (core_testing)
// 사용: 두절 판정(2 s, 3 s), 유효 기한(valid_until_monotonic_ns) 검사 등 시간을 다루는 모든 onboard
// 코드
#ifndef ARGUS_CORE_CLOCK_HPP_
#define ARGUS_CORE_CLOCK_HPP_

#include <cstdint>

namespace argus::core {

// 시간을 직접 읽지 않고 이 인터페이스로 읽으면, 테스트가 FakeClock 으로 시간을 정해
// 실제로 기다리지 않고 시간 초과 동작을 검사할 수 있다.
class IClock {
public:
    IClock() = default;
    virtual ~IClock() = default;
    IClock(const IClock&) = delete;
    IClock& operator=(const IClock&) = delete;
    IClock(IClock&&) = delete;
    IClock& operator=(IClock&&) = delete;

    // RPi monotonic clock(CLOCK_MONOTONIC) 의 현재 시각(ns). 메시지의 *_monotonic_ns 필드와 같은
    // 기준이다. 줄어들지 않는다. 시스템 시각 보정(NTP)의 영향을 받지 않는다.
    virtual std::int64_t now_monotonic_ns() const = 0;
};

}  // namespace argus::core

#endif  // ARGUS_CORE_CLOCK_HPP_
