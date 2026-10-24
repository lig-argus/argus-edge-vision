// 실제 시각을 읽는 IClock 구현.
// SteadyClock 객체는 각 실행 파일의 main() 에서 하나만 만든다.
// 시간을 쓰는 클래스에는 생성자 인자(const IClock&)로 넘긴다.
// 클래스가 SteadyClock 을 직접 만들면 테스트에서 FakeClock 으로 바꿀 수 없다.
#ifndef ARGUS_CORE_STEADY_CLOCK_HPP_
#define ARGUS_CORE_STEADY_CLOCK_HPP_

#include <chrono>
#include <cstdint>

#include "argus/core/clock.hpp"

namespace argus::core {

// std::chrono::steady_clock 을 사용한다. Linux(libstdc++, libc++) 에서는 CLOCK_MONOTONIC 이다.
class SteadyClock : public IClock {
public:
    std::int64_t now_monotonic_ns() const override {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
};

}  // namespace argus::core

#endif  // ARGUS_CORE_STEADY_CLOCK_HPP_
