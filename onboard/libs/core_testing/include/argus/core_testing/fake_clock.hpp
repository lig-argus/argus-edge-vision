// 테스트용 IClock. 테스트가 시각을 직접 정하고 앞으로 움직인다.
#ifndef ARGUS_CORE_TESTING_FAKE_CLOCK_HPP_
#define ARGUS_CORE_TESTING_FAKE_CLOCK_HPP_

#include <cstdint>

#include "argus/core/clock.hpp"

namespace argus::core_testing {

class FakeClock : public core::IClock {
public:
    explicit FakeClock(std::int64_t start_ns = 0) : now_ns_(start_ns) {}

    std::int64_t now_monotonic_ns() const override { return now_ns_; }

    // 테스트용 조작. IClock 의 약속(줄어들지 않음)을 지키도록 뒤로 돌리지 않는다.
    void advance_ns(std::int64_t delta_ns) {
        if (delta_ns > 0) {
            now_ns_ += delta_ns;
        }
    }

private:
    std::int64_t now_ns_;
};

}  // namespace argus::core_testing

#endif  // ARGUS_CORE_TESTING_FAKE_CLOCK_HPP_
