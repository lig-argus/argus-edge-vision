// 테스트용 IClock. 테스트가 시각을 직접 지정한다. 뒤로 가지 않는다
#ifndef ARGUS_GCS_TEST_FAKE_CLOCK_HPP_
#define ARGUS_GCS_TEST_FAKE_CLOCK_HPP_

#include <QtGlobal>

#include "argus/gcs/clock.hpp"

namespace argus::gcs {

class FakeClock : public IClock {
public:
    explicit FakeClock(qint64 startNs = 0) : nowNs_(startNs) {}

    qint64 nowMonotonicNs() const override { return nowNs_; }

    // 테스트용: 시각을 ns 만큼 앞으로 옮긴다. 음수는 무시한다 (monotonic clock 은 뒤로 가지 않는다)
    void advanceNs(qint64 ns) {
        if (ns > 0) {
            nowNs_ += ns;
        }
    }

private:
    qint64 nowNs_;
};

}  // namespace argus::gcs

#endif  // ARGUS_GCS_TEST_FAKE_CLOCK_HPP_
