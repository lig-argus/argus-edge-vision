// 현재 시각(monotonic clock)을 읽는 인터페이스. onboard 의 core::IClock 과 이름, 단위가 같다
// 구현: SteadyClock (std::chrono::steady_clock), 테스트용 FakeClock (gcs/test)
// 사용: TelemetryModel (지연, 연결 끊김 판정), CommandSender (ACK 1 s, IN_PROGRESS 뒤 3 s)
#ifndef ARGUS_GCS_CLOCK_HPP_
#define ARGUS_GCS_CLOCK_HPP_

#include <QtGlobal>

namespace argus::gcs {

class IClock {
public:
    IClock() = default;
    virtual ~IClock() = default;
    IClock(const IClock&) = delete;
    IClock& operator=(const IClock&) = delete;
    IClock(IClock&&) = delete;
    IClock& operator=(IClock&&) = delete;

    // monotonic clock 의 현재 시각 (ns). 기준점은 의미 없고 두 값의 차만 사용한다.
    // wall clock(시스템 시각)이 아니므로 시각 보정으로 뒤로 가지 않는다
    virtual qint64 nowMonotonicNs() const = 0;
};

class SteadyClock : public IClock {
public:
    qint64 nowMonotonicNs() const override;
};

}  // namespace argus::gcs

#endif  // ARGUS_GCS_CLOCK_HPP_
