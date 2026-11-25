// 지정한 시각에 handler 를 한 번 호출하도록 등록하는 인터페이스
// 구현: IPoller 를 구현하는 클래스 중 타이머를 함께 기다릴 수 있는 클래스 (ZmqPoller, PollPoller)
// 사용: 요청의 응답 시간 초과를 판정하는 통신 구현 (bus 의 DDS 구현)
#ifndef ARGUS_CORE_TIMER_SCHEDULER_HPP_
#define ARGUS_CORE_TIMER_SCHEDULER_HPP_

#include <cstdint>
#include <functional>

#include "argus/core/subscription.hpp"

namespace argus::core {

// IPoller 와 분리한 이유는 IFdWatcher 와 같다: EventLoop 는 process_events() 만 사용하고, 시간
// 초과를 판정하는 클래스는 등록만 사용한다
class ITimerScheduler {
public:
    using Handler = std::function<void()>;

    ITimerScheduler() = default;
    virtual ~ITimerScheduler() = default;
    ITimerScheduler(const ITimerScheduler&) = delete;
    ITimerScheduler& operator=(const ITimerScheduler&) = delete;
    ITimerScheduler(ITimerScheduler&&) = delete;
    ITimerScheduler& operator=(ITimerScheduler&&) = delete;

    // 구현이 사용하는 IClock 의 시각이 deadline_ns 이상이 되면 IPoller::process_events() 안에서
    // handler 를 한 번 호출한다. 호출한 뒤에는 등록이 없어진다. deadline_ns 가 이미 지난 시각이면
    // 다음 process_events() 에서 호출한다.
    // 반환된 Subscription 이 소멸하면 등록이 해제된다. Subscription 은 이 객체보다 늦게 소멸하여도
    // 된다. handler 가 비어 있으면 등록하지 않고 빈 Subscription 을 반환한다
    [[nodiscard]] virtual Subscription add_timer_at(std::int64_t deadline_ns, Handler handler) = 0;
};

}  // namespace argus::core

#endif  // ARGUS_CORE_TIMER_SCHEDULER_HPP_
