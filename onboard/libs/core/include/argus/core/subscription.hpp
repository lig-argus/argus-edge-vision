// 구독(bus 토픽, MAVLink 메시지, 모드 이벤트)이나 RPC 응답 등록 한 건을 나타내는 RAII handle.
// 등록 함수(subscribe, serve)가 반환한다. 이 객체가 사라지면 등록이 자동으로 해제된다.
// 등록한 객체가 이것을 멤버로 들고 있으면,
// 그 객체가 사라진 뒤 그 객체의 handler 가 불리는 일이 없다.
// reset() 과 소멸은 handler 의 등록 해제이며, 이미 수락한 요청이나 진행 중인 작업을 취소하지
// 않는다. 취소는 각 인터페이스의 명령(stop 등)으로 한다.
#ifndef ARGUS_CORE_SUBSCRIPTION_HPP_
#define ARGUS_CORE_SUBSCRIPTION_HPP_

#include <exception>
#include <functional>
#include <utility>

namespace argus::core {

class Subscription {
public:
    // 아무 등록도 나타내지 않는 빈 handle
    Subscription() = default;

    // release 는 등록을 해제하는 함수. 등록을 받은 쪽(구현)이 만든다
    explicit Subscription(std::function<void()> release) : release_(std::move(release)) {}

    ~Subscription() { reset(); }

    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;

    // 이동하면 등록이 새 객체로 넘어가고, 원래 객체는 빈 handle 이 된다
    Subscription(Subscription&& other) noexcept
        : release_(std::exchange(other.release_, nullptr)) {}
    Subscription& operator=(Subscription&& other) noexcept {
        if (this != &other) {
            reset();
            release_ = std::exchange(other.release_, nullptr);
        }
        return *this;
    }

    // 지금 등록을 푼다. 이미 빈 handle 이면 아무것도 하지 않는다.
    // 해제 함수는 예외를 던지면 안 된다. 던지면 프로그램을 멈춘다 (소멸자에서 불리기 때문)
    void reset() noexcept {
        if (release_) {
            try {
                std::exchange(release_, nullptr)();
            } catch (...) {
                std::terminate();
            }
        }
    }

    bool active() const { return static_cast<bool>(release_); }

private:
    std::function<void()> release_;
};

}  // namespace argus::core

#endif  // ARGUS_CORE_SUBSCRIPTION_HPP_
