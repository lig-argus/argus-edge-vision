// 2026-11-16 사용자 지시(제어 이벤트 구동): 새 추적 결과가 선택기에 받아들여지면 제어 tick을 바로
// 깨운다. 결과가 없으면 기존 주기(fallback)에 깬다. 연속 tick은 earliest(최소 간격) 전에는 돌지
// 않는다. 측정 근거(11/16 11:32 비행): 추적 출력 → 제어 tick 대기 p50 22.5ms가 남은 지연의 가장 큰
// 몫.
#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace follow {

class ControlTickWakeup {
public:
    // 추적 수신 스레드에서 부른다.
    void notify() {
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            pending_ = true;
        }
        wake_.notify_one();
    }

    // 제어 스레드에서 부른다. fallback까지 기다리되, 새 결과가 오면 earliest 이후 바로 돌아온다.
    // 새 결과로 깼으면 true, fallback 시각이 되어 깼으면 false.
    template <class TimePoint>
    bool wait(TimePoint fallback, TimePoint earliest) {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_until(lock, fallback, [this] { return pending_; });
        if (!pending_) {
            return false;
        }
        pending_ = false;
        lock.unlock();
        if (TimePoint::clock::now() < earliest) {
            std::this_thread::sleep_until(earliest);
        }
        return true;
    }

private:
    std::mutex mutex_;
    std::condition_variable wake_;
    bool pending_{false};
};

}  // namespace follow
