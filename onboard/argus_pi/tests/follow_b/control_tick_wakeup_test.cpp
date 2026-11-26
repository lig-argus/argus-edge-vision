// 2026-11-16 ControlTickWakeup 시험: 결과 없음→fallback, 결과 도착→즉시(최소 간격 지킴).
#include "argus/runtime/follow/include/control_tick_wakeup.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;
int failures = 0;
void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::cerr << "실패: " << what << '\n';
    }
}
double since(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
}  // namespace

int main() {
    {
        follow::ControlTickWakeup wakeup;
        const auto start = Clock::now();
        const bool event = wakeup.wait(start + milliseconds(50), start + milliseconds(20));
        const double ms = since(start);
        check(!event && ms >= 49. && ms < 80.,
              "결과 없음 → 50ms fallback (" + std::to_string(ms) + ")");
    }
    {
        follow::ControlTickWakeup wakeup;
        const auto start = Clock::now();
        std::thread sender([&] {
            std::this_thread::sleep_for(milliseconds(30));
            wakeup.notify();
        });
        const bool event = wakeup.wait(start + milliseconds(50), start + milliseconds(20));
        const double ms = since(start);
        sender.join();
        check(event && ms >= 29. && ms < 45., "30ms 결과 → 즉시 (" + std::to_string(ms) + ")");
    }
    {
        follow::ControlTickWakeup wakeup;
        const auto start = Clock::now();
        std::thread sender([&] {
            std::this_thread::sleep_for(milliseconds(5));
            wakeup.notify();
        });
        const bool event = wakeup.wait(start + milliseconds(50), start + milliseconds(20));
        const double ms = since(start);
        sender.join();
        check(event && ms >= 19.5 && ms < 35.,
              "5ms 결과 → 최소 간격 20ms 지킴 (" + std::to_string(ms) + ")");
    }
    {
        follow::ControlTickWakeup wakeup;
        wakeup.notify();  // 대기 전에 온 결과도 놓치지 않는다
        const auto start = Clock::now();
        const bool event = wakeup.wait(start + milliseconds(50), start);
        const double ms = since(start);
        check(event && ms < 10., "대기 전 결과 → 바로 (" + std::to_string(ms) + ")");
        const auto again = Clock::now();
        const bool second = wakeup.wait(again + milliseconds(30), again);
        check(!second && since(again) >= 29., "한 번 쓴 결과는 다시 깨우지 않음");
    }
    if (failures != 0) {
        std::cerr << failures << "개 실패\n";
        return 1;
    }
    std::cout << "ControlTickWakeupTest 통과\n";
    return 0;
}
