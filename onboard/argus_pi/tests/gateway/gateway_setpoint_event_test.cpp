// 2026-11-16 FC setpoint 이벤트 송신 시각 규칙 시험.
#include <iostream>
#include <string>

#include "argus/gateway/follow/include/setpoint_stream_timing.hpp"

namespace gw = argus::gateway;

namespace {
int failures = 0;
void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::cerr << "실패: " << what << '\n';
    }
}
constexpr std::int64_t kMs = 1'000'000;
}  // namespace

int main() {
    // 꺼짐(0): 기존 50ms 규칙과 같다. 새 명령이어도 50ms 전에는 보내지 않는다.
    check(gw::setpoint_stream_due(std::nullopt, 0, false, 0), "첫 송신은 바로");
    check(!gw::setpoint_stream_due(0, 49 * kMs, true, 0), "꺼짐: 49ms 새 명령 대기");
    check(gw::setpoint_stream_due(0, 50 * kMs, false, 0), "꺼짐: 50ms 재송신");
    // 켬(20ms): 새 명령은 20ms 이상이면 바로, 아니면 기다린다. 새 명령이 없으면 50ms 그대로.
    check(!gw::setpoint_stream_due(0, 19 * kMs, true, 20 * kMs),
          "켬: 19ms 새 명령은 최소 간격 대기");
    check(gw::setpoint_stream_due(0, 20 * kMs, true, 20 * kMs), "켬: 20ms 새 명령 즉시");
    check(!gw::setpoint_stream_due(0, 30 * kMs, false, 20 * kMs), "켬: 새 명령 없으면 30ms 미송신");
    check(gw::setpoint_stream_due(0, 50 * kMs, false, 20 * kMs), "켬: 50ms 재송신 유지");
    // 시간 역행(시계 재설정)은 새 명령이어도 보내지 않는다 (기존 규칙과 같음).
    check(!gw::setpoint_stream_due(100 * kMs, 90 * kMs, true, 20 * kMs), "시간 역행 미송신");
    // 환경변수 해석
    std::string error;
    check(gw::setpoint_event_min_interval_from_env(nullptr, &error) == 0 && error.empty(),
          "미설정=꺼짐");
    check(gw::setpoint_event_min_interval_from_env("0", &error) == 0 && error.empty(), "0=꺼짐");
    check(gw::setpoint_event_min_interval_from_env("20", &error) == 20 * kMs, "20ms 켬");
    check(gw::setpoint_event_min_interval_from_env("10", &error) == 10 * kMs, "하한 10ms");
    check(gw::setpoint_event_min_interval_from_env("50", &error) == 50 * kMs, "상한 50ms");
    error.clear();
    check(gw::setpoint_event_min_interval_from_env("5", &error) == 0 && !error.empty(),
          "5ms 거부→꺼짐");
    error.clear();
    check(gw::setpoint_event_min_interval_from_env("20ms", &error) == 0 && !error.empty(),
          "숫자 아님 거부→꺼짐");
    // 최대 송신률: 2ms마다 새 명령이 와도 20ms 간격을 넘지 않는다.
    int sends = 0;
    std::optional<std::int64_t> last;
    for (std::int64_t t = 0; t <= 1000 * kMs; t += 2 * kMs) {
        if (gw::setpoint_stream_due(last, t, true, 20 * kMs)) {
            ++sends;
            last = t;
        }
    }
    check(sends <= 51, "1초 송신 51회 이하 (" + std::to_string(sends) + ")");
    if (failures != 0) {
        std::cerr << failures << "개 실패\n";
        return 1;
    }
    std::cout << "GatewaySetpointEventTest 통과\n";
    return 0;
}
