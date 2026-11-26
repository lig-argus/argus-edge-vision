// 2026-11-16 사용자 지시(FC 송신 이벤트 방식): setpoint를 FC로 보낼지 정하는 시각 규칙.
// 기존: 마지막 송신 뒤 50ms마다 최신 setpoint를 다시 보낸다 (제어기 새 명령은 평균 약 25ms 대기).
// 이벤트(선택, 기본 꺼짐): 새 명령을 받으면 마지막 송신 뒤 min_interval 이상이면 그 루프에서 바로
// 보낸다. 50ms 재송신 주기는 그대로 유지한다. 최대 송신률은 1/min_interval (예: 20ms → 50Hz)로
// 묶인다.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <optional>
#include <string>

namespace argus::gateway {

inline constexpr std::int64_t kSetpointStreamPeriodNs = 50'000'000;
inline constexpr std::int64_t kSetpointEventMinIntervalLowNs = 10'000'000;
inline constexpr std::int64_t kSetpointEventMinIntervalHighNs = 50'000'000;

// event_min_interval_ns 0 = 꺼짐(기존 50ms만).
inline bool setpoint_stream_due(std::optional<std::int64_t> last_stream_ns, std::int64_t now_ns,
                                bool fresh_command, std::int64_t event_min_interval_ns) {
    if (!last_stream_ns) {
        return true;
    }
    const std::int64_t elapsed = now_ns - *last_stream_ns;
    if (elapsed >= kSetpointStreamPeriodNs) {
        return true;
    }
    return event_min_interval_ns > 0 && fresh_command && elapsed >= event_min_interval_ns;
}

// ARGUS_GW_SETPOINT_EVENT_MIN_MS: 비었거나 0이면 꺼짐, 10..50 정수(ms)만 켠다.
// 잘못된 값은 게이트웨이 기동을 막지 않고 꺼짐으로 둔다 (error에 이유를 남긴다).
inline std::int64_t setpoint_event_min_interval_from_env(const char* value, std::string* error) {
    if (value == nullptr || *value == '\0') {
        return 0;
    }
    char* end = nullptr;
    const long ms = std::strtol(value, &end, 10);
    if (end == value || *end != '\0') {
        if (error != nullptr) {
            *error = "ARGUS_GW_SETPOINT_EVENT_MIN_MS is not an integer; event send disabled";
        }
        return 0;
    }
    if (ms == 0) {
        return 0;
    }
    const std::int64_t ns = static_cast<std::int64_t>(ms) * 1'000'000;
    if (ns < kSetpointEventMinIntervalLowNs || ns > kSetpointEventMinIntervalHighNs) {
        if (error != nullptr) {
            *error = "ARGUS_GW_SETPOINT_EVENT_MIN_MS must be 0 or 10..50; event send disabled";
        }
        return 0;
    }
    return ns;
}

}  // namespace argus::gateway
