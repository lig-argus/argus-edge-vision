// FC 와 mavlink_gateway 각각의 링크 판단 (icd 2장, 6장)
// - 기대 주기(HEARTBEAT 1 Hz)의 두 배 동안 메시지가 없으면 지연, 3 s 동안 없으면 두절
// - 둘 다 두절이면 RPi 또는 라우터 이상
// - 수신률: 최근 10 s 동안 받은 HEARTBEAT 수 / 기대 수
#ifndef ARGUS_GCS_LINK_LINK_MONITOR_HPP_
#define ARGUS_GCS_LINK_LINK_MONITOR_HPP_

#include <QString>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>

#include "argus/gcs/link/log_entry.hpp"
#include "argus/gcs/link/vehicle_state.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

class LinkMonitor {
public:
    using LogHandler = std::function<void(const LogEntry&)>;

    static constexpr std::int64_t kQualityWindowNs = 10 * kNsPerSecond;

    // 이번 판단에서 연결됨(처음 연결 또는 두절 뒤 복구)으로 바뀐 링크
    struct Connected {
        bool fc{false};
        bool gateway{false};
    };

    explicit LinkMonitor(LogHandler log) : log_(std::move(log)) {}

    // 기체(sysid 1)의 FC 또는 mavlink_gateway 가 보낸 메시지 하나
    void onMessage(const mavlink_message_t& message, std::int64_t nowNs);

    // 링크 상태를 다시 판단해 state 에 쓴다 (주기 작업)
    Connected update(VehicleState& state, std::int64_t nowNs);

private:
    struct Times {
        std::optional<std::int64_t> firstNs;  // 처음 받은 시각 (수신률 구간의 시작)
        std::optional<std::int64_t> lastNs;
        std::deque<std::int64_t> heartbeats;  // 최근 kQualityWindowNs 동안의 HEARTBEAT 수신 시각
    };

    static void record(Times& times, const mavlink_message_t& message, std::int64_t nowNs);
    // 연결됨으로 바뀌었으면 true
    bool updateOne(ComponentLink& link, Times& times, const QString& name, LogKind kind,
                   std::int64_t nowNs);

    LogHandler log_;
    Times fc_;
    Times gateway_;
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_LINK_MONITOR_HPP_
