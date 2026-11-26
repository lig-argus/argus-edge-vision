// 가상 조종 (MANUAL_CONTROL 20 Hz, icd 5.2). RC 조종기가 없을 때의 시연과 시험용이다.
// 켜져 있는 동안만 보낸다. 보내지 않으면 PX4 가 조종 입력 두절로 판단한다.
// 켤 수 없는 조건(FC 링크 두절, 실제 RC 연결)이 생기면 스스로 끈다.
#ifndef ARGUS_GCS_LINK_MANUAL_CONTROL_HPP_
#define ARGUS_GCS_LINK_MANUAL_CONTROL_HPP_

#include <QString>
#include <cstdint>
#include <functional>
#include <optional>

#include "argus/gcs/link/command.hpp"
#include "argus/gcs/link/log_entry.hpp"
#include "argus/gcs/mavlink_connection/mavlink_connection.hpp"

namespace argus::gcs::link {

class ManualControl {
public:
    using LogHandler = std::function<void(const LogEntry&)>;

    static constexpr std::int64_t kPeriodNs = 50 * kNsPerMs;  // 20 Hz

    // 송신 간격이 이보다 길면 기록한다 (통제기 쪽 지연과 Wi-Fi 두절을 구분하기 위해)
    static constexpr std::int64_t kLateNs = 200 * kNsPerMs;

    ManualControl(mavlink_connection::IMavlinkConnection& connection, LogHandler log)
        : connection_(connection), log_(std::move(log)) {}

    // 켤 수 없는 이유. 실제 RC 가 있으면 RC 로만 조종한다 (PX4 v1.16 은 RC 우선 설정이 없다)
    static std::optional<QString> refusal(bool fcLinkUp, bool physicalRcPresent);

    // 켠다. refusal 이 있으면 켜지 않고 그 이유를 돌려준다
    std::optional<QString> enable(const std::optional<QString>& refusal, std::int64_t nowNs);
    void disable();

    bool enabled() const { return enabled_; }
    void setAxes(const ManualAxes& axes) { axes_ = axes; }
    const ManualAxes& axes() const { return axes_; }

    // 주기 작업. 켜져 있으면 주기마다 보내고, refusal 이 생기면 끈다
    void tick(const std::optional<QString>& refusal, std::int64_t nowNs);

private:
    void send(std::int64_t nowNs);

    mavlink_connection::IMavlinkConnection& connection_;
    LogHandler log_;
    ManualAxes axes_;
    bool enabled_{false};
    std::int64_t lastSentNs_{0};
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_MANUAL_CONTROL_HPP_
