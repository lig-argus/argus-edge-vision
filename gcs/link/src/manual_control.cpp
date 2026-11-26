#include "argus/gcs/link/manual_control.hpp"

#include "argus/gcs/link/link_protocol.hpp"

namespace argus::gcs::link {

std::optional<QString> ManualControl::refusal(bool fcLinkUp, bool physicalRcPresent) {
    if (!fcLinkUp) {
        return QStringLiteral("FC 미연결");
    }
    if (physicalRcPresent) {
        return QStringLiteral("RC 조종기 연결됨: 가상 조종기 사용 불가 (RC 우선)");
    }
    return std::nullopt;
}

std::optional<QString> ManualControl::enable(const std::optional<QString>& refusal,
                                             std::int64_t nowNs) {
    if (refusal) {
        log_(gcsLog(LogKind::Gcs, LogLevel::Serious,
                    QStringLiteral("가상 조종 켜지 않음. %1").arg(*refusal)));
        return refusal;
    }
    if (!enabled_) {
        enabled_ = true;
        log_(gcsLog(LogKind::Gcs, QStringLiteral("가상 조종 활성화 (MANUAL_CONTROL 20 Hz)")));
        send(nowNs);
    }
    return std::nullopt;
}

void ManualControl::disable() {
    if (enabled_) {
        enabled_ = false;
        log_(gcsLog(LogKind::Gcs, QStringLiteral("가상 조종 비활성화 (송신 중지)")));
    }
}

void ManualControl::tick(const std::optional<QString>& refusal, std::int64_t nowNs) {
    if (!enabled_) {
        return;
    }
    if (refusal) {
        enabled_ = false;
        log_(gcsLog(LogKind::Gcs, LogLevel::Serious,
                    QStringLiteral("가상 조종 꺼짐. %1").arg(*refusal)));
        return;
    }
    const std::int64_t gapNs = nowNs - lastSentNs_;
    if (gapNs < kPeriodNs) {
        return;
    }
    if (gapNs > kLateNs) {
        log_(gcsLog(
            LogKind::Gcs, LogLevel::Serious,
            QStringLiteral("가상 조종 송신이 %1 ms 늦음 (통제기 쪽 지연)").arg(gapNs / kNsPerMs)));
    }
    send(nowNs);
}

void ManualControl::send(std::int64_t nowNs) {
    lastSentNs_ = nowNs;
    mavlink_message_t message{};
    // sysid, compid 는 연결이 채운다. 버튼과 확장 축은 쓰지 않는다 (0)
    mavlink_msg_manual_control_pack(0, 0, &message, kVehicleSysid, axes_.x, axes_.y, axes_.z,
                                    axes_.r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    connection_.send(message);
}

}  // namespace argus::gcs::link
