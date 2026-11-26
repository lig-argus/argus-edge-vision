#include "argus/gcs/link/link_monitor.hpp"

#include <algorithm>

#include "argus/gcs/link/link_protocol.hpp"

namespace argus::gcs::link {

namespace {

constexpr std::int32_t kFullPct = 100;

bool isUp(LinkState state) { return state != LinkState::Never && state != LinkState::Lost; }

}  // namespace

void LinkMonitor::onMessage(const mavlink_message_t& message, std::int64_t nowNs) {
    if (message.sysid != kVehicleSysid) {
        return;
    }
    if (message.compid == kFcCompid) {
        record(fc_, message, nowNs);
    } else if (message.compid == kGatewayCompid) {
        record(gateway_, message, nowNs);
    }
}

void LinkMonitor::record(Times& times, const mavlink_message_t& message, std::int64_t nowNs) {
    if (!times.firstNs) {
        times.firstNs = nowNs;
    }
    times.lastNs = nowNs;
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        times.heartbeats.push_back(nowNs);
    }
}

LinkMonitor::Connected LinkMonitor::update(VehicleState& state, std::int64_t nowNs) {
    Connected connected;
    connected.fc = updateOne(state.fcLink, fc_, QStringLiteral("FC"), LogKind::Fc, nowNs);
    connected.gateway = updateOne(state.gatewayLink, gateway_, QStringLiteral("임무 컴퓨터"),
                                  LogKind::Gateway, nowNs);
    const bool suspect =
        state.fcLink.state == LinkState::Lost && state.gatewayLink.state == LinkState::Lost;
    if (suspect && !state.routerSuspect) {
        log_(gcsLog(
            LogKind::Gcs, LogLevel::Critical,
            QStringLiteral("FC 와 임무 컴퓨터 모두 두절: RPi 전원, Wi-Fi, 라우터 확인 필요")));
    }
    state.routerSuspect = suspect;
    return connected;
}

bool LinkMonitor::updateOne(ComponentLink& link, Times& times, const QString& name, LogKind kind,
                            std::int64_t nowNs) {
    while (!times.heartbeats.empty() && nowNs - times.heartbeats.front() > kQualityWindowNs) {
        times.heartbeats.pop_front();
    }
    if (!times.lastNs || !times.firstNs) {
        return false;
    }
    const std::int64_t silentNs = nowNs - *times.lastNs;
    LinkState next = LinkState::Ok;
    if (silentNs >= kLinkLostNs) {
        next = LinkState::Lost;
    } else if (silentNs >= kLinkDelayedNs) {
        next = LinkState::Delayed;
    }
    const LinkState before = link.state;
    if (next != LinkState::Lost && before == LinkState::Never) {
        log_(gcsLog(kind, QStringLiteral("%1 연결됨").arg(name)));
    } else if (next == LinkState::Lost && before != LinkState::Lost) {
        log_(gcsLog(kind, LogLevel::Critical,
                    QStringLiteral("%1 링크 두절 (3 s 동안 수신 없음)").arg(name)));
    } else if (next != LinkState::Lost && before == LinkState::Lost) {
        log_(gcsLog(kind, QStringLiteral("%1 링크 복구").arg(name)));
    }
    link.state = next;

    // 처음 받은 뒤 10 s 가 지나지 않았으면 그동안의 기대 수로 나눈다
    const std::int64_t windowNs =
        std::clamp(nowNs - *times.firstNs, kHeartbeatPeriodNs, kQualityWindowNs);
    const auto expected = static_cast<std::int32_t>(windowNs / kHeartbeatPeriodNs);
    const auto received = static_cast<std::int32_t>(times.heartbeats.size());
    link.qualityPct = std::min(kFullPct, received * kFullPct / expected);
    return isUp(next) && !isUp(before);
}

}  // namespace argus::gcs::link
