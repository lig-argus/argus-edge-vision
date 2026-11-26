#include "argus/gcs/link/command_sender.hpp"

#include "argus/gcs/link/link_protocol.hpp"

namespace argus::gcs::link {

namespace {

constexpr std::uint8_t kUnknownProgress = 255;  // COMMAND_ACK.progress 를 모름

std::uint8_t targetCompid(CommandTarget target) {
    switch (target) {
        case CommandTarget::Fc:
            return kFcCompid;
        case CommandTarget::Gateway:
            return kGatewayCompid;
    }
    return kFcCompid;
}

std::optional<CommandTarget> targetOf(std::uint8_t compid) {
    if (compid == kFcCompid) {
        return CommandTarget::Fc;
    }
    if (compid == kGatewayCompid) {
        return CommandTarget::Gateway;
    }
    return std::nullopt;
}

}  // namespace

QString resultName(std::uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED:
            return QStringLiteral("수락");
        case MAV_RESULT_TEMPORARILY_REJECTED:
            return QStringLiteral("일시 거부(TEMPORARILY_REJECTED)");
        case MAV_RESULT_DENIED:
            return QStringLiteral("거부(DENIED)");
        case MAV_RESULT_UNSUPPORTED:
            return QStringLiteral("미지원(UNSUPPORTED)");
        case MAV_RESULT_FAILED:
            return QStringLiteral("실패(FAILED)");
        case MAV_RESULT_IN_PROGRESS:
            return QStringLiteral("진행 중");
        case MAV_RESULT_CANCELLED:
            return QStringLiteral("취소(CANCELLED)");
        default:
            return QStringLiteral("결과 %1").arg(result);
    }
}

CommandSender::CommandSender(mavlink_connection::IMavlinkConnection& connection,
                             std::uint32_t firstNumber, LogHandler log, FinishHandler finished)
    : connection_(connection),
      log_(std::move(log)),
      finished_(std::move(finished)),
      nextNumber_(firstNumber % kCommandNumberModulus) {}

std::uint32_t CommandSender::takeNumber() {
    const std::uint32_t number = nextNumber_;
    nextNumber_ = (nextNumber_ + 1U) % kCommandNumberModulus;
    return number;
}

void CommandSender::send(const OutgoingCommand& command, std::int64_t nowNs) {
    const Key key{command.target, command.mavCmd};
    const auto previous = pending_.find(key);
    if (previous != pending_.end()) {
        log_(gcsLog(LogKind::Command, LogLevel::Serious,
                    QStringLiteral("%1 응답 대기 중단, %2 전송")
                        .arg(previous->second.command.name, command.name)));
        setStage(previous->second.command.command, CommandTrace::Stage::Failed, nowNs,
                 QStringLiteral("새 명령으로 대체"));
    }
    Pending& pending = pending_[key];
    pending = Pending{command, 0, nowNs, false};
    CommandTrace trace;
    trace.sentNs = nowNs;
    traces_[command.command] = trace;
    transmit(pending, nowNs);
}

void CommandSender::transmit(Pending& pending, std::int64_t nowNs) {
    const OutgoingCommand& command = pending.command;
    mavlink_message_t message{};
    // sysid, compid 는 연결이 이 통제기의 값으로 바꾼다
    mavlink_msg_command_long_pack(0, 0, &message, kVehicleSysid, targetCompid(command.target),
                                  command.mavCmd, static_cast<std::uint8_t>(pending.sends),
                                  command.params[0], command.params[1], command.params[2],
                                  command.params[3], command.params[4], command.params[5],
                                  command.params[6]);
    connection_.send(message);
    ++pending.sends;
    pending.lastEventNs = nowNs;
    CommandTrace& trace = traces_[command.command];
    trace.sends = pending.sends;
    if (trace.stage != CommandTrace::Stage::Waiting) {
        trace.stage = CommandTrace::Stage::Waiting;
        trace.stageNs = nowNs;
    }
    const QString resend =
        pending.sends > 1 ? QStringLiteral(" (재전송 %1)").arg(pending.sends - 1) : QString();
    log_(gcsLog(LogKind::Command, command.name + resend));
}

void CommandSender::tick(std::int64_t nowNs) {
    for (auto it = pending_.begin(); it != pending_.end();) {
        Pending& pending = it->second;
        const std::int64_t waited = nowNs - pending.lastEventNs;
        if (pending.inProgress && waited > kInProgressTimeoutNs) {
            finishFailed(pending, QStringLiteral("진행 중 응답 뒤 최종 응답 없음"), nowNs);
            it = pending_.erase(it);
            continue;
        }
        if (!pending.inProgress && waited > kRetryIntervalNs) {
            if (pending.sends >= kMaxSends) {
                finishFailed(pending, QStringLiteral("응답 없음 (%1회 전송)").arg(kMaxSends),
                             nowNs);
                it = pending_.erase(it);
                continue;
            }
            transmit(pending, nowNs);
        }
        ++it;
    }
}

bool CommandSender::onCommandAck(const mavlink_message_t& message, std::int64_t nowNs) {
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&message, &ack);
    const std::optional<CommandTarget> target = targetOf(message.compid);
    if (message.sysid != kVehicleSysid || !target || ack.target_system != gcsSysid_ ||
        ack.target_component != kGcsCompid) {
        return false;
    }
    const auto it = pending_.find(Key{*target, ack.command});
    if (it == pending_.end()) {
        return false;  // 이미 끝냈거나 포기한 명령의 늦은 응답
    }
    Pending& pending = it->second;
    const Command command = pending.command.command;
    if (ack.result == MAV_RESULT_IN_PROGRESS) {
        if (!pending.inProgress) {
            log_(gcsLog(LogKind::Reply, QStringLiteral("%1: 진행 중").arg(pending.command.name)));
        }
        pending.inProgress = true;
        pending.lastEventNs = nowNs;
        setStage(command, CommandTrace::Stage::InProgress, nowNs);
        traces_[command].progressPct =
            ack.progress == kUnknownProgress ? -1 : static_cast<std::int32_t>(ack.progress);
        return true;
    }
    const QString result = resultName(ack.result);
    const bool accepted = ack.result == MAV_RESULT_ACCEPTED;
    log_(gcsLog(LogKind::Reply, accepted ? LogLevel::Normal : LogLevel::Serious,
                QStringLiteral("%1: %2").arg(pending.command.name, result)));
    setStage(command, accepted ? CommandTrace::Stage::Accepted : CommandTrace::Stage::Failed, nowNs,
             accepted ? QString() : result);
    pending_.erase(it);
    finished_(command, static_cast<MAV_RESULT>(ack.result));
    return true;
}

bool CommandSender::awaiting(CommandTarget target, std::uint16_t mavCmd) const {
    return pending_.count(Key{target, mavCmd}) > 0;
}

bool CommandSender::awaitingCommand(Command command) const {
    for (const auto& [key, pending] : pending_) {
        if (pending.command.command == command) {
            return true;
        }
    }
    return false;
}

bool CommandSender::awaitingAny(CommandTarget target) const {
    for (const auto& [key, pending] : pending_) {
        if (key.first == target) {
            return true;
        }
    }
    return false;
}

CommandTrace CommandSender::trace(Command command) const {
    const auto found = traces_.find(command);
    return found != traces_.end() ? found->second : CommandTrace{};
}

void CommandSender::finishFailed(const Pending& pending, const QString& result,
                                 std::int64_t nowNs) {
    log_(gcsLog(LogKind::Reply, LogLevel::Serious,
                QStringLiteral("%1: %2").arg(pending.command.name, result)));
    setStage(pending.command.command, CommandTrace::Stage::Failed, nowNs, result);
}

void CommandSender::setStage(Command command, CommandTrace::Stage stage, std::int64_t nowNs,
                             const QString& result) {
    CommandTrace& trace = traces_[command];
    trace.stage = stage;
    trace.stageNs = nowNs;
    trace.result = result;
}

}  // namespace argus::gcs::link
