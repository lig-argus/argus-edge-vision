#include "argus/gcs/link/vehicle_link.hpp"

#include <utility>

#include "argus/gcs/link/command_encoder.hpp"
#include "argus/gcs/link/command_policy.hpp"
#include "argus/gcs/link/command_traits.hpp"
#include "argus/gcs/link/link_protocol.hpp"
#include "argus/gcs/link/telemetry_decoder.hpp"

namespace argus::gcs::link {

namespace {

bool isUp(LinkState state) { return state != LinkState::Never && state != LinkState::Lost; }

}  // namespace

VehicleLink::VehicleLink(mavlink_connection::IMavlinkConnection& connection,
                         const core::IClock& clock, std::uint32_t firstCommandNumber,
                         QObject* parent)
    : QObject(parent),
      connection_(connection),
      clock_(clock),
      links_([this](const LogEntry& entry) { log(entry); }),
      sender_(
          connection, firstCommandNumber, [this](const LogEntry& entry) { log(entry); },
          [this](Command command, MAV_RESULT result) { onCommandFinished(command, result); }),
      manual_(connection, [this](const LogEntry& entry) { log(entry); }),
      modeWatch_([this](const LogEntry& entry) { log(entry); }),
      envParams_([this](const mavlink_message_t& message) { connection_.send(message); },
                 [this](const LogEntry& entry) { log(entry); }) {
    connect(&connection_, &mavlink_connection::IMavlinkConnection::messageReceived, this,
            &VehicleLink::onMessage);
    setGcsSysid(kGcsSysidDefault);
}

bool VehicleLink::setGcsSysid(std::uint8_t sysid) {
    if (!isGcsSysid(sysid)) {
        return false;
    }
    state_.gcsSysid = sysid;
    sender_.setGcsSysid(sysid);
    envParams_.setSysid(sysid);
    videoStreams_.setSysid(sysid);
    return true;
}

// ------------------------------------------------------------------------------ 주기 작업

void VehicleLink::tick() {
    const std::int64_t nowNs = now();
    sender_.tick(nowNs);
    updateLinks(nowNs);
    tickVideoStreams(nowNs);
    envParams_.check(nowNs, state_.fcLink.state);
    manual_.tick(manualControlRefusal(), nowNs);
    if (const std::optional<StatusTextLine> line = fcText_.expire(nowNs)) {
        logStatusText({*line}, LogKind::Fc, LogOrigin::Fc);
    }
    if (const std::optional<StatusTextLine> line = gatewayText_.expire(nowNs)) {
        logStatusText({*line}, LogKind::Gateway, LogOrigin::Gateway);
    }
}

// FC 가 처음 연결되거나 두절 뒤 돌아오면(재부팅 등) 비행 환경 파라미터를 다시 읽고, mavlink_gateway
// 가 그러면 영상 접속 정보를 다시 요청한다
void VehicleLink::updateLinks(std::int64_t nowNs) {
    const LinkMonitor::Connected connected = links_.update(state_, nowNs);
    if (connected.fc) {
        envParams_.queueReads(nowNs);
    }
    if (connected.gateway) {
        videoStreams_.begin(nowNs);
    }
}

void VehicleLink::tickVideoStreams(std::int64_t nowNs) {
    if (const std::optional<mavlink_message_t> request = videoStreams_.sendDue(nowNs)) {
        connection_.send(*request);
    }
    if (videoStreams_.check(nowNs) == VideoStreamRequest::Outcome::NoReply) {
        log(gcsLog(LogKind::Gateway, LogLevel::Serious,
                   QStringLiteral("영상 접속 정보 응답 없음: 영상을 열 수 없음")));
    }
}

// ------------------------------------------------------------------------------ 수신

void VehicleLink::onMessage(const mavlink_message_t& message) {
    if (message.sysid != kVehicleSysid) {
        return;
    }
    const std::int64_t nowNs = now();
    links_.onMessage(message, nowNs);
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        onCommandAck(message, nowNs);
    } else if (message.compid == kFcCompid) {
        onFcMessage(message, nowNs);
    } else if (message.compid == kGatewayCompid) {
        onGatewayMessage(message, nowNs);
    }
    updateLinks(nowNs);
}

void VehicleLink::onFcMessage(const mavlink_message_t& message, std::int64_t nowNs) {
    const FcState before = state_.fc;
    if (decodeFcState(state_.fc, message, nowNs)) {
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            modeWatch_.check(before, state_.fc, nowNs);
        }
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t text{};
        mavlink_msg_statustext_decode(&message, &text);
        logStatusText(fcText_.add(text, nowNs), LogKind::Fc, LogOrigin::Fc);
    } else if (message.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
        envParams_.onMessage(message, nowNs);
    }
}

void VehicleLink::onGatewayMessage(const mavlink_message_t& message, std::int64_t nowNs) {
    if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t text{};
        mavlink_msg_statustext_decode(&message, &text);
        logStatusText(gatewayText_.add(text, nowNs), LogKind::Gateway, LogOrigin::Gateway);
        return;
    }
    if (message.msgid == MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION) {
        if (const std::optional<VideoStreams> streams =
                videoStreams_.onStreamInformation(message)) {
            log(gcsLog(LogKind::Gateway,
                       QStringLiteral("영상 접속 정보 수신: 스트림 %1개").arg(streams->size())));
            emit videoStreamsReceived(*streams);
        }
        return;
    }
    decodeGatewayState(state_, message, nowNs);
}

// 명령 절차 밖에서 보낸 영상 접속 정보 요청의 응답도 같은 메시지로 온다
void VehicleLink::onCommandAck(const mavlink_message_t& message, std::int64_t nowNs) {
    if (sender_.onCommandAck(message, nowNs)) {
        return;
    }
    if (videoStreams_.onCommandAck(message) == VideoStreamRequest::Outcome::Denied) {
        log(gcsLog(LogKind::Gateway, LogLevel::Serious,
                   QStringLiteral("영상 접속 정보 요청 거부: 영상을 열 수 없음")));
    }
}

void VehicleLink::onCommandFinished(Command command, MAV_RESULT result) {
    if (command == Command::RebootFc && result == MAV_RESULT_ACCEPTED) {
        // 재부팅 뒤 FC 링크가 돌아오면 파라미터를 다시 읽는다. 그 전 값은 버린다
        envParams_.onRebooting(now());
        log(gcsLog(LogKind::Fc, QStringLiteral("FC 재부팅 중: FC 링크 일시 두절 후 재연결")));
    }
}

void VehicleLink::logStatusText(const std::vector<StatusTextLine>& lines, LogKind kind,
                                LogOrigin origin) {
    for (const StatusTextLine& line : lines) {
        log(LogEntry{kind, line.level, origin, line.text});
    }
}

bool VehicleLink::addDetection(std::uint8_t sensor, DetectionFrame frame) {
    const std::optional<DetectionFrame> stored = detections_.add(sensor, std::move(frame), now());
    if (!stored) {
        return false;
    }
    emit detectionAdded(sensor, *stored);
    return true;
}

// ------------------------------------------------------------------------------ 명령

CommandContext VehicleLink::commandContext() const {
    CommandContext context;
    for (const CommandTraits& traits : allCommandTraits()) {
        if (sender_.awaitingCommand(traits.command)) {
            context.awaiting.push_back(traits.command);
        }
    }
    context.manualOn = manual_.enabled();
    context.manualAxes = manual_.axes();
    context.paramWriteActive = paramWriteActive();
    return context;
}

std::optional<QString> VehicleLink::refusal(Command command, const CommandArgs& args) const {
    const FlightConditions flight = conditions();
    return CommandPolicy(state_, flight, commandContext()).refusal(command, args);
}

std::optional<QString> VehicleLink::specificRefusal(Command command,
                                                    const CommandArgs& args) const {
    const FlightConditions flight = conditions();
    return CommandPolicy(state_, flight, commandContext()).specificRefusal(command, args);
}

std::optional<QString> VehicleLink::request(Command command, const CommandArgs& args) {
    if (std::optional<QString> why = refusal(command, args)) {
        log(gcsLog(LogKind::Command, LogLevel::Serious,
                   QStringLiteral("%1 미전송. %2").arg(commandName(command), *why)));
        return why;
    }
    const std::int64_t nowNs = now();
    const CommandTraits& traits = commandTraits(command);
    if (traits.changesMode) {
        modeWatch_.noteOperatorModeCommand(nowNs);
    }
    EncodeInputs inputs;
    if (traits.target == CommandTarget::Gateway) {
        inputs.commandNumber = sender_.takeNumber();
    }
    inputs.altAmslM = state_.fc.altAmslM;
    inputs.horizontalPositionKnown = conditions().horizontalPositionKnown();
    inputs.gcsSysid = state_.gcsSysid;
    const EncodedCommand encoded = encodeCommand(command, args, inputs);
    for (const QString& note : encoded.notes) {
        log(gcsLog(LogKind::Command, note));
    }
    sender_.send(OutgoingCommand{command, commandName(command), encoded.target, encoded.mavCmd,
                                 encoded.params},
                 nowNs);
    return std::nullopt;
}

// ------------------------------------------------------------------------------ 가상 조종

std::optional<QString> VehicleLink::manualControlRefusal() const {
    const FlightConditions flight = conditions();
    return ManualControl::refusal(flight.fcLinkUp(), flight.rcReceiverActive());
}

std::optional<QString> VehicleLink::specificManualControlRefusal() const {
    if (!isUp(state_.fcLink.state)) {
        return std::nullopt;  // 상단 막대의 링크 표시에 보인다
    }
    return manualControlRefusal();
}

std::optional<QString> VehicleLink::enableManualControl() {
    return manual_.enable(manualControlRefusal(), now());
}

// ------------------------------------------------------------------------------ 비행 환경 파라미터

void VehicleLink::setFlightEnvProfiles(std::vector<ParamProfile> profiles) {
    envParams_.setProfiles(std::move(profiles));
    if (isUp(state_.fcLink.state)) {
        envParams_.queueReads(now());
    }
}

QString VehicleLink::flightEnvText() const { return envParams_.text(state_.fcLink.state, now()); }

std::optional<QString> VehicleLink::flightEnvRefusal(FlightEnv env) const {
    if (std::optional<QString> why = envParams_.writeRefusal(env, state_)) {
        return why;
    }
    if (sender_.awaitingAny(CommandTarget::Fc)) {
        return QStringLiteral("앞서 보낸 FC 명령의 응답 대기 중");
    }
    return std::nullopt;
}

std::optional<QString> VehicleLink::applyFlightEnv(FlightEnv env) {
    if (std::optional<QString> why = flightEnvRefusal(env)) {
        log(gcsLog(LogKind::Command, LogLevel::Serious,
                   QStringLiteral("%1 환경으로 전환하지 않음. %2").arg(flightEnvName(env), *why)));
        return why;
    }
    return envParams_.apply(env, state_, now());
}

}  // namespace argus::gcs::link
