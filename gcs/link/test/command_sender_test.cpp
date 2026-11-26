#include "argus/gcs/link/command_sender.hpp"

#include <QtTest>
#include <cmath>
#include <vector>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/gcs/mavlink_connection_testing/fake_mavlink_connection.hpp"

namespace argus::gcs::link {

using mavlink_connection_testing::FakeMavlinkConnection;

namespace {

constexpr std::uint8_t kSysid = 251;
constexpr std::int64_t kMs = kNsPerMs;

OutgoingCommand trackStop(float number) {
    OutgoingCommand command;
    command.command = Command::TrackStop;
    command.name = QStringLiteral("추종 정지");
    command.target = CommandTarget::Gateway;
    command.mavCmd = MAV_CMD_ARGUS_TRACK_STOP;
    command.params = {NAN, NAN, NAN, NAN, NAN, NAN, number};
    return command;
}

OutgoingCommand trackStart(float number) {
    OutgoingCommand command = trackStop(number);
    command.command = Command::TrackStart;
    command.name = QStringLiteral("추종 시작");
    command.mavCmd = MAV_CMD_ARGUS_TRACK_START;
    return command;
}

OutgoingCommand land() {
    OutgoingCommand command;
    command.command = Command::Land;
    command.name = QStringLiteral("LAND");
    command.target = CommandTarget::Fc;
    command.mavCmd = MAV_CMD_NAV_LAND;
    command.params = {0, 0, 0, NAN, NAN, NAN, NAN};
    return command;
}

mavlink_message_t ack(std::uint8_t compid, std::uint16_t command, std::uint8_t result,
                      std::uint8_t targetSystem = kSysid, std::uint8_t progress = 255) {
    mavlink_message_t message{};
    mavlink_msg_command_ack_pack(kVehicleSysid, compid, &message, command, result, progress, 0,
                                 targetSystem, kGcsCompid);
    return message;
}

// 전송 길이를 반영해 읽는다. MAVLink 2 는 끝의 0 바이트를 잘라 보내므로 get 함수로 직접 읽으면 잘린
// 자리의 값이 틀린다
mavlink_command_long_t decode(const mavlink_message_t& message) {
    mavlink_command_long_t command{};
    mavlink_msg_command_long_decode(&message, &command);
    return command;
}

struct Recorder {
    std::vector<LogEntry> logs;
    std::vector<std::pair<Command, MAV_RESULT>> finished;
};

CommandSender makeSender(FakeMavlinkConnection& connection, Recorder& recorder,
                         std::uint32_t firstNumber = 100) {
    CommandSender sender(
        connection, firstNumber,
        [&recorder](const LogEntry& entry) { recorder.logs.push_back(entry); },
        [&recorder](Command command, MAV_RESULT result) {
            recorder.finished.emplace_back(command, result);
        });
    sender.setGcsSysid(kSysid);
    return sender;
}

}  // namespace

class CommandSenderTest : public QObject {
    Q_OBJECT

private slots:
    // 명령 번호는 시작값부터 1 씩 증가하고 2^24 에서 0 으로 돌아간다 (icd 4.1)
    void numbersIncrementAndWrap() {
        FakeMavlinkConnection connection;
        Recorder recorder;
        CommandSender sender = makeSender(connection, recorder, kCommandNumberModulus - 1);

        QCOMPARE(sender.takeNumber(), kCommandNumberModulus - 1);
        QCOMPARE(sender.takeNumber(), 0U);
        QCOMPARE(sender.takeNumber(), 1U);
    }

    // 1 s 마다 같은 값으로 다시 보내고, 3 번 보낸 뒤에도 응답이 없으면 응답 없음 (icd 4.1)
    void retriesWithSameValuesThenGivesUp() {
        FakeMavlinkConnection connection;
        Recorder recorder;
        CommandSender sender = makeSender(connection, recorder);
        sender.send(trackStop(42.0F), 0);

        sender.tick(1'000 * kMs);  // 아직 1 s 를 넘지 않음
        QCOMPARE(connection.sent().size(), 1);
        sender.tick(1'001 * kMs);
        sender.tick(2'002 * kMs);
        QCOMPARE(connection.sent().size(), 3);
        for (std::int32_t i = 0; i < 3; ++i) {
            const mavlink_command_long_t sent = decode(connection.sent()[i]);
            QCOMPARE(sent.param7, 42.0F);
            QCOMPARE(sent.confirmation, static_cast<std::uint8_t>(i));
            QCOMPARE(sent.target_component, kGatewayCompid);
        }

        sender.tick(3'003 * kMs);
        QCOMPARE(connection.sent().size(), 3);
        QVERIFY(!sender.awaitingCommand(Command::TrackStop));
        QCOMPARE(sender.trace(Command::TrackStop).stage, CommandTrace::Stage::Failed);
        QVERIFY(recorder.finished.empty());
    }

    // IN_PROGRESS 를 받으면 재전송을 멈추고, 마지막 IN_PROGRESS 뒤 3 s 동안 응답이 없으면 응답 없음
    void inProgressStopsRetriesAndTimesOut() {
        FakeMavlinkConnection connection;
        Recorder recorder;
        CommandSender sender = makeSender(connection, recorder);
        sender.send(trackStart(7.0F), 0);

        QVERIFY(sender.onCommandAck(
            ack(kGatewayCompid, MAV_CMD_ARGUS_TRACK_START, MAV_RESULT_IN_PROGRESS, kSysid, 40),
            500 * kMs));
        QCOMPARE(sender.trace(Command::TrackStart).stage, CommandTrace::Stage::InProgress);
        QCOMPARE(sender.trace(Command::TrackStart).progressPct, 40);

        sender.tick(2'000 * kMs);  // 재전송하지 않는다
        QCOMPARE(connection.sent().size(), 1);
        QVERIFY(sender.onCommandAck(
            ack(kGatewayCompid, MAV_CMD_ARGUS_TRACK_START, MAV_RESULT_IN_PROGRESS), 3'000 * kMs));
        sender.tick(5'900 * kMs);  // 마지막 IN_PROGRESS 뒤 2.9 s
        QVERIFY(sender.awaitingCommand(Command::TrackStart));
        sender.tick(6'001 * kMs);
        QVERIFY(!sender.awaitingCommand(Command::TrackStart));
        QCOMPARE(sender.trace(Command::TrackStart).stage, CommandTrace::Stage::Failed);
    }

    void finalAckFinishesCommand() {
        FakeMavlinkConnection connection;
        Recorder recorder;
        CommandSender sender = makeSender(connection, recorder);
        sender.send(land(), 0);

        QVERIFY(
            sender.onCommandAck(ack(kFcCompid, MAV_CMD_NAV_LAND, MAV_RESULT_ACCEPTED), 100 * kMs));
        QCOMPARE(sender.trace(Command::Land).stage, CommandTrace::Stage::Accepted);
        QCOMPARE(recorder.finished.size(), static_cast<std::size_t>(1));
        QCOMPARE(recorder.finished[0].second, MAV_RESULT_ACCEPTED);
        QVERIFY(!sender.awaitingAny(CommandTarget::Fc));
    }

    // 다른 통제기, 다른 구성 요소, 다른 명령의 응답은 처리하지 않는다 (icd 4.1)
    void ignoresAckForOthers() {
        FakeMavlinkConnection connection;
        Recorder recorder;
        CommandSender sender = makeSender(connection, recorder);
        sender.send(trackStop(1.0F), 0);

        QVERIFY(!sender.onCommandAck(
            ack(kGatewayCompid, MAV_CMD_ARGUS_TRACK_STOP, MAV_RESULT_ACCEPTED, 252), 0));
        QVERIFY(
            !sender.onCommandAck(ack(kFcCompid, MAV_CMD_ARGUS_TRACK_STOP, MAV_RESULT_ACCEPTED), 0));
        QVERIFY(!sender.onCommandAck(
            ack(kGatewayCompid, MAV_CMD_ARGUS_SCAN_START, MAV_RESULT_ACCEPTED), 0));
        QVERIFY(!sender.onCommandAck(
            ack(kGatewayCompid, MAV_CMD_ARGUS_TRACK_STOP, MAV_RESULT_ACCEPTED, 0), 0));
        QVERIFY(sender.awaitingCommand(Command::TrackStop));
    }

    // 진행 중인 TRACK_START 를 TRACK_STOP 으로 취소할 수 있다. 시작 명령은 CANCELLED 로 끝난다
    void stopCanBeSentWhileStartInProgress() {
        FakeMavlinkConnection connection;
        Recorder recorder;
        CommandSender sender = makeSender(connection, recorder);
        sender.send(trackStart(1.0F), 0);
        sender.onCommandAck(ack(kGatewayCompid, MAV_CMD_ARGUS_TRACK_START, MAV_RESULT_IN_PROGRESS),
                            100 * kMs);
        sender.send(trackStop(2.0F), 200 * kMs);

        QCOMPARE(connection.sent().size(), 2);
        QVERIFY(sender.awaitingCommand(Command::TrackStart));
        QVERIFY(sender.onCommandAck(
            ack(kGatewayCompid, MAV_CMD_ARGUS_TRACK_START, MAV_RESULT_CANCELLED), 300 * kMs));
        QCOMPARE(sender.trace(Command::TrackStart).stage, CommandTrace::Stage::Failed);
        QVERIFY(sender.awaitingCommand(Command::TrackStop));
    }

    // 같은 대상, 같은 MAV_CMD 를 다시 보내면 앞 명령의 대기를 포기한다
    void sameCommandReplacesPending() {
        FakeMavlinkConnection connection;
        Recorder recorder;
        CommandSender sender = makeSender(connection, recorder);
        sender.send(trackStop(1.0F), 0);
        sender.send(trackStop(2.0F), 100 * kMs);

        sender.tick(1'050 * kMs);  // 첫 명령 기준으로는 재전송 시각이지만 대기 중인 것은 두 번째
        QCOMPARE(connection.sent().size(), 2);
        QCOMPARE(decode(connection.sent().back()).param7, 2.0F);
    }
};

}  // namespace argus::gcs::link

QTEST_GUILESS_MAIN(argus::gcs::link::CommandSenderTest)
#include "command_sender_test.moc"
