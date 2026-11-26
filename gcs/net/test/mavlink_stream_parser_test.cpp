#include "argus/gcs/net/mavlink_stream_parser.hpp"

#include <QtTest>
#include <algorithm>
#include <array>
#include <vector>

#include "argus/gcs/core_testing/fake_clock.hpp"

namespace argus::gcs::net {
namespace {

using core_testing::FakeClock;
using mavlink_connection::LossRecord;
using mavlink_connection::senderKey;
using mavlink_connection::SenderStatistics;

constexpr std::uint8_t kChannel = MAVLINK_COMM_1;
constexpr std::uint32_t kUnknownMsgid = 441;  // 방언에 없는 메시지 번호
constexpr std::int64_t kMsToNs = 1'000'000;

// 송신자와 seq 를 정한 HEARTBEAT 프레임
QByteArray heartbeat(std::uint8_t sysid, std::uint8_t compid, std::uint8_t seq) {
    mavlink_get_channel_status(kChannel)->current_tx_seq = seq;
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack_chan(sysid, compid, kChannel, &message, MAV_TYPE_QUADROTOR,
                                    MAV_AUTOPILOT_PX4, 0, 0, 0);
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const std::uint16_t size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    QByteArray bytes(static_cast<qsizetype>(size), Qt::Uninitialized);
    std::copy_n(buffer.begin(), size, bytes.begin());
    return bytes;
}

// 메시지 번호만 방언에 없는 번호로 바꾼 프레임 (MAVLink 2 헤더의 7, 8 번째 바이트가 msgid 하위)
QByteArray unknownMessage(std::uint8_t sysid, std::uint8_t compid, std::uint8_t seq) {
    QByteArray bytes = heartbeat(sysid, compid, seq);
    bytes[7] = static_cast<char>(kUnknownMsgid & 0xFFU);
    bytes[8] = static_cast<char>((kUnknownMsgid >> 8U) & 0xFFU);
    return bytes;
}

}  // namespace

class MavlinkStreamParserTest : public QObject {
    Q_OBJECT

private slots:
    // 두 송신자가 섞여도 각자의 seq 로 센다
    void countsLossPerSender() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        std::int32_t messages = 0;
        const auto count = [&messages](const mavlink_message_t&) { ++messages; };
        parser.feed(heartbeat(1, 1, 10), count);
        parser.feed(heartbeat(1, 195, 200), count);
        parser.feed(heartbeat(1, 1, 11), count);
        parser.feed(heartbeat(1, 195, 203), count);  // 201, 202 손실
        parser.feed(heartbeat(1, 1, 12), count);

        QCOMPARE(messages, 5);
        const SenderStatistics& fc = parser.statistics().senders.at(senderKey(1, 1));
        const SenderStatistics& gateway = parser.statistics().senders.at(senderKey(1, 195));
        QCOMPARE(fc.received, 3U);
        QCOMPARE(fc.lost, 0U);
        QCOMPARE(gateway.received, 2U);
        QCOMPARE(gateway.lost, 2U);
    }

    void seqWrapsAround() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        const auto ignore = [](const mavlink_message_t&) {};
        parser.feed(heartbeat(1, 1, 254), ignore);
        parser.feed(heartbeat(1, 1, 255), ignore);
        parser.feed(heartbeat(1, 1, 0), ignore);  // 255 → 0 은 정상 진행
        parser.feed(heartbeat(1, 1, 2), ignore);  // 1 손실

        const SenderStatistics& fc = parser.statistics().senders.at(senderKey(1, 1));
        QCOMPARE(fc.lost, 1U);
        QCOMPARE(fc.reordered, 0U);
        QCOMPARE(fc.duplicated, 0U);
    }

    // 늦게 도착한 패킷은 순서 바뀜이고 손실에서 빠진다. 이미 받은 seq 는 중복
    void separatesReorderedAndDuplicated() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        const auto ignore = [](const mavlink_message_t&) {};
        for (const std::uint8_t seq : {10, 12, 11, 13, 13, 12, 14}) {
            parser.feed(heartbeat(1, 1, seq), ignore);
        }

        const SenderStatistics& fc = parser.statistics().senders.at(senderKey(1, 1));
        QCOMPARE(fc.received, 7U);
        QCOMPARE(fc.lost, 0U);
        QCOMPARE(fc.reordered, 1U);
        QCOMPARE(fc.duplicated, 2U);
    }

    // 송신자 재시작(seq 가 0 부터 다시): 뒤로 간 패킷이 연속으로 오면 다시 맞추고 그동안 센 값을
    // 되돌린다
    void resynchronizesAfterSenderRestart() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        const auto ignore = [](const mavlink_message_t&) {};
        for (std::int32_t seq = 40; seq <= 60; ++seq) {
            parser.feed(heartbeat(1, 1, static_cast<std::uint8_t>(seq)), ignore);
        }
        for (std::int32_t seq = 0; seq <= 5; ++seq) {
            parser.feed(heartbeat(1, 1, static_cast<std::uint8_t>(seq)), ignore);
        }

        const SenderStatistics& fc = parser.statistics().senders.at(senderKey(1, 1));
        QCOMPARE(fc.lost, 0U);
        QCOMPARE(fc.reordered, 0U);
        QCOMPARE(fc.duplicated, 0U);
        QCOMPARE(fc.lastSeq, static_cast<std::uint8_t>(5));
    }

    void joinsSplitDataAndMultipleMessages() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        std::vector<std::uint8_t> seqs;
        const auto collect = [&seqs](const mavlink_message_t& message) {
            seqs.push_back(message.seq);
        };
        const QByteArray first = heartbeat(1, 1, 1);
        parser.feed(first.left(5), collect);
        QVERIFY(seqs.empty());

        parser.feed(first.mid(5) + heartbeat(1, 1, 2), collect);  // 나머지 + 다음 메시지
        QCOMPARE(seqs, (std::vector<std::uint8_t>{1, 2}));
    }

    void countsBadCrcAndUnknownMessages() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        std::int32_t messages = 0;
        const auto count = [&messages](const mavlink_message_t&) { ++messages; };
        QByteArray broken = heartbeat(1, 1, 1);
        // 본문 1 바이트 변조
        broken[12] = static_cast<char>(static_cast<std::uint8_t>(broken.at(12)) ^ 0x01U);
        parser.feed(broken, count);
        parser.feed(unknownMessage(1, 1, 2), count);

        QCOMPARE(messages, 0);
        QCOMPARE(parser.statistics().badCrc, 1U);
        QCOMPARE(parser.statistics().unknown, 1U);
        // 방언에 없는 메시지도 송신자 seq 에는 반영한다 (다음 메시지에서 손실로 세지 않게)
        parser.feed(heartbeat(1, 1, 3), count);
        QCOMPARE(parser.statistics().senders.at(senderKey(1, 1)).lost, 0U);
    }

    // 메시지 번호별 수신 수와 프레임 바이트
    void countsPerMessageAndBytes() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        const auto ignore = [](const mavlink_message_t&) {};
        const QByteArray frame = heartbeat(1, 1, 1);
        parser.feed(frame, ignore);
        parser.feed(heartbeat(1, 1, 2), ignore);
        parser.feed(unknownMessage(1, 1, 3), ignore);

        const SenderStatistics& fc = parser.statistics().senders.at(senderKey(1, 1));
        const auto frameSize = static_cast<std::uint64_t>(frame.size());
        QCOMPARE(fc.bytes, 3 * frameSize);
        QCOMPARE(fc.messages.size(), static_cast<std::size_t>(2));
        QCOMPARE(fc.messages.at(MAVLINK_MSG_ID_HEARTBEAT).received, 2U);
        QCOMPARE(fc.messages.at(MAVLINK_MSG_ID_HEARTBEAT).bytes, 2 * frameSize);
        QCOMPARE(fc.messages.at(kUnknownMsgid).received, 1U);
    }

    // 손실 기록: 시각, 건너뛴 seq 범위, 빠진 수, 앞뒤 메시지. 늦게 도착한 패킷은 그 기록에 반영
    void recordsWhereLossHappened() {
        FakeClock clock(1'000 * kMsToNs);
        MavlinkStreamParser parser(clock);
        const auto ignore = [](const mavlink_message_t&) {};
        parser.feed(heartbeat(1, 1, 10), ignore);
        clock.advanceNs(1'500 * kMsToNs);
        parser.feed(unknownMessage(1, 1, 14), ignore);  // 11, 12, 13 손실

        const SenderStatistics& fc = parser.statistics().senders.at(senderKey(1, 1));
        QCOMPARE(fc.losses.size(), static_cast<std::size_t>(1));
        const LossRecord& record = fc.losses.front();
        QCOMPARE(record.detectedNs, 2'500 * kMsToNs);
        QCOMPARE(record.lastSeq, static_cast<std::uint8_t>(10));
        QCOMPARE(record.nextSeq, static_cast<std::uint8_t>(14));
        QCOMPARE(record.missing, 3);
        QCOMPARE(record.late, 0);
        QCOMPARE(record.beforeMsgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
        QCOMPARE(record.afterMsgid, kUnknownMsgid);
        QCOMPARE(fc.messages.at(MAVLINK_MSG_ID_HEARTBEAT).lastReceivedNs, 1'000 * kMsToNs);
        QCOMPARE(fc.messages.at(kUnknownMsgid).lastReceivedNs, 2'500 * kMsToNs);

        parser.feed(heartbeat(1, 1, 12), ignore);  // 12 가 늦게 도착
        QCOMPARE(fc.losses.front().late, 1);
        QCOMPARE(fc.lost, 2U);
        parser.feed(heartbeat(1, 1, 15), ignore);  // 정상 진행은 기록하지 않는다
        QCOMPARE(fc.losses.size(), static_cast<std::size_t>(1));
    }

    // 송신자마다 최근 kMaxLossRecords 개만 보관한다
    void keepsRecentLossRecordsOnly() {
        FakeClock clock;
        MavlinkStreamParser parser(clock);
        const auto ignore = [](const mavlink_message_t&) {};
        constexpr std::int32_t kGaps = 60;
        for (std::int32_t i = 0; i <= kGaps; ++i) {
            if (i > 0) {
                clock.advanceNs(1);
            }
            parser.feed(heartbeat(1, 1, static_cast<std::uint8_t>(i * 2)), ignore);  // 매번 1 손실
        }

        const SenderStatistics& fc = parser.statistics().senders.at(senderKey(1, 1));
        QCOMPARE(fc.lost, static_cast<std::uint64_t>(kGaps));
        QCOMPARE(fc.losses.size(), SenderStatistics::kMaxLossRecords);
        QCOMPARE(fc.losses.front().detectedNs,
                 static_cast<std::int64_t>(kGaps) -
                     static_cast<std::int64_t>(SenderStatistics::kMaxLossRecords) + 1);
        QCOMPARE(fc.losses.back().detectedNs, static_cast<std::int64_t>(kGaps));
    }
};

}  // namespace argus::gcs::net

QTEST_GUILESS_MAIN(argus::gcs::net::MavlinkStreamParserTest)
#include "mavlink_stream_parser_test.moc"
