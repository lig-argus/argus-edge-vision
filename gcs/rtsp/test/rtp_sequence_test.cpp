// RTP 순번으로 손실, 순서 바뀜, 중복 집계
#include "argus/gcs/rtsp/rtp_sequence.hpp"

#include <QtTest>

namespace argus::gcs::rtsp {

namespace {

constexpr std::uint32_t kSsrc = 42;

}  // namespace

class RtpSequenceTest : public QObject {
    Q_OBJECT

private slots:
    void countsConsecutivePackets() {
        RtpSequenceCounter counter;
        for (std::uint16_t seq = 10; seq < 15; ++seq) {
            counter.push(kSsrc, seq, 0);
        }

        QCOMPARE(counter.stats().received, std::uint64_t{5});
        QCOMPARE(counter.stats().lost, std::uint64_t{0});
        QVERIFY(counter.stats().losses.empty());
    }

    void recordsGap() {
        RtpSequenceCounter counter;
        counter.push(kSsrc, 10, 0);
        counter.push(kSsrc, 14, 5'000);

        QCOMPARE(counter.stats().lost, std::uint64_t{3});
        QCOMPARE(counter.stats().losses.size(), std::size_t{1});
        const RtpLossRecord& record = counter.stats().losses.front();
        QCOMPARE(record.detectedNs, std::int64_t{5'000});
        QCOMPARE(record.lastSeq, std::uint16_t{10});
        QCOMPARE(record.nextSeq, std::uint16_t{14});
        QCOMPARE(record.missing, std::int32_t{3});
    }

    // 손실로 센 패킷이 늦게 오면 순서 바뀜으로 세고 손실에서 뺀다
    void lateArrivalIsReordered() {
        RtpSequenceCounter counter;
        counter.push(kSsrc, 10, 0);
        counter.push(kSsrc, 12, 0);
        counter.push(kSsrc, 11, 0);

        QCOMPARE(counter.stats().lost, std::uint64_t{0});
        QCOMPARE(counter.stats().reordered, std::uint64_t{1});
    }

    void countsDuplicate() {
        RtpSequenceCounter counter;
        counter.push(kSsrc, 10, 0);
        counter.push(kSsrc, 10, 0);

        QCOMPARE(counter.stats().duplicated, std::uint64_t{1});
    }

    void wrapsAround() {
        RtpSequenceCounter counter;
        counter.push(kSsrc, 65535, 0);
        counter.push(kSsrc, 1, 0);

        QCOMPARE(counter.stats().lost, std::uint64_t{1});
    }

    // 송신자 재시작: 큰 건너뜀이나 SSRC 변경은 손실로 세지 않는다
    void restartIsNotLoss() {
        RtpSequenceCounter counter;
        counter.push(kSsrc, 10, 0);
        counter.push(kSsrc, 10 + RtpSequenceCounter::kMaxGap + 1, 0);
        counter.push(kSsrc + 1, 5, 0);
        counter.push(kSsrc + 1, 6, 0);

        QCOMPARE(counter.stats().lost, std::uint64_t{0});
        QCOMPARE(counter.stats().received, std::uint64_t{4});
    }

    void keepsRecentLossRecords() {
        RtpSequenceCounter counter;
        std::uint16_t seq = 0;
        counter.push(kSsrc, seq, 0);
        for (std::size_t i = 0; i < RtpStats::kMaxLossRecords + 3; ++i) {
            seq = static_cast<std::uint16_t>(seq + 2U);
            counter.push(kSsrc, seq, static_cast<std::int64_t>(i));
        }

        QCOMPARE(counter.stats().losses.size(), RtpStats::kMaxLossRecords);
        QCOMPARE(counter.stats().losses.front().detectedNs, std::int64_t{3});
    }
};

}  // namespace argus::gcs::rtsp

QTEST_GUILESS_MAIN(argus::gcs::rtsp::RtpSequenceTest)
#include "rtp_sequence_test.moc"
