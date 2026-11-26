// 해독한 프레임의 PTS → 그 프레임의 RTP 타임스탬프
#include "argus/gcs/rtsp/rtp_frame_times.hpp"

#include <QtTest>

namespace argus::gcs::rtsp {

namespace {

constexpr std::uint32_t kSsrc = 7;
constexpr std::uint32_t kFrameTicks = 3'000;  // 30 FPS, 90 kHz

std::optional<std::uint32_t> rtpTimeOf(const std::optional<RtpFrameTime>& time) {
    if (!time) {
        return std::nullopt;
    }
    return time->rtpTime;
}

}  // namespace

class RtpFrameTimesTest : public QObject {
    Q_OBJECT

private slots:
    // 한 프레임의 패킷 여럿은 기록 하나
    void recordsOncePerFrame() {
        RtpFrameTimes times;
        times.recordPacket(100, kSsrc, kFrameTicks);
        times.recordPacket(100, kSsrc, kFrameTicks);
        times.recordPacket(200, kSsrc, 2 * kFrameTicks);

        QCOMPARE(rtpTimeOf(times.takeFrame(100)), std::optional<std::uint32_t>(kFrameTicks));
        QCOMPARE(rtpTimeOf(times.takeFrame(200)), std::optional<std::uint32_t>(2 * kFrameTicks));
        QVERIFY(!times.takeFrame(300).has_value());
    }

    // 수신 시작 직후 지터 버퍼가 몰린 프레임 여럿에 같은 PTS 를 붙여도 들어온 차례대로 꺼낸다
    void samePtsFramesInArrivalOrder() {
        RtpFrameTimes times;
        times.recordPacket(100, kSsrc, kFrameTicks);
        times.recordPacket(100, kSsrc, 2 * kFrameTicks);

        QCOMPARE(rtpTimeOf(times.takeFrame(100)), std::optional<std::uint32_t>(kFrameTicks));
        QCOMPARE(rtpTimeOf(times.takeFrame(100)), std::optional<std::uint32_t>(2 * kFrameTicks));
    }

    // 해독되지 않은 프레임(손실로 해독기가 버림)의 기록은 다음 프레임을 찾을 때 버린다
    void skipsFramesNotDecoded() {
        RtpFrameTimes times;
        times.recordPacket(100, kSsrc, kFrameTicks);
        times.recordPacket(std::nullopt, kSsrc, 2 * kFrameTicks);
        times.recordPacket(300, kSsrc, 3 * kFrameTicks);

        QCOMPARE(rtpTimeOf(times.takeFrame(300)), std::optional<std::uint32_t>(3 * kFrameTicks));
        QVERIFY(!times.takeFrame(100).has_value());
    }

    // 기록이 넘쳐 지워진 프레임은 찾지 못하고, 남은 기록은 그대로 둔다
    void olderThanRecordsIsNotFound() {
        RtpFrameTimes times;
        times.recordPacket(500, kSsrc, kFrameTicks);

        QVERIFY(!times.takeFrame(400).has_value());
        QCOMPARE(rtpTimeOf(times.takeFrame(500)), std::optional<std::uint32_t>(kFrameTicks));
    }

    void keepsBoundedRecords() {
        RtpFrameTimes times;
        for (std::uint32_t i = 0; i < RtpFrameTimes::kMaxRecords + 10U; ++i) {
            times.recordPacket(i, kSsrc, i * kFrameTicks);
        }

        QVERIFY(!times.takeFrame(5).has_value());
        QCOMPARE(rtpTimeOf(times.takeFrame(10)), std::optional<std::uint32_t>(10 * kFrameTicks));
    }
};

}  // namespace argus::gcs::rtsp

QTEST_GUILESS_MAIN(argus::gcs::rtsp::RtpFrameTimesTest)
#include "rtp_frame_times_test.moc"
