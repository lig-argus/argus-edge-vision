// RTCP Sender Report 로 RTP 타임스탬프 → UTC (docs/spec/video.md 4장)
#include "argus/gcs/rtsp/rtcp_clock.hpp"

#include <QtTest>

namespace argus::gcs::rtsp {

namespace {

constexpr std::uint64_t kNtpToUnixSeconds = 2'208'988'800;
constexpr std::int64_t kNsPerSecond = 1'000'000'000;
constexpr std::uint32_t kSsrc = 0x1234'5678U;

// 1970년 기준 초와 2^32 분수 → NTP 64 비트
constexpr std::uint64_t ntp(std::uint64_t unixSeconds, std::uint32_t fraction) {
    return ((unixSeconds + kNtpToUnixSeconds) << 32U) | fraction;
}

}  // namespace

class RtcpClockTest : public QObject {
    Q_OBJECT

private slots:
    void convertsNtpToUnixNs() {
        QCOMPARE(RtcpClock::ntpToUnixNs(ntp(1'000, 0)), 1'000 * kNsPerSecond);
        // 분수 2^31 = 0.5 초
        QCOMPARE(RtcpClock::ntpToUnixNs(ntp(1'000, 0x8000'0000U)),
                 1'000 * kNsPerSecond + kNsPerSecond / 2);
    }

    void noTimeBeforeFirstReport() {
        const RtcpClock clock;

        QVERIFY(!clock.utcNs(kSsrc, 90'000).has_value());
    }

    // SR 의 RTP 타임스탬프에서 앞뒤로 떨어진 만큼 90 kHz 로 시각을 옮긴다
    void convertsRelativeToReport() {
        RtcpClock clock;
        clock.onSenderReport(kSsrc, ntp(1'000, 0), 900'000);

        QCOMPARE(clock.utcNs(kSsrc, 900'000 + 3'000).value_or(0),
                 1'000 * kNsPerSecond + 33'333'333);  // 3000 틱 = 1 프레임 (30 FPS)
        QCOMPARE(clock.utcNs(kSsrc, 900'000 - 90'000).value_or(0), 999 * kNsPerSecond);
    }

    // RTP 타임스탬프는 32 비트라 한 바퀴 돈 값도 가까운 쪽으로 본다
    void handlesTimestampWraparound() {
        RtcpClock clock;
        clock.onSenderReport(kSsrc, ntp(1'000, 0), 0xFFFF'FFFFU - 89'999U);

        QCOMPARE(clock.utcNs(kSsrc, 90'000).value_or(0), 1'002 * kNsPerSecond);
    }

    // 송신자(SSRC)마다 따로 기억하고, 새 SR 은 앞 SR 을 대신한다
    void keepsLatestReportPerSender() {
        RtcpClock clock;
        clock.onSenderReport(kSsrc, ntp(1'000, 0), 0);
        clock.onSenderReport(kSsrc + 1, ntp(2'000, 0), 0);
        clock.onSenderReport(kSsrc, ntp(1'500, 0), 0);

        QCOMPARE(clock.utcNs(kSsrc, 0).value_or(0), 1'500 * kNsPerSecond);
        QCOMPARE(clock.utcNs(kSsrc + 1, 0).value_or(0), 2'000 * kNsPerSecond);
        QVERIFY(!clock.utcNs(kSsrc + 2, 0).has_value());
    }
};

}  // namespace argus::gcs::rtsp

QTEST_GUILESS_MAIN(argus::gcs::rtsp::RtcpClockTest)
#include "rtcp_clock_test.moc"
