#include "argus/gcs/clock.hpp"

#include <QtTest>

#include "fake_clock.hpp"

namespace argus::gcs {

class ClockTest : public QObject {
    Q_OBJECT

private slots:
    void steadyClockDoesNotGoBackwards() {
        const SteadyClock clock;
        const qint64 first = clock.nowMonotonicNs();
        QTest::qSleep(2);
        const qint64 second = clock.nowMonotonicNs();

        QVERIFY(second >= first);
        QVERIFY(second - first >= 1'000'000);  // 2 ms 잤으니 1 ms 이상 지났다
    }

    void fakeClockStartsAtGivenTimeAndAdvances() {
        FakeClock clock(500);
        QCOMPARE(clock.nowMonotonicNs(), static_cast<qint64>(500));

        clock.advanceNs(1'000);
        QCOMPARE(clock.nowMonotonicNs(), static_cast<qint64>(1'500));
    }

    void fakeClockIgnoresNegativeAdvance() {
        FakeClock clock(100);
        clock.advanceNs(-50);

        QCOMPARE(clock.nowMonotonicNs(), static_cast<qint64>(100));
    }
};

}  // namespace argus::gcs

QTEST_GUILESS_MAIN(argus::gcs::ClockTest)
#include "clock_test.moc"
