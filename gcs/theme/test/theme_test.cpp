#include <QtTest>

#include "argus/gcs/theme/motion.hpp"
#include "argus/gcs/theme/theme_tokens.hpp"

namespace argus::gcs::theme {

class ThemeTest : public QObject {
    Q_OBJECT

private slots:
    // 꺼진 조작부 색: 앞 색을 뒤 색 위에 40 % 로 겹친다
    void blendMixesChannels() {
        QCOMPARE(blend(0xffffffU, 0x000000U, 40U), static_cast<QRgb>(0x666666U));
        QCOMPARE(blend(0x000000U, 0xffffffU, 0U), static_cast<QRgb>(0xffffffU));
        QCOMPARE(blend(0x123456U, 0xabcdefU, 100U), static_cast<QRgb>(0x123456U));
    }

    void withAlphaKeepsColor() {
        const QColor color = withAlpha(kCiBlue, 128);
        QCOMPARE(color.rgb() & 0xffffffU, kCiBlue);
        QCOMPARE(color.alpha(), 128);
    }

    void easeInOutIsSymmetricAndClamped() {
        QCOMPARE(motion::easeInOut(-1.0), 0.0);
        QCOMPARE(motion::easeInOut(0.5), 0.5);
        QCOMPARE(motion::easeInOut(2.0), 1.0);
        QVERIFY(qFuzzyCompare(motion::easeInOut(0.25) + motion::easeInOut(0.75), 1.0));
    }

    // 같은 시각이면 같은 각이고, 한 바퀴 뒤 같은 자리로 돌아온다
    void spinTurnRepeatsEachPeriod() {
        QCOMPARE(motion::spinTurn(0), 0.0);
        QCOMPARE(motion::spinTurn(motion::kSpinPeriodMs), 0.0);
        QVERIFY(
            qFuzzyCompare(motion::spinTurn(100), motion::spinTurn(100 + motion::kSpinPeriodMs)));
        QVERIFY(motion::spinTurn(-1) >= 0.0);
    }
};

}  // namespace argus::gcs::theme

QTEST_GUILESS_MAIN(argus::gcs::theme::ThemeTest)
#include "theme_test.moc"
