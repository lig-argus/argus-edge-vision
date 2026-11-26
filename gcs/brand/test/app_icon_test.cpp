#include "argus/gcs/brand/app_icon.hpp"

#include <QImage>
#include <QtTest>

#include "argus/gcs/theme/theme_tokens.hpp"

namespace argus::gcs::brand {

namespace {

constexpr std::int32_t kSize = 256;

QString logoFile() { return QStringLiteral(ARGUS_GCS_RESOURCES_DIR "/lig_white.svg"); }

QImage paintIcon() {
    QImage image(kSize, kSize, QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);
    QPainter painter(&image);
    paintAppIcon(painter, QRectF(0, 0, kSize, kSize), logoFile());
    painter.end();
    return image;
}

}  // namespace

class AppIconTest : public QObject {
    Q_OBJECT

private slots:
    void logoFileExists() { QVERIFY(QFile::exists(logoFile())); }

    // 바탕은 주색이고 둥근 모서리 밖은 투명하다
    void fillsRoundedSquareWithCiBlue() {
        const QImage icon = paintIcon();
        QCOMPARE(icon.pixelColor(kSize / 2, 8).rgb() & 0xffffffU, theme::kCiBlue);
        QCOMPARE(icon.pixelColor(0, 0).alpha(), 0);
    }

    // 가운데 가로줄에 흰 워드마크가 그려진다
    void drawsWhiteLogoInTheMiddle() {
        const QImage icon = paintIcon();
        bool white = false;
        for (std::int32_t x = 0; x < kSize && !white; ++x) {
            white = icon.pixelColor(x, kSize / 2).lightness() > 240;
        }
        QVERIFY(white);
    }
};

}  // namespace argus::gcs::brand

QTEST_MAIN(argus::gcs::brand::AppIconTest)
#include "app_icon_test.moc"
