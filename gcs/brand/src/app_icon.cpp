#include "argus/gcs/brand/app_icon.hpp"

#include <QColor>
#include <QFont>
#include <QPainterPath>
#include <QSvgRenderer>
#include <algorithm>

#include "argus/gcs/theme/theme_tokens.hpp"

namespace argus::gcs::brand {

namespace {

// CI 매뉴얼의 앱 아이콘 그림(한 변 = 1)에서 잰 비율
constexpr double kCorner = 0.09;           // 모서리 반지름
constexpr double kLogoWidth = 0.71;        // LIG 워드마크 폭
constexpr double kTitleCapHeight = 0.075;  // ARGUS 대문자 높이

// 글자 외곽선을 만드는 기준 크기. 외곽선을 대문자 높이에 맞춰 줄이므로 값 자체는 결과에 영향이 없다
constexpr std::int32_t kOutlinePixelSize = 100;

}  // namespace

void paintAppIcon(QPainter& painter, const QRectF& rect, const QString& logoPath) {
    painter.save();
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(rect.topLeft());
    const double s = std::min(rect.width(), rect.height());

    QPainterPath frame;
    frame.addRoundedRect(QRectF(0, 0, s, s), s * kCorner, s * kCorner);
    painter.setClipPath(frame);
    painter.fillRect(QRectF(0, 0, s, s), QColor(theme::kCiBlue));
    painter.setClipping(false);

    // LIG 워드마크는 아이콘 정중앙, ARGUS 는 워드마크 아래 남은 칸의 가운데. 둘 다 가로 가운데
    QSvgRenderer logo(logoPath);
    const QSizeF natural = logo.defaultSize();
    const double logoWidth = s * kLogoWidth;
    const double logoHeight =
        natural.isEmpty() ? 0.0 : natural.height() * logoWidth / natural.width();
    const double logoTop = (s - logoHeight) / 2.0;
    const double logoBottom = logoTop + logoHeight;
    const double titleBaseline = logoBottom + (((s - logoBottom) + (s * kTitleCapHeight)) / 2.0);
    logo.render(&painter, QRectF((s - logoWidth) / 2.0, logoTop, logoWidth, logoHeight));

    // ARGUS: 글자를 외곽선으로 만들어 그린다 (대문자 높이를 정확히 맞추기 위함)
    QFont font(QStringLiteral("Roboto"));
    font.setPixelSize(kOutlinePixelSize);
    font.setBold(true);
    QPainterPath title;
    title.setFillRule(Qt::WindingFill);
    title.addText(0, 0, font, QStringLiteral("ARGUS"));
    const QRectF ink = title.boundingRect();
    if (ink.height() > 0.0) {
        const double k = (s * kTitleCapHeight) / ink.height();
        painter.translate((s / 2.0) - (ink.center().x() * k), titleBaseline);
        painter.scale(k, k);
        painter.translate(0, -ink.bottom());
        painter.fillPath(title, QColor(theme::kCiFuturisticGray));
    }
    painter.restore();
}

}  // namespace argus::gcs::brand
