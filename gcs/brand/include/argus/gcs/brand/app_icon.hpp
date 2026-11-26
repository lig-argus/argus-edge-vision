// 통제기 실행 아이콘. LIG CI 매뉴얼의 앱 아이콘 구성을 따른다: 주색 바탕의 둥근 사각형 가운데에 흰
// LIG 워드마크, 그 아래 칸 가운데에 체계 이름 ARGUS 를 보조색(Futuristic Gray)으로 둔다.
// 벡터로 그린다. 실행 중 창과 작업 표시줄의 아이콘, Windows 실행 파일의 .ico 가 같은 함수로
// 만들어진다
#ifndef ARGUS_GCS_BRAND_APP_ICON_HPP_
#define ARGUS_GCS_BRAND_APP_ICON_HPP_

#include <QPainter>
#include <QRectF>
#include <QString>

namespace argus::gcs::brand {

// LIG 워드마크(흰색 SVG)의 Qt 리소스 경로. 리소스는 실행 파일(app)이 등록한다
inline const QString& logoResourcePath() {
    static const QString kPath = QStringLiteral(":/lig_white.svg");
    return kPath;
}

// rect(정사각형)에 아이콘을 그린다. ARGUS 글자는 앱 글꼴(Roboto Bold)을 등록한 뒤 불러야 같은
// 모양이다. logoPath 는 워드마크 SVG 경로이고, 테스트는 소스 폴더의 파일을 넘긴다
void paintAppIcon(QPainter& painter, const QRectF& rect,
                  const QString& logoPath = logoResourcePath());

}  // namespace argus::gcs::brand

#endif  // ARGUS_GCS_BRAND_APP_ICON_HPP_
