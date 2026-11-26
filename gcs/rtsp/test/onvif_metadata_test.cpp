// 메타데이터 트랙의 ONVIF XML 해석. 문서 예와 좌표 변환은 docs/spec/video.md 3.2, 3.3
#include "argus/gcs/rtsp/onvif_metadata.hpp"

#include <QtTest>
#include <cmath>
#include <string_view>

namespace argus::gcs::rtsp {

namespace {

// video.md 3.2 의 예: 640×480, 상자 x 100~150, y 100~180 (영상 픽셀, 원점 왼쪽 위)
constexpr std::string_view kExample =
    "<?xml version='1.0' encoding='UTF-8'?>\n"
    "<tt:MetadataStream xmlns:tt='http://www.onvif.org/ver10/schema' "
    "xmlns:argus='urn:argus:metadata:1'>\n"
    "  <tt:VideoAnalytics>\n"
    "    <tt:Frame UtcTime='2026-10-15T07:15:27.321Z' Source='camera-ir'>\n"
    "      <tt:Transformation>\n"
    "        <tt:Translate x='-1.0' y='-1.0'/>\n"
    "        <tt:Scale x='0.003125' y='0.00416667'/>\n"
    "      </tt:Transformation>\n"
    "      <tt:Object ObjectId='12'>\n"
    "        <tt:Appearance>\n"
    "          <tt:Shape>\n"
    "            <tt:BoundingBox left='100.0' top='380.0' right='150.0' bottom='300.0'/>\n"
    "            <tt:CenterOfGravity x='125.0' y='340.0'/>\n"
    "          </tt:Shape>\n"
    "          <tt:Class>\n"
    "            <tt:Type Likelihood='0.90'>Vehicle</tt:Type>\n"
    "          </tt:Class>\n"
    "        </tt:Appearance>\n"
    "        <tt:Extension>\n"
    "          <argus:ClassId>0</argus:ClassId>\n"
    "          <argus:Label>tank</argus:Label>\n"
    "          <argus:Generation>1</argus:Generation>\n"
    "          <argus:State>CONFIRMED</argus:State>\n"
    "        </tt:Extension>\n"
    "      </tt:Object>\n"
    "      <tt:Extension>\n"
    "        <argus:FrameId>100</argus:FrameId>\n"
    "        <argus:ImageWidth>640</argus:ImageWidth>\n"
    "        <argus:ImageHeight>480</argus:ImageHeight>\n"
    "      </tt:Extension>\n"
    "    </tt:Frame>\n"
    "  </tt:VideoAnalytics>\n"
    "</tt:MetadataStream>\n";

QByteArray example() { return {kExample.data(), static_cast<qsizetype>(kExample.size())}; }

FrameMetadata parsed(const QByteArray& xml) {
    return parseOnvifMetadata(xml).value_or(FrameMetadata{});
}

}  // namespace

class OnvifMetadataTest : public QObject {
    Q_OBJECT

private slots:
    void readsFrameFields() {
        const std::optional<FrameMetadata> frame = parseOnvifMetadata(example());

        QVERIFY(frame.has_value());
        // 2026-10-15T07:15:27.321Z
        QCOMPARE(parsed(example()).utcNs, std::int64_t{1792048527321} * 1'000'000);
        QCOMPARE(parsed(example()).source, QStringLiteral("camera-ir"));
        QCOMPARE(parsed(example()).frameId, std::optional<std::uint64_t>(100));
        QCOMPARE(parsed(example()).imageWidth, std::optional<std::uint32_t>(640));
        QCOMPARE(parsed(example()).imageHeight, std::optional<std::uint32_t>(480));
    }

    void readsObject() {
        const FrameMetadata frame = parsed(example());

        QCOMPARE(frame.objects.size(), std::size_t{1});
        const TrackedObject& object = frame.objects.front();
        QCOMPARE(object.objectId, std::uint32_t{12});
        QCOMPARE(object.likelihood, 0.90);
        QCOMPARE(object.type, QStringLiteral("Vehicle"));
        QCOMPARE(object.classId, std::optional<std::int32_t>(0));
        QCOMPARE(object.label, QStringLiteral("tank"));
        QCOMPARE(object.generation, std::optional<std::uint32_t>(1));
        QCOMPARE(object.state, QStringLiteral("CONFIRMED"));
    }

    // y 를 뒤집은 ONVIF 좌표 → 원점 왼쪽 위 0~1 좌표: x 100/640~150/640, y 100/480~180/480
    void convertsBoxToImageUnits() {
        const QRectF box = parsed(example()).objects.front().box;
        const double tolerance = 1e-4;

        QVERIFY(std::abs(box.left() - 100.0 / 640) < tolerance);
        QVERIFY(std::abs(box.right() - 150.0 / 640) < tolerance);
        QVERIFY(std::abs(box.top() - 100.0 / 480) < tolerance);
        QVERIFY(std::abs(box.bottom() - 180.0 / 480) < tolerance);
    }

    // 추적 결과가 없는 프레임도 보낸다 (메타데이터는 살아 있고 객체만 없음, video.md 3.2)
    void emptyFrameHasNoObjects() {
        const QByteArray xml =
            "<tt:MetadataStream xmlns:tt='http://www.onvif.org/ver10/schema'>\n"
            "  <tt:VideoAnalytics><tt:Frame UtcTime='2026-10-15T07:15:27.354Z' "
            "Source='camera-eo'/></tt:VideoAnalytics>\n"
            "</tt:MetadataStream>\n";

        QVERIFY(parseOnvifMetadata(xml).has_value());
        QCOMPARE(parsed(xml).source, QStringLiteral("camera-eo"));
        QVERIFY(parsed(xml).objects.empty());
        QVERIFY(!parsed(xml).frameId.has_value());
    }

    // Transformation 이 없으면 좌표가 이미 정규화 좌표(−1~1)다
    void withoutTransformationCoordinatesAreNormalized() {
        const QByteArray xml =
            "<tt:MetadataStream xmlns:tt='http://www.onvif.org/ver10/schema'>\n"
            "  <tt:VideoAnalytics><tt:Frame UtcTime='2026-10-15T07:15:27.354Z' "
            "Source='camera-ir'>\n"
            "    <tt:Object ObjectId='3'><tt:Appearance><tt:Shape>\n"
            "      <tt:BoundingBox left='-1.0' top='1.0' right='0.0' bottom='0.0'/>\n"
            "    </tt:Shape></tt:Appearance></tt:Object>\n"
            "  </tt:Frame></tt:VideoAnalytics>\n"
            "</tt:MetadataStream>\n";

        QCOMPARE(parsed(xml).objects.front().box, QRectF(0.0, 0.0, 0.5, 0.5));  // 왼쪽 위 사분면
    }

    // 숫자가 아닌 확장 요소는 없는 것으로 본다
    void ignoresInvalidExtensionNumbers() {
        QByteArray xml = example();
        xml.replace("<argus:ClassId>0<", "<argus:ClassId>tank<");
        xml.replace("<argus:FrameId>100<", "<argus:FrameId>-<");

        QVERIFY(!parsed(xml).objects.front().classId.has_value());
        QVERIFY(!parsed(xml).frameId.has_value());
    }

    void rejectsBrokenDocuments() {
        QVERIFY(!parseOnvifMetadata(QByteArray()).has_value());
        QVERIFY(
            !parseOnvifMetadata(example().left(example().size() / 2)).has_value());  // 잘린 문서
        // UtcTime 이 없으면 영상과 짝지을 수 없다
        const QByteArray withoutUtc =
            "<tt:MetadataStream xmlns:tt='http://www.onvif.org/ver10/schema'>\n"
            "  <tt:VideoAnalytics><tt:Frame Source='camera-ir'/></tt:VideoAnalytics>\n"
            "</tt:MetadataStream>\n";
        QVERIFY(!parseOnvifMetadata(withoutUtc).has_value());
    }
};

}  // namespace argus::gcs::rtsp

QTEST_GUILESS_MAIN(argus::gcs::rtsp::OnvifMetadataTest)
#include "onvif_metadata_test.moc"
