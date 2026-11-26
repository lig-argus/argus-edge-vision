#include "argus/gcs/rtsp/onvif_metadata.hpp"

#include <QDateTime>
#include <QPointF>
#include <QXmlStreamReader>

namespace argus::gcs::rtsp {

namespace {

constexpr QLatin1StringView kOnvifNs{"http://www.onvif.org/ver10/schema"};
constexpr QLatin1StringView kArgusNs{"urn:argus:metadata:1"};
constexpr std::int64_t kNsPerMs = 1'000'000;

// tt:Transformation: 정규화 좌표 = 프레임 좌표 × Scale + Translate. 없으면 이미 정규화 좌표(−1~1)
struct Transformation {
    double translateX{0.0};
    double translateY{0.0};
    double scaleX{1.0};
    double scaleY{1.0};

    // ONVIF 프레임 좌표(원점 왼쪽 아래, y 위 방향) → 0~1 좌표(원점 왼쪽 위, y 아래 방향)
    QPointF toUnit(double x, double y) const {
        const double normalizedX = x * scaleX + translateX;  // −1~1
        const double normalizedY = y * scaleY + translateY;
        return {(normalizedX + 1.0) / 2.0, 1.0 - (normalizedY + 1.0) / 2.0};
    }
};

double attributeNumber(const QXmlStreamReader& xml, QLatin1StringView name, double fallback) {
    bool ok = false;
    const double value = xml.attributes().value(name).toDouble(&ok);
    return ok ? value : fallback;
}

bool isOnvif(const QXmlStreamReader& xml, QLatin1StringView name) {
    return xml.namespaceUri() == kOnvifNs && xml.name() == name;
}

bool isArgus(const QXmlStreamReader& xml) { return xml.namespaceUri() == kArgusNs; }

// ISO 8601 (예: 2026-10-15T07:15:27.321Z) → 1970년 기준 ns
std::optional<std::int64_t> utcNs(const QString& text) {
    const QDateTime time = QDateTime::fromString(text, Qt::ISODateWithMs);
    if (!time.isValid()) {
        return std::nullopt;
    }
    return static_cast<std::int64_t>(time.toMSecsSinceEpoch()) * kNsPerMs;
}

// argus: 확장 요소의 부호 없는 정수
template <typename T>
std::optional<T> unsignedNumber(const QString& text) {
    bool ok = false;
    const qulonglong value = text.trimmed().toULongLong(&ok);
    return ok ? std::optional<T>(static_cast<T>(value)) : std::nullopt;
}

void readArgusObjectField(QStringView name, const QString& text, TrackedObject& object) {
    if (name == QLatin1StringView("ClassId")) {
        bool ok = false;
        const std::int32_t value = text.trimmed().toInt(&ok);
        if (ok) {
            object.classId = value;
        }
    } else if (name == QLatin1StringView("Label")) {
        object.label = text.trimmed();
    } else if (name == QLatin1StringView("Generation")) {
        object.generation = unsignedNumber<std::uint32_t>(text);
    } else if (name == QLatin1StringView("State")) {
        object.state = text.trimmed();
    }
}

void readArgusFrameField(QStringView name, const QString& text, FrameMetadata& frame) {
    if (name == QLatin1StringView("FrameId")) {
        frame.frameId = unsignedNumber<std::uint64_t>(text);
    } else if (name == QLatin1StringView("ImageWidth")) {
        frame.imageWidth = unsignedNumber<std::uint32_t>(text);
    } else if (name == QLatin1StringView("ImageHeight")) {
        frame.imageHeight = unsignedNumber<std::uint32_t>(text);
    }
}

// 문서를 읽는 동안의 상태. 요소 하나를 읽을 때마다 startElement 를 부른다
class Reader {
public:
    explicit Reader(QXmlStreamReader& xml) : xml_(xml) {}

    // 첫 tt:Frame 이 끝나면 false (한 문서 = 한 프레임)
    bool endElement() {
        if (isOnvif(xml_, QLatin1StringView("Object")) && object_) {
            frame_.objects.push_back(std::move(*object_));
            object_.reset();
        } else if (isOnvif(xml_, QLatin1StringView("Frame"))) {
            return false;
        }
        return true;
    }

    void startElement() {
        if (isOnvif(xml_, QLatin1StringView("Frame")) && !sawFrame_) {
            readFrame();
        } else if (!sawFrame_) {
            return;
        } else if (isOnvif(xml_, QLatin1StringView("Translate"))) {
            transform_.translateX = attributeNumber(xml_, QLatin1StringView("x"), 0.0);
            transform_.translateY = attributeNumber(xml_, QLatin1StringView("y"), 0.0);
        } else if (isOnvif(xml_, QLatin1StringView("Scale"))) {
            transform_.scaleX = attributeNumber(xml_, QLatin1StringView("x"), 1.0);
            transform_.scaleY = attributeNumber(xml_, QLatin1StringView("y"), 1.0);
        } else if (isOnvif(xml_, QLatin1StringView("Object"))) {
            object_ = TrackedObject{};
            object_->objectId = static_cast<std::uint32_t>(
                xml_.attributes().value(QLatin1StringView("ObjectId")).toULongLong());
        } else if (isOnvif(xml_, QLatin1StringView("BoundingBox")) && object_) {
            readBox();
        } else if (isOnvif(xml_, QLatin1StringView("Type")) && object_) {
            object_->likelihood = attributeNumber(xml_, QLatin1StringView("Likelihood"), 0.0);
            object_->type = xml_.readElementText().trimmed();
        } else if (isArgus(xml_)) {
            const QString name = xml_.name().toString();
            const QString text = xml_.readElementText();
            if (object_) {
                readArgusObjectField(name, text, *object_);
            } else {
                readArgusFrameField(name, text, frame_);
            }
        }
    }

    // tt:Frame 과 UtcTime 을 읽었으면 그 프레임
    std::optional<FrameMetadata> result() && {
        if (!sawFrame_ || !utcValid_) {
            return std::nullopt;
        }
        return std::move(frame_);
    }

private:
    void readFrame() {
        sawFrame_ = true;
        frame_.source = xml_.attributes().value(QLatin1StringView("Source")).toString();
        const std::optional<std::int64_t> ns =
            utcNs(xml_.attributes().value(QLatin1StringView("UtcTime")).toString());
        if (ns) {
            frame_.utcNs = *ns;
            utcValid_ = true;
        }
    }

    void readBox() {
        if (!object_) {
            return;
        }
        const QPointF topLeft =
            transform_.toUnit(attributeNumber(xml_, QLatin1StringView("left"), 0.0),
                              attributeNumber(xml_, QLatin1StringView("top"), 0.0));
        const QPointF bottomRight =
            transform_.toUnit(attributeNumber(xml_, QLatin1StringView("right"), 0.0),
                              attributeNumber(xml_, QLatin1StringView("bottom"), 0.0));
        object_->box = QRectF(topLeft, bottomRight).normalized();
    }

    QXmlStreamReader& xml_;
    FrameMetadata frame_;
    Transformation transform_;
    std::optional<TrackedObject> object_;  // 읽는 중인 tt:Object
    bool sawFrame_{false};
    bool utcValid_{false};
};

}  // namespace

std::optional<FrameMetadata> parseOnvifMetadata(const QByteArray& xml) {
    QXmlStreamReader stream(xml);
    Reader reader(stream);
    while (!stream.atEnd()) {
        const QXmlStreamReader::TokenType token = stream.readNext();
        if (token == QXmlStreamReader::EndElement) {
            if (!reader.endElement()) {
                break;
            }
        } else if (token == QXmlStreamReader::StartElement) {
            reader.startElement();
        }
    }
    if (stream.hasError()) {
        return std::nullopt;
    }
    return std::move(reader).result();
}

}  // namespace argus::gcs::rtsp
