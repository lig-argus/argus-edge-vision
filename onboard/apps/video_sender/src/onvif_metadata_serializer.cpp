#include "argus/video_sender/onvif_metadata_serializer.hpp"

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <locale>
#include <optional>
#include <ostream>
#include <sstream>
#include <string_view>
#include <utility>

#include "argus/perception_api/types.hpp"

namespace argus::video_sender {
namespace {

constexpr std::int64_t kNsPerSecond = 1'000'000'000;
constexpr std::int64_t kNsPerMs = 1'000'000;
constexpr std::string_view kStateConfirmed = "CONFIRMED";
constexpr std::string_view kStatePredicted = "PREDICTED";
constexpr std::uint8_t kFirstNonControlCharacter = 0x20;  // 공백 문자
constexpr std::int32_t kCoordinateDecimals = 1;
constexpr std::int32_t kMaxDecimals = 8;
constexpr std::size_t kMinDecimals = 1;

// UtcTime 의 초 단위까지의 형식 (ISO 8601). std::put_time 의 형식 문자열이다
constexpr const char* kDateTimeFormat = "%Y-%m-%dT%H:%M:%S";
constexpr std::int32_t kMillisecondDigits = 3;

// ONVIF 정규화 좌표의 범위 (video.md 3.3). tt:Translate 는 최솟값이고, tt:Scale 은 영상 크기를
// 범위의 길이에 대응시키는 배율이다
constexpr double kNormalizedMin = -1.0;
constexpr double kNormalizedMax = 1.0;
constexpr double kNormalizedLength = kNormalizedMax - kNormalizedMin;

// 문자열을 XML 의 속성 값과 요소 내용에 넣을 수 있는 형태로 출력한다
// - 특수 문자 5개(& < > " ')는 entity 로 변환한다
// - 탭, 줄바꿈, CR 은 문자 참조로 변환한다. 그대로 출력하면 XML 파서가 속성 값 안의 이 문자들을
//   공백으로 변환한다
// - 그 밖의 제어 문자(0x20 미만)는 XML 1.0 문서에 포함할 수 없으므로 출력하지 않는다
struct Escaped {
    std::string_view text;
};

std::ostream& operator<<(std::ostream& out, Escaped escaped) {
    for (const char c : escaped.text) {
        switch (c) {
            case '&':
                out << "&amp;";
                break;
            case '<':
                out << "&lt;";
                break;
            case '>':
                out << "&gt;";
                break;
            case '"':
                out << "&quot;";
                break;
            case '\'':
                out << "&apos;";
                break;
            case '\t':
                out << "&#x9;";
                break;
            case '\n':
                out << "&#xA;";
                break;
            case '\r':
                out << "&#xD;";
                break;
            default:
                if (static_cast<std::uint8_t>(c) >= kFirstNonControlCharacter) {
                    out << c;
                }
                break;
        }
    }
    return out;
}

// 소수 kCoordinateDecimals 자리로 출력한다
struct Coordinate {
    double value;
};

std::ostream& operator<<(std::ostream& out, Coordinate coordinate) {
    return out << std::fixed << std::setprecision(kCoordinateDecimals) << coordinate.value;
}

// 소수 kMaxDecimals 자리로 변환한 뒤 끝의 0 을 삭제한다. 소수 kMinDecimals 자리는 남긴다
// (0.00312500 → 0.003125, 1.00000000 → 1.0)
struct Trimmed {
    double value;
};

std::ostream& operator<<(std::ostream& out, Trimmed trimmed) {
    std::ostringstream digits;
    digits.imbue(std::locale::classic());
    digits << std::fixed << std::setprecision(kMaxDecimals) << trimmed.value;
    std::string text = digits.str();
    // 끝의 0 을 하나씩 삭제한다. 소수 첫째 자리는 0 이어도 남긴다 ("1.00000000" → "1.0")
    const std::size_t min_size = text.find('.') + kMinDecimals + 1;
    while (text.size() > min_size && text.back() == '0') {
        text.pop_back();
    }
    return out << text;
}

// unix_ns 를 "2026-10-15T07:15:27.321Z" 형식으로 출력한다. ms 미만은 절사한다
struct UtcTime {
    std::int64_t unix_ns;
};

std::ostream& operator<<(std::ostream& out, UtcTime time) {
    const auto seconds = static_cast<std::time_t>(time.unix_ns / kNsPerSecond);
    const std::int64_t ms = (time.unix_ns % kNsPerSecond) / kNsPerMs;
    std::tm utc{};
    gmtime_r(&seconds, &utc);
    return out << std::put_time(&utc, kDateTimeFormat) << '.' << std::setfill('0')
               << std::setw(kMillisecondDigits) << ms << 'Z';
}

struct ImageSize {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

ImageSize image_size_of(const CollectedFrame& collected) {
    if (collected.tracks.has_value()) {
        return {collected.tracks.value().image_width, collected.tracks.value().image_height};
    }
    return {collected.frame.width, collected.frame.height};
}

// ONVIF 프레임 좌표(0~pixels)를 정규화 좌표(-1~1)로 변환하는 배율 (video.md 3.3).
// 영상 크기가 0 이면 나눌 수 없으므로 0 을 반환한다
double scale_of(std::uint32_t pixels) {
    if (pixels == 0) {
        return 0.0;
    }
    return kNormalizedLength / static_cast<double>(pixels);
}

// argus:State 에 넣는 문자열 (video.md 3.2). 통제기의 파서가 같은 문자열로 상태를 확인한다.
// TrackState 에 정의되지 않은 값이면 값이 없는 optional 을 반환한다
std::optional<std::string_view> state_name(perception_api::TrackState state) {
    switch (state) {
        case perception_api::TrackState::Confirmed:
            return kStateConfirmed;
        case perception_api::TrackState::Predicted:
            return kStatePredicted;
    }
    return std::nullopt;
}

void write_transformation(std::ostream& out, ImageSize size) {
    out << R"(<tt:Transformation><tt:Translate x=")" << Trimmed{kNormalizedMin} << R"(" y=")"
        << Trimmed{kNormalizedMin} << R"("/><tt:Scale x=")" << Trimmed{scale_of(size.width)}
        << R"(" y=")" << Trimmed{scale_of(size.height)} << R"("/></tt:Transformation>)";
}

// 영상 픽셀 좌표(y 아래 방향)를 ONVIF 프레임 좌표(y 위 방향)로 변환해 출력한다 (video.md 3.3)
void write_shape(std::ostream& out, const perception_api::PixelBox& box, ImageSize size) {
    const auto image_height = static_cast<double>(size.height);
    const double top = image_height - box.y1;
    const double bottom = image_height - box.y2;
    out << R"(<tt:Shape><tt:BoundingBox left=")" << Coordinate{box.x1} << R"(" top=")"
        << Coordinate{top} << R"(" right=")" << Coordinate{box.x2} << R"(" bottom=")"
        << Coordinate{bottom} << R"("/><tt:CenterOfGravity x=")"
        << Coordinate{(box.x1 + box.x2) / 2.0} << R"(" y=")" << Coordinate{(top + bottom) / 2.0}
        << R"("/></tt:Shape>)";
}

void write_object_extension(std::ostream& out, const perception_api::Track& track,
                            std::uint32_t generation) {
    out << "<tt:Extension><argus:ClassId>" << track.classification.class_id << "</argus:ClassId>"
        << "<argus:Label>" << Escaped{track.classification.label} << "</argus:Label>"
        << "<argus:Generation>" << generation << "</argus:Generation>";
    // 정의되지 않은 상태 값을 CONFIRMED 나 PREDICTED 로 출력하지 않도록 요소를 생략한다
    const std::optional<std::string_view> state = state_name(track.state);
    if (state.has_value()) {
        out << "<argus:State>" << state.value() << "</argus:State>";
    }
    out << "</tt:Extension>";
}

void write_frame_extension(std::ostream& out, std::uint64_t frame_id, ImageSize size) {
    out << "<tt:Extension><argus:FrameId>" << frame_id << "</argus:FrameId>"
        << "<argus:ImageWidth>" << size.width << "</argus:ImageWidth>"
        << "<argus:ImageHeight>" << size.height << "</argus:ImageHeight></tt:Extension>";
}

void write_object(std::ostream& out, const perception_api::Track& track, std::uint32_t generation,
                  ImageSize size,
                  const OnvifMetadataSerializer::OnvifTypeByClassId& onvif_type_by_class_id) {
    out << R"(<tt:Object ObjectId=")" << track.track_id << R"("><tt:Appearance>)";
    write_shape(out, track.box, size);
    const auto onvif_type = onvif_type_by_class_id.find(track.classification.class_id);
    if (onvif_type != onvif_type_by_class_id.end()) {
        out << R"(<tt:Class><tt:Type Likelihood=")" << Trimmed{track.classification.confidence}
            << R"(">)" << Escaped{onvif_type->second} << "</tt:Type></tt:Class>";
    }
    out << "</tt:Appearance>";
    write_object_extension(out, track, generation);
    out << "</tt:Object>";
}

}  // namespace

OnvifMetadataSerializer::OnvifMetadataSerializer(OnvifTypeByClassId onvif_type_by_class_id)
    : onvif_type_by_class_id_(std::move(onvif_type_by_class_id)) {}

std::string OnvifMetadataSerializer::serialize(const CollectedFrame& collected) const {
    const ImageSize size = image_size_of(collected);
    std::ostringstream out;
    // 프로세스의 locale 과 무관하게 소수점 문자를 '.' 로, 자릿수 구분 문자 없이 출력한다
    out.imbue(std::locale::classic());
    out << R"(<tt:MetadataStream xmlns:tt="http://www.onvif.org/ver10/schema")"
        << R"( xmlns:argus="urn:argus:metadata:1"><tt:VideoAnalytics>)"
        << R"(<tt:Frame UtcTime=")" << UtcTime{collected.frame.unix_ns} << R"(" Source=")"
        << Escaped{collected.frame.source} << R"(">)";
    write_transformation(out, size);
    if (collected.tracks.has_value()) {
        const perception_api::Tracks& tracks = collected.tracks.value();
        for (const perception_api::Track& track : tracks.tracks) {
            write_object(out, track, tracks.generation, size, onvif_type_by_class_id_);
        }
    }
    write_frame_extension(out, collected.frame.frame_id, size);
    out << "</tt:Frame></tt:VideoAnalytics></tt:MetadataStream>";
    return out.str();
}

}  // namespace argus::video_sender
