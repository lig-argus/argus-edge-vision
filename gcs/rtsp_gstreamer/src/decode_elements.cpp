#include "decode_elements.hpp"

#include <array>
#include <utility>

namespace argus::gcs::rtsp_gstreamer {

std::optional<QString> videoDecodeElements(const QString& encodingName) {
    // RTP/H.264 (RFC 6184, 기본), RTP/JPEG (RFC 2435, 송신부 설정으로 고를 수 있음).
    // avdec_h264 는 rtph264depay 의 출력을 그대로 받으므로 h264parse(plugins-bad)를 두지 않는다
    static const std::array<std::pair<QLatin1StringView, QLatin1StringView>, 2> kTable{{
        {QLatin1StringView("H264"), QLatin1StringView("rtph264depay ! avdec_h264")},
        {QLatin1StringView("JPEG"), QLatin1StringView("rtpjpegdepay ! jpegdec")},
    }};
    for (const auto& [name, elements] : kTable) {
        if (encodingName.compare(name, Qt::CaseInsensitive) == 0) {
            return QString(elements);
        }
    }
    return std::nullopt;
}

}  // namespace argus::gcs::rtsp_gstreamer
