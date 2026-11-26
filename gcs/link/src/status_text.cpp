#include "argus/gcs/link/status_text.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

namespace argus::gcs::link {

LogLevel statusTextLevel(std::uint8_t severity) {
    if (severity <= MAV_SEVERITY_CRITICAL) {
        return LogLevel::Critical;
    }
    if (severity == MAV_SEVERITY_ERROR) {
        return LogLevel::Serious;
    }
    return severity == MAV_SEVERITY_WARNING ? LogLevel::Caution : LogLevel::Off;
}

std::vector<StatusTextLine> StatusTextAssembler::add(const mavlink_statustext_t& text,
                                                     std::int64_t nowNs) {
    std::vector<StatusTextLine> lines;
    const auto push = [&lines](std::optional<StatusTextLine> line) {
        if (line) {
            lines.push_back(std::move(*line));
        }
    };
    const auto length = static_cast<qsizetype>(
        std::find(std::begin(text.text), std::end(text.text), '\0') - std::begin(text.text));
    const QByteArray chunk(static_cast<const char*>(text.text), length);
    const bool last = length < static_cast<qsizetype>(std::size(text.text));
    // MAVLink 규칙에서 id 0 은 한 조각짜리 문구지만 PX4 는 id 를 0 부터 매겨, 재부팅 뒤 첫 긴
    // 문구도 id 0 으로 나눠 보낸다. 그래서 id 0 이라도 50 자를 꽉 채운 조각은 다음 조각과 모은다
    if (text.id == 0 && last && !(pending_ && pending_->id == 0)) {
        push(flush());
        lines.push_back({QString::fromUtf8(chunk).trimmed(), statusTextLevel(text.severity)});
        return lines;
    }
    // 다른 문구가 시작되거나 순서가 어긋나면 모으던 것을 먼저 낸다
    if (pending_ && (pending_->id != text.id || pending_->nextSeq != text.chunk_seq)) {
        push(flush());
    }
    if (!pending_) {
        pending_ = Pending{text.id, text.chunk_seq, {}, nowNs, text.severity};
    }
    pending_->text += chunk;
    pending_->nextSeq = static_cast<std::uint8_t>(text.chunk_seq + 1U);
    pending_->lastNs = nowNs;
    if (last) {
        push(flush());
    }
    return lines;
}

std::optional<StatusTextLine> StatusTextAssembler::expire(std::int64_t nowNs) {
    if (pending_ && nowNs - pending_->lastNs > kTimeoutNs) {
        return flush();
    }
    return std::nullopt;
}

std::optional<StatusTextLine> StatusTextAssembler::flush() {
    if (!pending_) {
        return std::nullopt;
    }
    StatusTextLine line{QString::fromUtf8(pending_->text).trimmed(),
                        statusTextLevel(pending_->severity)};
    pending_.reset();
    return line;
}

}  // namespace argus::gcs::link
