#include "argus/gcs/rtsp/frame_synchronizer.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

namespace argus::gcs::rtsp {

namespace {

std::int64_t distanceNs(std::int64_t a, std::int64_t b) { return a > b ? a - b : b - a; }

}  // namespace

FrameSynchronizer::FrameSynchronizer(std::int64_t toleranceNs) : toleranceNs_(toleranceNs) {}

void FrameSynchronizer::addMetadata(FrameMetadata metadata) {
    // 이미 표시한 프레임보다 오래된 메타데이터: 그 프레임은 지나갔다
    if (lastFrameUtcNs_ && metadata.utcNs < *lastFrameUtcNs_ - toleranceNs_) {
        ++droppedMetadata_;
        return;
    }
    const auto at = std::upper_bound(
        held_.begin(), held_.end(), metadata.utcNs,
        [](std::int64_t utcNs, const FrameMetadata& held) { return utcNs < held.utcNs; });
    held_.insert(at, std::move(metadata));
    while (held_.size() > kMaxHeld) {
        held_.pop_front();
        ++droppedMetadata_;
    }
}

std::optional<FrameMetadata> FrameSynchronizer::takeFor(std::int64_t frameUtcNs) {
    lastFrameUtcNs_ = frameUtcNs;
    // 이 프레임의 허용 범위보다 오래된 것은 다시 쓸 일이 없다
    while (!held_.empty() && held_.front().utcNs < frameUtcNs - toleranceNs_) {
        held_.pop_front();
        ++droppedMetadata_;
    }
    auto best = held_.end();
    std::int64_t bestGap = toleranceNs_ + 1;
    for (auto it = held_.begin(); it != held_.end() && it->utcNs <= frameUtcNs + toleranceNs_;
         ++it) {
        const std::int64_t gap = distanceNs(it->utcNs, frameUtcNs);
        if (gap < bestGap) {
            bestGap = gap;
            best = it;
        }
    }
    if (best == held_.end()) {
        ++unmatchedFrames_;
        return std::nullopt;
    }
    FrameMetadata found = std::move(*best);
    // 짝보다 오래된 것도 함께 버린다 (이 프레임 앞의 프레임 것이었다)
    droppedMetadata_ += static_cast<std::uint64_t>(std::distance(held_.begin(), best));
    held_.erase(held_.begin(), std::next(best));
    ++matched_;
    return found;
}

}  // namespace argus::gcs::rtsp
