#include "argus/gcs/link/detection_store.hpp"

#include <algorithm>

namespace argus::gcs::link {

std::optional<DetectionFrame> DetectionStore::add(std::uint8_t sensor, DetectionFrame frame,
                                                  std::int64_t nowNs) {
    if (sensor != ARGUS_SENSOR_IR && sensor != ARGUS_SENSOR_EO) {
        return std::nullopt;
    }
    if (!accept(sensor, frame)) {
        return std::nullopt;
    }
    frame.receivedNs = nowNs;
    record(sensor, frame);
    return frame;
}

bool DetectionStore::accept(std::uint8_t sensor, const DetectionFrame& frame) {
    const std::deque<DetectionFrame>& recent = history(sensor);
    if (recent.empty()) {
        return true;
    }
    const DetectionFrame& last = recent.back();
    if (frame.source == last.source) {
        if (frame.generation < last.generation) {
            return false;
        }
        if (frame.generation == last.generation) {
            return frame.frameId > last.frameId;
        }
    }
    clear(sensor);  // source 가 바뀌었거나 generation 이 커졌다: 새 문맥
    return true;
}

void DetectionStore::record(std::uint8_t sensor, DetectionFrame& frame) {
    std::deque<std::int64_t>& times = times_[sensor];
    times.push_back(frame.receivedNs);
    while (frame.receivedNs - times.front() > kFpsWindowNs) {
        times.pop_front();
    }
    frame.fps = static_cast<double>(times.size());
    std::deque<DetectionFrame>& frames = history_[sensor];
    frames.push_back(frame);
    while (frames.size() > kHistory) {
        frames.pop_front();
    }
}

void DetectionStore::clear(std::uint8_t sensor) {
    history_.erase(sensor);
    times_.erase(sensor);
}

std::optional<DetectionFrame> DetectionStore::latest(std::uint8_t sensor) const {
    const std::deque<DetectionFrame>& frames = history(sensor);
    if (frames.empty()) {
        return std::nullopt;
    }
    return frames.back();
}

std::optional<DetectionFrame> DetectionStore::forFrame(std::uint8_t sensor,
                                                       std::uint32_t frameId) const {
    const std::deque<DetectionFrame>& frames = history(sensor);
    const auto match = std::find_if(frames.rbegin(), frames.rend(),
                                    [frameId](const auto& f) { return f.frameId == frameId; });
    if (match == frames.rend()) {
        return std::nullopt;
    }
    return *match;
}

const std::deque<DetectionFrame>& DetectionStore::history(std::uint8_t sensor) const {
    static const std::deque<DetectionFrame> kNone;
    const auto found = history_.find(sensor);
    return found == history_.end() ? kNone : found->second;
}

}  // namespace argus::gcs::link
