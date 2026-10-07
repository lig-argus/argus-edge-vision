#pragma once
#include "argus/ports/IDetector.hpp"
#include <cmath>
namespace argus {
class BoxPostprocessor {
  public:
    void validate(const DetectionResult &result, const Frame &frame, const std::string &session) const {
        if (result.frame_id != frame.frame_id || result.session_id != session)
            throw std::invalid_argument("worker result belongs to another frame/session");
        if (!std::isfinite(result.inference_ms) || result.inference_ms < 0 ||
            !std::isfinite(result.preprocess_ms) || result.preprocess_ms < 0)
            throw std::invalid_argument("invalid worker timing");
        if (result.infer_done_monotonic_ns < frame.timing.received_monotonic_ns)
            throw std::invalid_argument("worker uses an incompatible clock");
        for (const auto &b : result.full_boxes) {
            if (b.class_id < 0 || !std::isfinite(b.confidence) || b.confidence < 0 || b.confidence > 1 ||
                b.x1 < 0 || b.y1 < 0 || b.x2 < b.x1 || b.y2 < b.y1 || b.x2 >= int(frame.width) ||
                b.y2 >= int(frame.height))
                throw std::invalid_argument("invalid source-coordinate full box");
        }
    }
};
} // namespace argus
