#pragma once
#include "argus/contracts/Observation.hpp"
namespace argus {
struct DetectionResult {
    std::uint64_t frame_id{}, infer_done_monotonic_ns{};
    std::string session_id, model;
    double preprocess_ms{}, inference_ms{}, ipc_ms{};
    std::vector<FullBox> full_boxes; // original source pixel coordinates, NOT letterbox coordinates.
};
class IDetector {
  public:
    virtual ~IDetector() = default;
    virtual DetectionResult infer(const Frame &) = 0;
};
} // namespace argus
