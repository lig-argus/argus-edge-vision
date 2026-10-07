#pragma once
#include "argus/contracts/Frame.hpp"
#include <vector>
namespace argus {
struct FullBox {
    int class_id{};
    std::string label;
    double confidence{};
    int x1{}, y1{}, x2{}, y2{};
};
struct Observation {
    std::string source, session_id, model;
    std::uint64_t frame_id{}, published_monotonic_ns{}, infer_done_monotonic_ns{};
    unsigned image_width{}, image_height{};
    FrameTiming timing;
    double inference_ms{}, preprocess_ms{}, queue_dwell_ms{}, ipc_ms{};
    std::vector<FullBox> full_boxes;
};
} // namespace argus
