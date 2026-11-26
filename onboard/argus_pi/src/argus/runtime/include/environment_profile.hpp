#pragma once
#include <string>
namespace argus {
struct EnvironmentProfile {
    std::string name, frame_source, detector, tracker, vehicle_port, coordinate_frame;
};
EnvironmentProfile load_profile(const std::string& path, const std::string& name);
void validate_profile(const EnvironmentProfile&);
struct RuntimeOptions {
    std::string project_root, profiles, profile{"REAL_IR"}, frame_source{"auto"}, backend{"auto"};
    std::string camera{"/dev/video0"}, fourcc{"MJPG"}, python,
        worker_module{"argus.perception.detection.python_vision_worker"}, hef, labels;
    std::string ir_capture{"/home/user/thessen_raw14_viewer/build/i3_raw14_capture"},
        ir_preprocess{"minmax"};
    // 2026-11-25 batch 4: 팀 vision 6374 (bind, loopback 만). 예전 --publish-endpoint (5555
    // connect) 는 뜻이 바뀌어 받지 않는다 (environment_profile.cpp)
    std::string vision_endpoint{"tcp://127.0.0.1:6374"}, latency_csv;
    unsigned width{640}, height{480}, fps{30}, queue_capacity{2}, ir_device{}, ir_timeout{15};
    unsigned worker_timeout_ms{10000}, worker_startup_ms{30000}, log_every{30};
    double score_threshold{0.35}, preview_fps{10}, npu_input_fps{15};
    unsigned jpeg_quality{75}, npu_jpeg_quality{80};
    bool preview{}, npu_input{}, check_config{};
};
RuntimeOptions parse_arguments(int argc, char** argv);
}  // namespace argus
