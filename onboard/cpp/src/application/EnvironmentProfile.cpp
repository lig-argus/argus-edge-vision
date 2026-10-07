#include "argus/application/EnvironmentProfile.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <toml++/toml.h>
namespace argus {
EnvironmentProfile load_profile(const std::string &path, const std::string &name) {
    auto config = toml::parse_file(path);
    auto section = config[name].as_table();
    if (!section)
        throw std::invalid_argument("unknown profile: " + name);
    auto text = [&](const char *key) {
        auto value = (*section)[key].value<std::string>();
        if (!value || value->empty())
            throw std::invalid_argument("missing profile field: " + std::string(key));
        return *value;
    };
    EnvironmentProfile profile{name,
                               text("frame_source"),
                               text("detector"),
                               text("tracker"),
                               text("vehicle_port"),
                               text("coordinate_frame")};
    validate_profile(profile);
    return profile;
}
void validate_profile(const EnvironmentProfile &p) {
    std::set<std::string> sources, detectors, ports;
    if (p.name == "REAL_IR") {
        sources = {"ir_v4l2", "ir_sdk"};
        detectors = {"hailo_yolox", "dxm1_yolox"};
        ports = {"mavsdk_udp", "mavsdk_serial"};
    } else if (p.name == "REPLAY") {
        sources = {"replay"};
        detectors = {"pc_yolox", "hailo_yolox", "dxm1_yolox", "none"};
        ports = {"replay"};
    } else if (p.name == "SITL_IMAGE") {
        sources = {"gazebo_camera"};
        detectors = {"pc_yolox", "hailo_yolox", "dxm1_yolox"};
        ports = {"mavsdk_udp"};
    } else if (p.name == "SITL_POSE") {
        sources = {"gazebo_pose"};
        detectors = {"none"};
        ports = {"mavsdk_udp"};
    } else
        throw std::invalid_argument("unsupported profile");
    const std::set<std::string> trackers{"bytetrack_cmc", "ocsort_cmc", "hybridsort_hmiou_off_cmc",
                                         "hybridsort_hmiou_on_cmc"};
    if (!sources.count(p.frame_source) || !detectors.count(p.detector) || !ports.count(p.vehicle_port) ||
        (p.name == "SITL_POSE" ? p.tracker != "pose" : !trackers.count(p.tracker)) ||
        p.coordinate_frame != (p.name == "SITL_POSE" ? "gazebo_world_enu" : "camera_pixel"))
        throw std::invalid_argument("incompatible profile fields");
}
RuntimeOptions parse_arguments(int argc, char **argv) {
    RuntimeOptions o;
    o.project_root =
        std::filesystem::canonical("/proc/self/exe").parent_path().parent_path().parent_path().string();
    o.profiles = o.project_root + "/config/profiles.toml";
    o.python = o.project_root + "/.venv-rpi/bin/python";
    o.labels = o.project_root + "/config/labels.txt";
    for (int i = 1; i < argc; ++i) {
        std::string key = argv[i];
        if (key == "--publish-preview") {
            o.preview = true;
            continue;
        }
        if (key == "--publish-npu-input") {
            o.npu_input = true;
            continue;
        }
        if (key == "--check-config") {
            o.check_config = true;
            continue;
        }
        if (key == "--help") {
            std::cout
                << "argus-onboard --profile REAL_IR --frame-source auto|ir_sdk|ir_v4l2|replay --backend "
                   "auto|hailo|hailo-sync|mock\n"
                << "  --hef PATH --labels PATH --camera DEVICE_OR_REPLAY --publish-preview --preview-max-fps "
                   "10\n"
                << "  --ir-capture PATH --ir-device 0 --ir-timeout 15 --ir-preprocess minmax|fixed14\n"
                << "  --frame-queue-size 2 --worker-timeout-ms 10000 --latency-csv PATH --check-config\n";
            std::exit(0);
        }
        if (++i == argc)
            throw std::invalid_argument("missing option value: " + key);
        std::string v = argv[i];
        auto integer = [&] {
            std::size_t pos;
            auto n = std::stoul(v, &pos);
            if (pos != v.size() || v.empty() || v[0] == '-' || n > 1000000000)
                throw std::invalid_argument("invalid unsigned integer: " + key);
            return unsigned(n);
        };
        auto real = [&] {
            std::size_t pos;
            auto n = std::stod(v, &pos);
            if (pos != v.size() || !std::isfinite(n))
                throw std::invalid_argument("invalid number: " + key);
            return n;
        };
        if (key == "--project-root")
            o.project_root = v;
        else if (key == "--profiles")
            o.profiles = v;
        else if (key == "--profile")
            o.profile = v;
        else if (key == "--frame-source")
            o.frame_source = v;
        else if (key == "--backend")
            o.backend = v;
        else if (key == "--camera")
            o.camera = v;
        else if (key == "--fourcc")
            o.fourcc = v;
        else if (key == "--python")
            o.python = v;
        else if (key == "--worker-module")
            o.worker_module = v;
        else if (key == "--hef")
            o.hef = v;
        else if (key == "--labels")
            o.labels = v;
        else if (key == "--ir-capture")
            o.ir_capture = v;
        else if (key == "--ir-preprocess")
            o.ir_preprocess = v;
        else if (key == "--publish-endpoint")
            o.publish_endpoint = v;
        else if (key == "--latency-csv")
            o.latency_csv = v;
        else if (key == "--width")
            o.width = integer();
        else if (key == "--height")
            o.height = integer();
        else if (key == "--fps")
            o.fps = integer();
        else if (key == "--frame-queue-size")
            o.queue_capacity = integer();
        else if (key == "--ir-device")
            o.ir_device = integer();
        else if (key == "--ir-timeout")
            o.ir_timeout = integer();
        else if (key == "--worker-timeout-ms")
            o.worker_timeout_ms = integer();
        else if (key == "--worker-startup-ms")
            o.worker_startup_ms = integer();
        else if (key == "--log-every")
            o.log_every = integer();
        else if (key == "--jpeg-quality")
            o.jpeg_quality = integer();
        else if (key == "--npu-input-jpeg-quality")
            o.npu_jpeg_quality = integer();
        else if (key == "--score-threshold")
            o.score_threshold = real();
        else if (key == "--preview-max-fps")
            o.preview_fps = real();
        else if (key == "--npu-input-max-fps")
            o.npu_input_fps = real();
        else
            throw std::invalid_argument("unknown argument: " + key);
    }
    if (!o.width || !o.height || o.width > 8192 || o.height > 8192 || !o.fps || !o.queue_capacity ||
        o.queue_capacity > 64 || o.ir_device > 31 || !o.ir_timeout || o.ir_timeout > 300 ||
        !o.worker_timeout_ms || !o.worker_startup_ms || o.jpeg_quality > 100 || o.npu_jpeg_quality > 100 ||
        o.score_threshold < 0 || o.score_threshold > 1 || o.preview_fps <= 0 || o.npu_input_fps <= 0 ||
        (o.ir_preprocess != "minmax" && o.ir_preprocess != "fixed14"))
        throw std::invalid_argument("invalid runtime limits");
    return o;
}
} // namespace argus
