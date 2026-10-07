#include "argus/application/CompositionRoot.hpp"
#include "argus/adapters/I3SdkFrameSource.hpp"
#include "argus/adapters/OpenCvFrameSource.hpp"
#include "argus/adapters/PythonVisionAdapter.hpp"
namespace argus {
CompositionRoot::CompositionRoot(RuntimeOptions options, const IClock &clock)
    : options_(std::move(options)), profile_(load_profile(options_.profiles, options_.profile)),
      clock_(clock) {
    if (profile_.name == "SITL_POSE")
        throw std::invalid_argument("SITL_POSE uses the existing pose application, not this IMAGE2D worker");
    if (options_.backend == "auto") {
        if (profile_.detector != "hailo_yolox")
            throw std::invalid_argument("selected Detector is not implemented; no implicit mock fallback");
        options_.backend = "hailo";
    }
    if (options_.backend != "mock" && options_.backend != "hailo" && options_.backend != "hailo-sync")
        throw std::invalid_argument("unsupported detector backend");
    if (options_.backend != "mock" && options_.hef.empty())
        throw std::invalid_argument("Hailo requires a HEF");
    frame_source_factories["ir_sdk"] = [this] {
        return std::make_unique<I3SdkFrameSource>(clock_, options_.ir_capture, options_.ir_device,
                                                  options_.ir_timeout);
    };
    frame_source_factories["ir_v4l2"] = [this] {
        return std::make_unique<OpenCvFrameSource>(clock_, options_.camera, false, options_.width,
                                                   options_.height, options_.fps, options_.fourcc);
    };
    frame_source_factories["replay"] = [this] {
        return std::make_unique<OpenCvFrameSource>(clock_, options_.camera, true, options_.width,
                                                   options_.height, options_.fps, options_.fourcc);
    };
    if (!frame_source_factories.count(source_kind()))
        throw std::invalid_argument("FrameSource is not implemented: " + source_kind());
}
std::string CompositionRoot::source_kind() const {
    return options_.frame_source == "auto" ? profile_.frame_source : options_.frame_source;
}
std::string CompositionRoot::source_name() const {
    return source_kind() == "ir_sdk"
               ? "ir_sdk:" + options_.ir_capture + ":device=" + std::to_string(options_.ir_device)
               : source_kind() + ":" + options_.camera;
}
std::unique_ptr<IFrameSource> CompositionRoot::create_frame_source() const {
    auto it = frame_source_factories.find(source_kind());
    if (it == frame_source_factories.end())
        throw std::invalid_argument("FrameSource is not implemented: " + source_kind());
    auto source = it->second();
    if (!source)
        throw std::runtime_error("FrameSource factory returned null");
    return source;
}
std::unique_ptr<IDetector> CompositionRoot::create_detector(zmq::context_t &context,
                                                            const std::string &session) const {
    return std::make_unique<PythonVisionAdapter>(context, options_, source_name(), session);
}
template <class T> static auto create_registered(const T &factories, const std::string &name) {
    auto it = factories.find(name);
    if (it == factories.end())
        throw std::invalid_argument("component not implemented: " + name);
    auto result = it->second();
    if (!result)
        throw std::runtime_error("component factory returned null");
    return result;
}
std::unique_ptr<ITracker> CompositionRoot::create_tracker() const {
    return create_registered(tracker_factories, profile_.tracker);
}
std::unique_ptr<ICameraMotion> CompositionRoot::create_camera_motion(const std::string &name) const {
    return create_registered(camera_motion_factories, name);
}
std::unique_ptr<IVehiclePort> CompositionRoot::create_vehicle_port() const {
    return create_registered(vehicle_port_factories, profile_.vehicle_port);
}
} // namespace argus
