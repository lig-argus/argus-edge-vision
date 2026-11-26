#pragma once
#include <functional>
#include <map>
#include <memory>
#include <zmq.hpp>

#include "argus/perception/camera_capture/include/i_frame_source.hpp"
#include "argus/perception/detection/include/i_detector.hpp"
#include "argus/perception/detection/include/vision_bus_owner.hpp"
#include "argus/runtime/include/environment_profile.hpp"
namespace argus {
class CompositionRoot {
    RuntimeOptions options_;
    EnvironmentProfile profile_;
    const IClock& clock_;

public:
    std::map<std::string, std::function<std::unique_ptr<IFrameSource>()>> frame_source_factories;
    CompositionRoot(RuntimeOptions, const IClock&);
    std::unique_ptr<IFrameSource> create_frame_source() const;
    // vision_bus: worker 미리보기를 받아 6374 로 보내는 소유자 (detector 보다 오래 산다)
    std::unique_ptr<IDetector> create_detector(zmq::context_t&, const std::string& session,
                                               VisionBusOwner& vision_bus) const;
    std::string source_kind() const;
    std::string source_name() const;
    const RuntimeOptions& options() const { return options_; }
    const EnvironmentProfile& profile() const { return profile_; }
};
}  // namespace argus
