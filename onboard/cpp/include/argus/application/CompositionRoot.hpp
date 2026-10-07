#pragma once
#include "argus/application/EnvironmentProfile.hpp"
#include "argus/ports/ICameraMotion.hpp"
#include "argus/ports/IDetector.hpp"
#include "argus/ports/IFrameSource.hpp"
#include "argus/ports/ITracker.hpp"
#include "argus/ports/IVehiclePort.hpp"
#include <functional>
#include <map>
#include <memory>
#include <zmq.hpp>
namespace argus {
class CompositionRoot {
    RuntimeOptions options_;
    EnvironmentProfile profile_;
    const IClock &clock_;

  public:
    std::map<std::string, std::function<std::unique_ptr<IFrameSource>()>> frame_source_factories;
    std::map<std::string, std::function<std::unique_ptr<ITracker>()>> tracker_factories;
    std::map<std::string, std::function<std::unique_ptr<ICameraMotion>()>> camera_motion_factories;
    std::map<std::string, std::function<std::unique_ptr<IVehiclePort>()>> vehicle_port_factories;
    CompositionRoot(RuntimeOptions, const IClock &);
    std::unique_ptr<IFrameSource> create_frame_source() const;
    std::unique_ptr<IDetector> create_detector(zmq::context_t &, const std::string &session) const;
    std::unique_ptr<ITracker> create_tracker() const;
    std::unique_ptr<ICameraMotion> create_camera_motion(const std::string &name) const;
    std::unique_ptr<IVehiclePort> create_vehicle_port() const;
    std::string source_kind() const;
    std::string source_name() const;
    const RuntimeOptions &options() const {
        return options_;
    }
    const EnvironmentProfile &profile() const {
        return profile_;
    }
};
} // namespace argus
