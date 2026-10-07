#pragma once
#include "argus/ports/ITracker.hpp"
namespace argus {
class ICameraMotion {
  public:
    virtual ~ICameraMotion() = default;
    virtual CameraMotion between(const FrameTiming &, const FrameTiming &) = 0;
};
} // namespace argus
