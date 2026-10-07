#pragma once
#include "argus/contracts/Track.hpp"
namespace argus {
struct CameraMotion {
    std::uint64_t previous_exposure_ns{}, exposure_ns{};
    double rotation[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
    bool valid{};
};
class ITracker {
  public:
    virtual ~ITracker() = default;
    virtual TrackBatch update(const Observation &, const CameraMotion &) = 0;
};
} // namespace argus
