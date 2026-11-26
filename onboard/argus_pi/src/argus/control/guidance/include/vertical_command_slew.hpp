#pragma once
#include <algorithm>
namespace follow {
// NED down m/s, host dt seconds; caller applies existing protection after this limit.
inline double limit_vertical_command_change(double requested, double reference, double dt,
                                            double acceleration, double braking) {
    const double delta = requested - reference;
    const double budget = (delta * reference < 0 ? braking : acceleration) * dt;
    return reference + std::clamp(delta, -budget, budget);
}
}  // namespace follow
