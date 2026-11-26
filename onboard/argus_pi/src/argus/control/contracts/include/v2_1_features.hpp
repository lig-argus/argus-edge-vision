#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "argus/common/time/include/time.hpp"

namespace follow {
constexpr std::int64_t kV21CvWindowNs = 3000000000LL;

inline bool v21_enabled(const char* name) {
    const char* gate = std::getenv("ARGUS_V21_ENABLED");
    const char* value = std::getenv(name);
    return gate && value && std::strcmp(gate, "1") == 0 && std::strcmp(value, "1") == 0;
}

inline double v21_vertical_box_center(double anchor, double anchor_fraction,
                                      double predicted_height) {
    return anchor + (.5 - anchor_fraction) * predicted_height;
}
}  // namespace follow
