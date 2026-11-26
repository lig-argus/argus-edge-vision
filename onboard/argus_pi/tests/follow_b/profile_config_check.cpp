#include <iostream>

#include "argus/runtime/config/include/bus_following_config.hpp"
int main(int argc, char** argv) {
    try {
        if (argc != 4) {
            return 2;
        }
        for (int i = 1; i < argc; ++i) {
            const auto c = follow::load_bus_following_config(argv[i]);
            if (c.selector.recovery.estimated_speed_deadzone_m_s != .05) {
                return 3;
            }
            if (i > 1 &&
                (!c.guidance.outdoor_profile || c.guidance.predictive_target_height_m != 4.)) {
                return 4;
            }
        }
        std::cout << "three profile loaders preserve 4m outdoor target and share .05m/s deadzone\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
