// YOLOX observations SUB부터 tracking/tracks·운용 명령·PX4까지를 조립하는 공통 실행부다.
#pragma once

#include <functional>
#include <map>

#include "argus/runtime/config/include/bus_following_config.hpp"

namespace follow {
class IFollowBackend;

class BusFollowingApplication final {
public:
    BusFollowingApplication(BusFollowingConfig, std::function<bool()> interrupted);
    void enable_vmy(BusFollowingConfig moving, BusFollowingConfig yaw, std::string endpoint,
                    double scan_rate, std::chrono::milliseconds scan_timeout,
                    std::map<std::uint32_t, std::string> sensor_sources = {});
    int run();
    int run_with_backend(IFollowBackend& backend);

private:
    BusFollowingConfig config_;
    std::function<bool()> interrupted_;
    bool vmy_enabled_{false};
    BusFollowingConfig moving_profile_, yaw_profile_;
    std::string vmy_endpoint_;
    std::map<std::uint32_t, std::string> sensor_sources_;
    double scan_rate_{10};
    std::chrono::milliseconds scan_timeout_{60000};
};

}  // namespace follow
