#include "argus/runtime/config/include/tracking_monitor_config.hpp"

#include <set>
#include <sstream>
#include <stdexcept>
namespace follow {
namespace {
long integer(const std::string& value, long low, long high) {
    std::size_t used = 0;
    const auto number = std::stol(value, &used);
    if (used != value.size() || number < low || number > high) {
        throw std::invalid_argument("monitor integer outside allowed range");
    }
    return number;
}
}  // namespace
TrackingMonitorConfig parse_tracking_monitor_options(int argc, char** argv) {
    TrackingMonitorConfig config;
    config.selector.recovery.enabled = true;
    config.selector.recovery.max_gap = std::chrono::milliseconds(4000);
    config.selector.recovery.allow_competitor_departure = true;
    config.selector.use_fitted_ground_position = false;
    config.selector.require_local_monotonic_sample = true;
    std::set<std::string> seen;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help") {
            config.help = true;
            continue;
        }
        if (!seen.insert(key).second || i + 1 >= argc) {
            throw std::invalid_argument("duplicate or missing monitor option: " + key);
        }
        const std::string value = argv[++i];
        if (key == "--input") {
            if (value != "observations" && value != "tracks") {
                throw std::invalid_argument("input must be observations or tracks");
            }
            config.observations_input = value == "observations";
        } else if (key == "--observations") {
            config.observations_endpoint = value;
        } else if (key == "--tracks") {
            config.tracks_endpoint = value;
        } else if (key == "--status") {
            config.status_endpoint = value;
        } else if (key == "--command") {
            config.command_endpoint = value;
        } else if (key == "--max-age-ms") {
            config.selector.max_tracks_age = std::chrono::milliseconds(integer(value, 10, 5000));
        } else if (key == "--max-gap-ms") {
            config.selector.recovery.max_gap =
                std::chrono::milliseconds(integer(value, 100, 10000));
        } else if (key == "--classes") {
            std::stringstream values(value);
            std::string item;
            std::set<std::int32_t> unique;
            while (std::getline(values, item, ',')) {
                const auto id = static_cast<std::int32_t>(integer(item, 0, 1000000));
                if (!unique.insert(id).second) {
                    throw std::invalid_argument("duplicate class id");
                }
                config.class_ids.push_back(id);
            }
            if (config.class_ids.empty()) {
                throw std::invalid_argument("classes must not be empty when specified");
            }
        } else {
            throw std::invalid_argument("unsupported monitoring option: " + key);
        }
    }
    for (const auto& address : {config.observations_endpoint, config.tracks_endpoint,
                                config.status_endpoint, config.command_endpoint}) {
        if (address.empty() || address.size() > 512 ||
            (address.rfind("tcp://", 0) != 0 && address.rfind("ipc://", 0) != 0)) {
            throw std::invalid_argument("monitor endpoints must be tcp:// or ipc://");
        }
    }
    if (config.status_endpoint == config.command_endpoint ||
        config.status_endpoint == config.tracks_endpoint ||
        config.command_endpoint == config.tracks_endpoint) {
        throw std::invalid_argument("tracking/status/command endpoints must differ");
    }
    (void)TargetSelector(config.selector);
    return config;
}
}  // namespace follow
