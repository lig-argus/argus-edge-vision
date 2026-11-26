#include <atomic>
#include <csignal>
#include <iostream>
#include <stdexcept>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/perception/geometry/include/pinhole_distortion.hpp"
#include "argus/runtime/follow/include/bus_following_application.hpp"
#ifdef ARGUS_FOLLOW_B
#include <thread>

#include "argus/transport/follow_b/include/zeromq_follow_backend.hpp"
#endif

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
}  // namespace
int main(int argc, char** argv) {
    try {
        std::string moving, yaw, endpoint = "tcp://127.0.0.1:6363", mode = "yaw";
        std::string camera_profile;
        std::map<std::uint32_t, std::string> sensor_sources;
        bool check = false;
        double rate = 10;
        std::chrono::milliseconds timeout{60000};
#ifdef ARGUS_FOLLOW_B
        follow::ZeroMqFollowBackendConfig backend_config;
        endpoint = "tcp://127.0.0.1:6373";
#endif
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            const auto next = [&]() {
                if (++i >= argc) {
                    throw std::invalid_argument("missing " + key);
                }
                return std::string(argv[i]);
            };
            if (key == "--moving-profile") {
                moving = next();
            } else if (key == "--yaw-profile") {
                yaw = next();
            } else if (key == "--operator-endpoint") {
                endpoint = next();
            } else if (key == "--mode") {
                mode = next();
            } else if (key == "--camera-profile") {
                camera_profile = next();
            } else if (key == "--sensor-source") {
                const auto value = next();
                const auto split = value.find('=');
                if (split == std::string::npos || split == 0 || split > 3 ||
                    value.substr(0, split).find_first_not_of("0123456789") != std::string::npos ||
                    value.size() <= split + 1 || value.size() - split - 1 > 128) {
                    throw std::invalid_argument("sensor source must be 1=IR_SOURCE or 2=EO_SOURCE");
                }
                const auto id = std::stoul(value.substr(0, split));
                if ((id != 1 && id != 2) ||
                    !sensor_sources.emplace(id, value.substr(split + 1)).second) {
                    throw std::invalid_argument("sensor source ID outside range or duplicated");
                }
            } else if (key == "--scan-rate") {
                rate = std::stod(next());
            } else if (key == "--scan-timeout-ms") {
                timeout = std::chrono::milliseconds{std::stoll(next())};
            } else if (key == "--check-config") {
                check = true;
#ifdef ARGUS_FOLLOW_B
            } else if (key == "--fc-state-endpoint") {
                backend_config.telemetry_endpoint = next();
            } else if (key == "--legacy-mode-rpc") {
                backend_config.mode_events = false;
            } else if (key == "--onboard-rpc-endpoint") {
                backend_config.rpc_endpoint = next();
            } else if (key == "--control-pub-endpoint") {
                backend_config.control_endpoint = next();
#endif
            } else {
                throw std::invalid_argument("unsupported vMy option: " + key);
            }
        }
        if (moving.empty() || yaw.empty() || (mode != "moving" && mode != "yaw")) {
            throw std::invalid_argument("both profiles and moving/yaw mode are required");
        }
        auto moving_config = follow::load_bus_following_config(moving);
        auto yaw_config = follow::load_bus_following_config(yaw);
        if (!camera_profile.empty()) {
            const auto camera = follow::read_native_json(camera_profile);
            if (camera.value("expected_uid", "") != moving_config.expected_fc_uid) {
                throw std::invalid_argument("camera profile UID mismatch");
            }
            const auto& geometry = camera.at("geometry");
            for (auto* config : {&moving_config, &yaw_config}) {
                auto& calibration = config->geometry;
                calibration.distortion = follow::read_pixel_distortion(geometry);
                calibration.fx = geometry.at("fx");
                calibration.fy = geometry.at("fy");
                calibration.cx = geometry.at("cx");
                calibration.cy = geometry.at("cy");
                calibration.image_width = geometry.at("image_width");
                calibration.image_height = geometry.at("image_height");
                for (unsigned i = 0; i < 9; ++i) {
                    calibration.camera_to_body[i] =
                        geometry.at("camera_to_body").at(i / 3).at(i % 3);
                }
                calibration.camera_offset_body_m =
                    geometry.at("camera_offset_body_m").get<std::array<double, 3>>();
                config->guidance.pid_camera = calibration;
                (void)follow::CalibratedGroundGeometry(calibration);
                (void)follow::FlightGuidanceController(config->guidance);
            }
        }
        follow::BusFollowingApplication app{mode == "moving" ? moving_config : yaw_config,
                                            [] { return stopped != 0; }};
        app.enable_vmy(moving_config, yaw_config, endpoint, rate, timeout, sensor_sources);
        if (check) {
            std::cout << "vMy profiles valid; no FC connection or command\n";
            return 0;
        }
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
#ifdef ARGUS_FOLLOW_B
        backend_config.expected_uid = moving_config.expected_fc_uid;
        follow::ZeroMqFollowBackend backend{backend_config};
        // Observation/selection starts without FC. Writer permission remains in the backend.
        return app.run_with_backend(backend);
#else
        return app.run();
#endif
    } catch (const std::exception& error) {
        std::cerr << "vMy: " << error.what() << '\n';
        return 1;
    }
}
