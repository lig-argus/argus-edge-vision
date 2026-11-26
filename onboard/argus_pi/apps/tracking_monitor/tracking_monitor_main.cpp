// Pi 검출/외부 tracks → 실제 tracker/TargetManager. 기체 연결과 송신 객체는 구성하지 않는다.
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "argus/commands/target/include/zeromq_target_command_gateway.hpp"
#include "argus/diagnostics/tracking/include/target_monitor_status.hpp"
#include "argus/diagnostics/tracking/include/target_view_codec.hpp"
#include "argus/runtime/config/include/tracking_monitor_config.hpp"
#include "argus/transport/zeromq/include/target_tracks_worker.hpp"
#include "argus/transport/zeromq/include/zeromq_operations.hpp"
#include "argus/transport/zeromq/include/zeromq_tracks_bus.hpp"
namespace {
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }
}  // namespace
int main(int argc, char** argv) {
    try {
        const auto cfg = follow::parse_tracking_monitor_options(argc, argv);
        if (cfg.help) {
            std::cout
                << "tracking_target_monitor --input tracks [--tracks tcp://127.0.0.1:5560] "
                   "[--status tcp://127.0.0.1:5561] [--command tcp://127.0.0.1:5562] [--classes "
                   "0,1] [--max-age-ms 250] [--max-gap-ms 4000]\nSelect/Start/Stop control target "
                   "memory only. No vehicle connection or flight commands.\n";
            return 0;
        }
        follow::ZeroMqContext context;
        follow::TargetSelector selector(cfg.selector);
        auto status = follow::zmq_open_socket(context, ZMQ_PUB, "monitor status PUB");
        const int zero = 0, hwm = 2;
        follow::zmq_set_option(status.get(), ZMQ_LINGER, &zero, sizeof(zero), "monitor PUB linger");
        follow::zmq_set_option(status.get(), ZMQ_SNDHWM, &hwm, sizeof(hwm), "monitor PUB HWM");
        follow::zmq_apply_endpoint(status.get(), cfg.status_endpoint,
                                   follow::BusSocketEndpoint::Bind, "monitor status bind");
        if (cfg.observations_input) {
            throw std::invalid_argument("this build accepts external UCMC tracks only");
        }

        follow::TargetTracksWorker tracks(context, selector, cfg.tracks_endpoint);
        follow::ZeroMqTargetCommandGateway commands(context, selector, cfg.command_endpoint);
        tracks.start();
        commands.start();
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        std::cout << "Tracking/target monitoring ready; flight_control=false; input="
                  << "external real tracks" << std::endl;
        while (!stopping) {
            const auto now = follow::Clock::now();
            selector.advance(now);
            auto selection = selector.snapshot(now);
            const auto payload = follow::encode_target_monitor_status(selection, now);
            if (zmq_send(status.get(), follow::kTargetMonitorTopic,
                         sizeof(follow::kTargetMonitorTopic) - 1, ZMQ_SNDMORE) < 0 ||
                zmq_send(status.get(), payload.data(), payload.size(), 0) < 0) {
                throw std::runtime_error("monitor status publish failed");
            }
            const auto view = follow::build_target_view_snapshot(std::move(selection), now,
                                                                 cfg.selector.max_tracks_age);
            const auto objects = follow::encode_target_view(view);
            if (zmq_send(status.get(), follow::kTargetViewTopic,
                         sizeof(follow::kTargetViewTopic) - 1, ZMQ_SNDMORE) < 0 ||
                zmq_send(status.get(), objects.data(), objects.size(), 0) < 0) {
                throw std::runtime_error("target object view publish failed");
            }
            const auto ts = tracks.stats();
            const auto cs = commands.stats();
            if (!ts.running || !cs.running) {
                throw std::runtime_error("monitor input or command worker stopped");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        (void)selector.execute({"monitor-exit", follow::TargetCommandAction::Stop, 0, 0});
        commands.stop();
        tracks.stop();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Tracking monitor: " << error.what() << '\n';
        return 2;
    }
}
