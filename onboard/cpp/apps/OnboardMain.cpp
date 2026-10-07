#include "argus/application/PerceptionWorker.hpp"
#include <iostream>
#include <unistd.h>
static volatile std::sig_atomic_t interrupted{};
static void stop(int) {
    interrupted = 1;
}
int main(int argc, char **argv) {
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    try {
        auto options = argus::parse_arguments(argc, argv);
        argus::SystemClock clock;
        argus::CompositionRoot root(options, clock);
        if (options.check_config) {
            std::cout << "orchestrator=C++ profile=" << root.profile().name
                      << " frame_source=" << root.source_kind()
                      << " queue_capacity=" << options.queue_capacity
                      << " detector=" << root.options().backend
                      << " capability=perception_only exposure_time=unavailable tracker=unimplemented "
                         "vehicle_port=unimplemented\n";
            return 0;
        }
        zmq::context_t context(1);
        argus::MessageBus bus(context, options.publish_endpoint);
        std::string session = std::to_string(getpid()) + "-" + std::to_string(clock.now().monotonic_ns);
        try {
            bus.diagnostic(root.source_name(), "starting", "C++ CompositionRoot", clock.now());
            argus::PerceptionWorker worker(root, clock, context, bus, session);
            worker.run(interrupted);
            bus.diagnostic(root.source_name(), "stopped", "C++ perception stopped", clock.now());
        } catch (const std::exception &error) {
            try {
                bus.diagnostic(root.source_name(), "fault", error.what(), clock.now());
            } catch (...) {
            }
            throw;
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "argus-onboard: " << error.what() << '\n';
        return 1;
    }
}
