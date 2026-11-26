#include <unistd.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "argus/perception/include/perception_worker.hpp"
#ifdef ARGUS_WITH_DDS
#include "argus/transport/dds/dds_observation_publisher.hpp"
#endif
static volatile std::sig_atomic_t interrupted{};
static void stop(int) { interrupted = 1; }
int main(int argc, char** argv) {
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    try {
        auto options = argus::parse_arguments(argc, argv);
        argus::SystemClock clock;
        argus::CompositionRoot root(options, clock);
        if (options.check_config) {
            std::cout
                << "orchestrator=C++ profile=" << root.profile().name
                << " frame_source=" << root.source_kind()
                << " queue_capacity=" << options.queue_capacity
                << " detector=" << root.options().backend
                << " capability=perception_only exposure_time=unavailable tracker=unimplemented "
                   "vehicle_port=unimplemented\n";
            return 0;
        }
        zmq::context_t context(1);
        std::string session =
            std::to_string(getpid()) + "-" + std::to_string(clock.now().monotonic_ns);
        // 2026-11-25 batch 4: 6374 (팀 vision, protobuf) 는 이 프로세스가 bind 한다. 소유자
        // 스레드가 팀 ZmqBus 와 발행자 6개를 만들고, worker 미리보기와 아래 관측, 진단을 보낸다.
        // detector 와 PerceptionWorker 보다 오래 산다 (stopped, fault 진단은 detector 를 없앤 뒤
        // 보낸다). bind 하지 못하면 worker 를 띄우기 전에 "vision bus ownership failed: ..." 로
        // 끝낸다
        argus::VisionBusOwner bus(
            context, argus::VisionBusOptions{options.vision_endpoint, session, root.source_name(),
                                             options.profile});
        std::cerr
            << "vision output team bus " << bus.endpoint()
            << " (vision/frame/gray8, vision/overlay/jpeg, vision/npu_input/jpeg, "
               "vision/overlay/h264, perception/observations, diagnostics/events, protobuf)\n";
        // 탐지 결과: ZeroMQ는 항상, DDS는 ARGUS_DDS_PUBLISH=1 일 때 함께 발행한다.
        // DDS 준비(프로파일·참여자) 실패는 탐지·영상을 멈추지 않는다: 경고를 남기고 ZeroMQ만 쓴다.
        // DDS로 받는 추적기는 이때 DEADLINE/LIVELINESS 이벤트로 끊김을 알게 된다.
        std::vector<argus::ObservationPublisher*> observation_targets{&bus};
        std::unique_ptr<argus::ObservationPublisher> dds_observations;
        if (const char* flag = std::getenv("ARGUS_DDS_PUBLISH"); flag && std::string(flag) == "1") {
#ifdef ARGUS_WITH_DDS
            try {
                dds_observations = std::make_unique<argus::DdsObservationPublisher>();
                observation_targets.push_back(dds_observations.get());
            } catch (const std::exception& error) {
                std::cerr << "DDS_OBSERVATION_PUBLISHER_DISABLED " << error.what() << std::endl;
            }
#else
            throw std::runtime_error(
                "ARGUS_DDS_PUBLISH=1 but this build has no DDS (ARGUS_WITH_DDS=OFF)");
#endif
        }
        argus::FanOutObservationPublisher observations(observation_targets);
        try {
            bus.diagnostic(root.source_name(), "starting", "C++ CompositionRoot", clock.now());
            argus::PerceptionWorker worker(root, clock, context, bus, observations, session);
            worker.run(interrupted);
            bus.diagnostic(root.source_name(), "stopped", "C++ perception stopped", clock.now());
        } catch (const std::exception& error) {
            bus.diagnostic(root.source_name(), "fault", error.what(), clock.now());
            throw;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "argus-onboard: " << error.what() << '\n';
        return 1;
    }
}
