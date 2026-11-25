// 영상 송신부 실행 파일 (argus_video_sender)
// 규약: docs/spec/video.md 5장, docs/design/uml/class_video.md 1절
// 입력은 둘 중 하나다.
// - 인자 없음: bus 의 vision/frame/gray8, perception/observations, tracking/tracks (ZeroMQ.
//   docs/spec/process_interfaces.md 5.1). 통신 구현과 실제 시계는 이 파일에서만 생성한다
// - --test-pattern: 카메라와 탐지 프로세스 없이 시험용 영상과 박스
// 영상 트랙의 코덱은 --codec 으로 정한다. 설정 값은 코드의 기본값(protocol/zmq_endpoints.hpp)을
// 사용한다
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <zmq.hpp>

#include "argus/bus_zmq/zmq_bus.hpp"
#include "argus/bus_zmq/zmq_poller.hpp"
#include "argus/core/clock.hpp"
#include "argus/core/event_loop.hpp"
#include "argus/core/poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/perception_api_bus/zmq_codecs.hpp"
#include "argus/protocol/topics.hpp"
#include "argus/protocol/zmq_endpoints.hpp"
#include "argus/video_sender/collected_frame.hpp"
#include "argus/video_sender/frame_collector.hpp"
#include "argus/video_sender/metadata_rtp_packetizer.hpp"
#include "argus/video_sender/onvif_metadata_serializer.hpp"
#include "argus/video_sender/rtsp_server.hpp"
#include "argus/video_sender/test_pattern_frame_source.hpp"
#include "argus/video_sender/test_pattern_scene.hpp"
#include "argus/video_sender/video_bus_input.hpp"
#include "argus/video_sender/video_encode_config.hpp"
#include "argus/video_sender/video_sender.hpp"

namespace {

namespace bus_zmq = argus::bus_zmq;
namespace core = argus::core;
namespace perception_api = argus::perception_api;
namespace protocol = argus::protocol;
namespace video_sender = argus::video_sender;

// 메타데이터 트랙의 RTP payload type. 동적 번호(96~127) 중 video.md 1장과 5장의 값
constexpr std::uint8_t kMetadataPayloadType = 107;

// class_id 와 ONVIF 분류명의 대응 (video.md 8장). 모델이 바뀌면 함께 바뀌는 값이므로 설정 파일을
// 추가할 때 설정으로 옮긴다
constexpr std::int32_t kTankClassId = 0;
constexpr const char* kTankOnvifType = "Vehicle";

constexpr std::chrono::milliseconds kFramePeriod{33};  // 30 FPS (video.md 2장)

// FrameCollector::process_timeouts() 의 호출 간격. 대기 시간(100 ms)이 지난 프레임의 출력이 이
// 간격만큼 늦어질 수 있으므로 대기 시간의 1/10 로 정한다
constexpr std::chrono::milliseconds kTimeoutCheckPeriod{10};

// core::EventLoop 가 process_events() 한 번에 handler 를 호출하는 최대 시간. 주기 작업이 프레임
// 간격(33 ms) 안에 실행되도록 그보다 짧게 정한다
constexpr std::chrono::milliseconds kHandlerBudget{10};

constexpr std::string_view kTestPatternOption = "--test-pattern";
constexpr std::string_view kCodecOption = "--codec";
constexpr std::string_view kCodecJpeg = "jpeg";
constexpr std::string_view kCodecH264 = "h264";

constexpr const char* kLogPrefix = "argus_video_sender: ";

struct Options {
    bool test_pattern = false;
    video_sender::VideoCodec codec = video_sender::VideoCodec::H264;
};

std::optional<video_sender::VideoCodec> codec_of(std::string_view name) {
    if (name == kCodecJpeg) {
        return video_sender::VideoCodec::Jpeg;
    }
    if (name == kCodecH264) {
        return video_sender::VideoCodec::H264;
    }
    return std::nullopt;
}

// 명령행 인자를 해석한다. 정의되지 않은 인자나 값이 있으면 값이 없는 optional 을 반환한다
std::optional<Options> parse_options(std::int32_t argc, char** argv) {
    Options options;
    for (std::int32_t i = 1; i < argc; ++i) {
        const std::string_view argument{argv[i]};
        if (argument == kTestPatternOption) {
            options.test_pattern = true;
        } else if (argument == kCodecOption && i + 1 < argc) {
            const std::optional<video_sender::VideoCodec> codec = codec_of(argv[++i]);
            if (!codec.has_value()) {
                return std::nullopt;
            }
            options.codec = codec.value();
        } else {
            return std::nullopt;
        }
    }
    return options;
}

void print_usage() {
    std::cerr << "사용법: argus_video_sender [" << kTestPatternOption << "] [" << kCodecOption
              << " " << kCodecH264 << "|" << kCodecJpeg << "]\n"
              << "  인자 없음  bus 의 프레임, 탐지 결과, 추적 결과를 송신\n"
              << "  " << kTestPatternOption
              << "  카메라와 탐지 프로세스 없이 시험용 영상과 박스를 송신\n"
              << "  " << kCodecOption << "  영상 트랙의 코덱. 지정하지 않으면 " << kCodecH264
              << "\n";
}

// 등록할 소켓이 없으므로 wait 동안 대기만 한다 (--test-pattern 에서 사용)
class SleepPoller final : public core::IPoller {
public:
    void process_events(std::chrono::milliseconds wait,
                        std::chrono::milliseconds /*budget*/) override {
        std::this_thread::sleep_for(wait);
    }
};

std::int64_t unix_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

// RFC 3550 은 SSRC 와 sequence number 시작값을 무작위로 정하도록 한다
video_sender::MetadataRtpConfig random_metadata_rtp_config() {
    std::random_device random;
    video_sender::MetadataRtpConfig config{kMetadataPayloadType, random()};
    config.first_sequence_number = static_cast<std::uint16_t>(random());
    return config;
}

// 입력과 무관한 송신 쪽: RTSP 서버, 송신부, 메시지 결합. 입력은 호출자가 collector() 에 연결한다.
// clock 은 이 객체보다 오래 유지되어야 한다
class SenderPipeline {
public:
    SenderPipeline(const core::IClock& clock, video_sender::VideoCodec codec)
        : server_config_(make_server_config(codec)),
          server_(clock, server_config_),
          sender_(video_sender::OnvifMetadataSerializer{{{kTankClassId, kTankOnvifType}}}, server_),
          collector_(clock),
          collected_(collector_.subscribe([this](const video_sender::CollectedFrame& collected) {
              sender_.send(collected);
          })) {}

    // RTSP 서버를 연다. 실패하면 false (원인은 RtspServer 가 std::cerr 에 기록한다)
    bool start() {
        if (!server_.start()) {
            return false;
        }
        std::cout << kLogPrefix << "rtsp://<RPi 주소>:" << server_.port() << server_config_.path
                  << "\n";
        return true;
    }

    void stop() { server_.stop(); }

    video_sender::FrameCollector& collector() { return collector_; }

    // 대기 시간이 지난 프레임을 출력하는 주기 작업
    [[nodiscard]] core::PeriodicTimer start_timeout_checks(core::EventLoop& loop) {
        return loop.add_periodic(kTimeoutCheckPeriod, [this] { collector_.process_timeouts(); });
    }

private:
    static video_sender::RtspServerConfig make_server_config(video_sender::VideoCodec codec) {
        video_sender::RtspServerConfig config{random_metadata_rtp_config()};
        config.video = video_sender::make_video_encode_config(codec);
        return config;
    }

    video_sender::RtspServerConfig server_config_;
    video_sender::RtspServer server_;
    video_sender::VideoSender sender_;
    video_sender::FrameCollector collector_;
    core::Subscription collected_;
};

std::int32_t run_with_test_pattern(video_sender::VideoCodec codec) {
    const core::SteadyClock clock;
    SleepPoller poller;
    core::EventLoop loop{poller, clock, kHandlerBudget};
    core::stop_on_signals(&loop);

    SenderPipeline pipeline{clock, codec};
    if (!pipeline.start()) {
        return EXIT_FAILURE;
    }
    video_sender::FrameCollector& collector = pipeline.collector();
    const video_sender::TestPatternConfig pattern;
    video_sender::TestPatternFrameSource source{clock, unix_now_ns, pattern};
    const core::Subscription frames = source.subscribe([&](const perception_api::Frame& frame) {
        collector.on_frame(frame);
        collector.on_observations(video_sender::test_pattern_observations(pattern, frame));
        collector.on_tracks(video_sender::test_pattern_tracks(pattern, frame));
    });
    const core::PeriodicTimer frame_timer =
        loop.add_periodic(kFramePeriod, [&] { source.emit_frame(); });
    const core::PeriodicTimer timeout_timer = pipeline.start_timeout_checks(loop);

    loop.run();
    pipeline.stop();
    return EXIT_SUCCESS;
}

// 영상 송신부가 구독하는 이름의 ZeroMQ 엔드포인트 (process_interfaces.md 5.1). 모두 connect 한다
bus_zmq::ZmqEndpointTable endpoints() {
    const std::string vision{protocol::kDefaultVisionEndpoint};
    return {
        {std::string(protocol::kVisionFrameGray8), vision},
        {std::string(protocol::kPerceptionObservations), vision},
        {std::string(protocol::kTrackingTracks), std::string(protocol::kDefaultTrackingEndpoint)},
    };
}

std::int32_t run_with_bus(video_sender::VideoCodec codec) {
    const core::SteadyClock clock;
    zmq::context_t context;
    bus_zmq::ZmqPoller poller{clock};
    const bus_zmq::ZmqEndpointTable table = endpoints();
    bus_zmq::ZmqBus bus{context, poller, clock, table, argus::perception_api_bus::zmq_codecs()};
    core::EventLoop loop{poller, clock, kHandlerBudget};
    core::stop_on_signals(&loop);

    SenderPipeline pipeline{clock, codec};
    if (!pipeline.start()) {
        return EXIT_FAILURE;
    }
    // pipeline 과 bus 보다 뒤에 선언하여 먼저 소멸시킨다
    const std::unique_ptr<video_sender::VideoBusInput> input =
        video_sender::VideoBusInput::create(bus, pipeline.collector());
    if (!input) {
        pipeline.stop();
        return EXIT_FAILURE;  // 원인은 ZmqBus 가 기록한다
    }
    const core::PeriodicTimer timeout_timer = pipeline.start_timeout_checks(loop);
    for (const auto& [name, endpoint] : table) {
        std::cout << kLogPrefix << name << " " << endpoint << "\n";
    }

    loop.run();
    pipeline.stop();
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::optional<Options> options = parse_options(argc, argv);
        if (!options.has_value()) {
            print_usage();
            return EXIT_FAILURE;
        }
        if (options.value().test_pattern) {
            return run_with_test_pattern(options.value().codec);
        }
        return run_with_bus(options.value().codec);
    } catch (const std::exception& error) {
        std::cerr << kLogPrefix << error.what() << "\n";
    } catch (...) {
        std::cerr << kLogPrefix << "알 수 없는 예외\n";
    }
    return EXIT_FAILURE;
}
