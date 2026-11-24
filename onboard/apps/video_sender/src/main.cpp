// 영상 송신부 실행 파일 (argus_video_sender)
// 규약: docs/spec/video.md 5장, docs/design/uml/class_video.md 1절
// 입력은 --test-pattern (시험용 영상)만 지원한다. bus 의 프레임, 탐지 결과, 추적 결과를 입력으로
// 하는 구성은 메시지의 .proto 확정 후 추가한다. 영상 트랙의 코덱은 --codec 으로 정한다
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <random>
#include <string_view>
#include <thread>

#include "argus/core/event_loop.hpp"
#include "argus/core/poller.hpp"
#include "argus/core/steady_clock.hpp"
#include "argus/core/subscription.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/video_sender/collected_frame.hpp"
#include "argus/video_sender/frame_collector.hpp"
#include "argus/video_sender/metadata_rtp_packetizer.hpp"
#include "argus/video_sender/onvif_metadata_serializer.hpp"
#include "argus/video_sender/rtsp_server.hpp"
#include "argus/video_sender/test_pattern_frame_source.hpp"
#include "argus/video_sender/test_pattern_scene.hpp"
#include "argus/video_sender/video_encode_config.hpp"
#include "argus/video_sender/video_sender.hpp"

namespace {

namespace core = argus::core;
namespace perception_api = argus::perception_api;
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
std::optional<Options> parse_options(int argc, char** argv) {
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
    std::cerr << "사용법: argus_video_sender " << kTestPatternOption << " [" << kCodecOption << " "
              << kCodecH264 << "|" << kCodecJpeg << "]\n"
              << "  " << kTestPatternOption
              << "  카메라와 탐지 프로세스 없이 시험용 영상과 박스를 송신\n"
              << "  " << kCodecOption << "  영상 트랙의 코덱. 지정하지 않으면 " << kCodecH264
              << "\n";
}

// 등록할 소켓이 없으므로 wait 동안 대기만 한다. bus 입력을 추가할 때 bus 의 poller 로 교체한다
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

std::int32_t run_with_test_pattern(video_sender::VideoCodec codec) {
    const core::SteadyClock clock;
    SleepPoller poller;
    core::EventLoop loop{poller, clock, kHandlerBudget};
    core::stop_on_signals(&loop);

    video_sender::RtspServerConfig server_config{random_metadata_rtp_config()};
    server_config.video = video_sender::make_video_encode_config(codec);
    video_sender::RtspServer server{clock, server_config};
    if (!server.start()) {
        return EXIT_FAILURE;  // 원인은 RtspServer 가 std::cerr 에 기록한다
    }

    video_sender::VideoSender sender{
        video_sender::OnvifMetadataSerializer{{{kTankClassId, kTankOnvifType}}}, server};
    video_sender::FrameCollector collector{clock};
    const core::Subscription collected_frames = collector.subscribe(
        [&](const video_sender::CollectedFrame& collected) { sender.send(collected); });

    const video_sender::TestPatternConfig pattern;
    video_sender::TestPatternFrameSource source{clock, unix_now_ns, pattern};
    const core::Subscription frames = source.subscribe([&](const perception_api::Frame& frame) {
        collector.on_frame(frame);
        collector.on_observations(video_sender::test_pattern_observations(pattern, frame));
        collector.on_tracks(video_sender::test_pattern_tracks(pattern, frame));
    });
    const core::PeriodicTimer frame_timer =
        loop.add_periodic(kFramePeriod, [&] { source.emit_frame(); });
    const core::PeriodicTimer timeout_timer =
        loop.add_periodic(kTimeoutCheckPeriod, [&] { collector.process_timeouts(); });

    std::cout << "argus_video_sender: rtsp://<RPi 주소>:" << server.port() << server_config.path
              << "\n";
    loop.run();
    server.stop();
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::optional<Options> options = parse_options(argc, argv);
        if (options.has_value() && options.value().test_pattern) {
            return run_with_test_pattern(options.value().codec);
        }
        print_usage();
    } catch (const std::exception& error) {
        std::cerr << "argus_video_sender: " << error.what() << "\n";
    } catch (...) {
        std::cerr << "argus_video_sender: 알 수 없는 예외\n";
    }
    return EXIT_FAILURE;
}
