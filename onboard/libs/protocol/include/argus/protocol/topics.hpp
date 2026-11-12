// RPi 안 토픽 이름, 요청-응답 이름, 토픽별 큐 길이(queue_depth) 기본값. 코드에서 이름과 숫자를 직접
// 사용하지 않고 이 상수를 사용한다. 발행하는 쪽과 구독하는 쪽이 같은 상수를 읽으므로 두 쪽의 값이
// 항상 같다. 규약: docs/spec/process_interfaces.md 5.1 (토픽), 5.2 (요청-응답). 규약이 바뀌면 이
// 파일도 함께 바꾼다. 이름 규칙: "<그룹>/<이름>". 통신 구현은 이름을 그 통신 라이브러리가 허용하는
// 형식으로 변환할 수 있다 (bus/bus.hpp contract). 탐지(Python)는 이 헤더를 사용하지 못하므로 같은
// 이름을 따로 정의한다.
#ifndef ARGUS_PROTOCOL_TOPICS_HPP_
#define ARGUS_PROTOCOL_TOPICS_HPP_

#include <string_view>

namespace argus::protocol {

// 큐 길이 기본값. 한 프레임에 메시지가 최대 3개(gray8, overlay, npu_input) 연달아 나오므로 4
// (규약 5.1). 실제 값은 각 실행 파일이 설정 파일에서 읽고, 없을 때만 이 값을 사용한다.
inline constexpr int kDefaultQueueDepth = 4;

// 토픽 (발행 → 구독)
// 탐지 → 영상 송신부
inline constexpr std::string_view kVisionFrameGray8 = "vision/frame/gray8";
inline constexpr int kVisionFrameGray8QueueDepth = kDefaultQueueDepth;
// 탐지 → 지상 미리보기
inline constexpr std::string_view kVisionOverlayJpeg = "vision/overlay/jpeg";
inline constexpr int kVisionOverlayJpegQueueDepth = kDefaultQueueDepth;
// 탐지 → (진단용, 구독자 없음)
inline constexpr std::string_view kVisionNpuInputJpeg = "vision/npu_input/jpeg";
inline constexpr int kVisionNpuInputJpegQueueDepth = kDefaultQueueDepth;
// 탐지 → 추적기, 영상 송신부
inline constexpr std::string_view kPerceptionObservations = "perception/observations";
inline constexpr int kPerceptionObservationsQueueDepth = kDefaultQueueDepth;
// 추적기 → 추종 제어, onboard, 영상 송신부
inline constexpr std::string_view kTrackingTracks = "tracking/tracks";
inline constexpr int kTrackingTracksQueueDepth = kDefaultQueueDepth;
// 추적기 → onboard
inline constexpr std::string_view kTrackingStatus = "tracking/status";
inline constexpr int kTrackingStatusQueueDepth = kDefaultQueueDepth;
// onboard → 추적기, 추종 제어
inline constexpr std::string_view kFcAttitude = "fc/attitude";
inline constexpr int kFcAttitudeQueueDepth = kDefaultQueueDepth;
// onboard → 추적기, 추종 제어
inline constexpr std::string_view kFcState = "fc/state";
inline constexpr int kFcStateQueueDepth = kDefaultQueueDepth;
// onboard → 추종 제어
inline constexpr std::string_view kFcModeEvent = "fc/mode_event";
inline constexpr int kFcModeEventQueueDepth = kDefaultQueueDepth;
// 추종 제어 → onboard
inline constexpr std::string_view kControlSetpoint = "control/setpoint";
inline constexpr int kControlSetpointQueueDepth = kDefaultQueueDepth;
// 추종 제어 → onboard
inline constexpr std::string_view kFollowStatus = "follow/status";
inline constexpr int kFollowStatusQueueDepth = kDefaultQueueDepth;

// 요청-응답 (요청 → 응답)
// 추종 제어 → onboard
inline constexpr std::string_view kControlModeRequest = "control/mode_request";
// onboard → 추종 제어
inline constexpr std::string_view kFollowCommand = "follow/command";

}  // namespace argus::protocol

#endif  // ARGUS_PROTOCOL_TOPICS_HPP_
