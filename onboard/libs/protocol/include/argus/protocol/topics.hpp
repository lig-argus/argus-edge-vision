// RPi 안 ZeroMQ 토픽 이름과 RPC 이름. 코드에서 이름을 문자열로 직접 쓰지 않고 이 상수를 쓴다.
// 규약: docs/spec/process_interfaces.md 5.1 (토픽), 5.2 (RPC). 규약이 바뀌면 이 파일도 함께 바꾼다.
// 탐지(Python)는 이 헤더를 쓰지 못하므로 같은 이름을 따로 정의한다.
#ifndef ARGUS_PROTOCOL_TOPICS_HPP_
#define ARGUS_PROTOCOL_TOPICS_HPP_

#include <string_view>

namespace argus::protocol {

// 토픽 (발행 → 구독)
// 탐지 → 영상 송신부
inline constexpr std::string_view kVisionFrameGray8 = "vision/frame/gray8";
// 탐지 → 지상 미리보기
inline constexpr std::string_view kVisionOverlayJpeg = "vision/overlay/jpeg";
// 탐지 → (진단용, 구독자 없음)
inline constexpr std::string_view kVisionNpuInputJpeg = "vision/npu_input/jpeg";
// 탐지 → 추적기, 영상 송신부
inline constexpr std::string_view kPerceptionObservations = "perception/observations";
// 추적기 → 추종 제어, onboard, 영상 송신부
inline constexpr std::string_view kTrackingTracks = "tracking/tracks";
// 추적기 → onboard
inline constexpr std::string_view kTrackingStatus = "tracking/status";
// onboard → 추적기, 추종 제어
inline constexpr std::string_view kFcAttitude = "fc/attitude";
// onboard → 추적기, 추종 제어
inline constexpr std::string_view kFcState = "fc/state";
// onboard → 추종 제어
inline constexpr std::string_view kFcModeEvent = "fc/mode_event";
// 추종 제어 → onboard
inline constexpr std::string_view kControlSetpoint = "control/setpoint";
// 추종 제어 → onboard
inline constexpr std::string_view kFollowStatus = "follow/status";

// RPC (요청 → 응답)
// 추종 제어 → onboard
inline constexpr std::string_view kControlModeRequest = "control/mode_request";
// onboard → 추종 제어
inline constexpr std::string_view kFollowCommand = "follow/command";

}  // namespace argus::protocol

#endif  // ARGUS_PROTOCOL_TOPICS_HPP_
