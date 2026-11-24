// 시험용 영상의 내용: 프레임마다 이동하는 사각형 하나와, 그 사각형에 대한 탐지 결과와 추적 결과
// 사용: TestPatternFrameSource (영상), src/main.cpp (--test-pattern 의 탐지 결과와 추적 결과)
//
// 영상: 가로 방향으로 밝기가 증가하는 배경 위에 밝은 사각형 하나. 사각형은 프레임마다 오른쪽으로
// 이동하고 오른쪽 끝에 도달하면 왼쪽 끝에서 다시 시작한다. 추적 결과의 박스가 그 사각형의 위치와
// 같으므로, 수신 화면에서 박스가 사각형과 일치하는지로 영상과 메타데이터의 동기화를 확인할 수 있다
#ifndef ARGUS_VIDEO_SENDER_TEST_PATTERN_SCENE_HPP_
#define ARGUS_VIDEO_SENDER_TEST_PATTERN_SCENE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "argus/perception_api/types.hpp"

namespace argus::video_sender {

struct TestPatternConfig {
    std::string source = "camera-ir";
    std::string session_id = "test-pattern";

    // IR 카메라의 영상 크기 (video.md 2장). 0 이면 픽셀이 없는 프레임과 크기가 0 인 박스가 된다
    std::uint32_t width = 640;
    std::uint32_t height = 480;
};

// frame_id 번째 프레임의 사각형 위치 (영상 픽셀 좌표)
perception_api::PixelBox test_pattern_box(const TestPatternConfig& config, std::uint64_t frame_id);

// frame_id 번째 프레임의 픽셀 (8 비트 흑백, 한 줄이 width 바이트, width × height 바이트)
std::vector<std::uint8_t> test_pattern_pixels(const TestPatternConfig& config,
                                              std::uint64_t frame_id);

// frame 의 사각형을 객체 하나로 하는 탐지 결과와 추적 결과. source, session_id, frame_id, 영상
// 크기는 frame 과 같다
perception_api::Observations test_pattern_observations(const TestPatternConfig& config,
                                                       const perception_api::Frame& frame);
perception_api::Tracks test_pattern_tracks(const TestPatternConfig& config,
                                           const perception_api::Frame& frame);

}  // namespace argus::video_sender

#endif  // ARGUS_VIDEO_SENDER_TEST_PATTERN_SCENE_HPP_
