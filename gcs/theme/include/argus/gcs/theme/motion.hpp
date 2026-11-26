// 움직임 값. 같은 뜻의 시간과 곡선을 화면마다 따로 적으면 서로 어긋나므로 한 곳에서 정한다.
// 시간은 Astro UXDS 컴포넌트의 움직임 시간을 따른다
#ifndef ARGUS_GCS_THEME_MOTION_HPP_
#define ARGUS_GCS_THEME_MOTION_HPP_

#include <algorithm>
#include <cstdint>

namespace argus::gcs::motion {

constexpr std::int32_t kFrameMs = 33;          // 애니메이션 한 장 (약 30 fps)
constexpr std::int32_t kHoverMs = 100;         // 마우스를 올렸을 때의 강조
constexpr std::int32_t kPopupMs = 200;         // 도움말 상자가 나타나는 시간
constexpr std::int32_t kEaseMs = 300;          // 채움, 자리 바뀜
constexpr std::int32_t kSpinPeriodMs = 1367;   // 도는 원 한 바퀴
constexpr std::int32_t kMorphMs = 900;         // 기체 그림과 스캔 원판 사이 전환
constexpr std::int32_t kMapRevealMs = 600;     // 지도(격자, 항적)가 나타나고 사라지는 시간
constexpr std::int32_t kPingMs = 600;          // 스캔 원판에 표적이 처음 보일 때 원이 퍼지는 시간
constexpr std::int32_t kSwitchMs = 100;        // 스위치 손잡이와 바탕
constexpr std::int32_t kDialogFadeMs = 500;    // 확인 창이 나타나는 시간
constexpr std::int32_t kScreenFadeMs = 500;    // 시작 화면과 운용 화면 사이 전환
constexpr std::int32_t kTooltipDelayMs = 800;  // 도움말 상자가 뜨기까지
constexpr std::int64_t kResultShowMs = 3000;   // 끝난 결과(스캔 결과, 명령 수락)를 보이는 시간

// 처음과 끝이 느린 곡선 (3 차 ease-in-out). 0~0.3 s 움직임은 모두 이 곡선을 쓴다
inline double easeInOut(double t) {
    t = std::clamp(t, 0.0, 1.0);
    if (t < 0.5) {
        return 4.0 * t * t * t;
    }
    const double u = (-2.0 * t) + 2.0;
    return 1.0 - (u * u * u / 2.0);
}

// 도는 원의 지금 각 (0~1, 한 바퀴). 모든 원이 같은 박자로 돌도록 같은 시계의 시각을 넘긴다
inline double spinTurn(std::int64_t nowMs) {
    const std::int64_t phase = ((nowMs % kSpinPeriodMs) + kSpinPeriodMs) % kSpinPeriodMs;
    return static_cast<double>(phase) / kSpinPeriodMs;
}

}  // namespace argus::gcs::motion

#endif  // ARGUS_GCS_THEME_MOTION_HPP_
