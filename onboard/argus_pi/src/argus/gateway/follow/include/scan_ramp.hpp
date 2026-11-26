// 2026-11-13 사용자 지시: 탐색(45002) 회전 가감속 램프, 돌다 멈추기, 밀림 멈춤.
// 램프: 회전 속도를 kScanRampS 동안 0에서 목표까지 올리고, 남은 회전각이 줄면 같은 가속도로 줄여
// 목표 각에서 멈춘다(속도 ≤ √(2·가속도·남은각)). 완료를 막지 않도록 kScanMinRateDegS 아래로는
// 줄이지 않는다.
// 돌다 멈추기: kScanStepDeg씩 돌고(각 구간마다 램프) kScanPauseS 멈춘 뒤 다음 구간을 돈다. 마지막
// 구간 뒤에는 멈추지 않고 끝낸다.
// 밀림 멈춤: 위치 유지 목표에서 kScanDriftStopM 넘게 밀리면 회전을 멈추고, kScanDriftResumeM 안으로
// 돌아오고 수평 속도가 kScanDriftResumeSpeedMS 이하가 되면(최대 kScanDriftMaxWaitS) 램프부터 다시
// 돈다. 위치 정보가 없으면 이 판단을 하지 않는다.
// 표적 발견·운용자 정지·제한 시간은 호출자가 지금처럼 바로 멈춘다. 효과는 실기 확인 전이다.
//
// 2026-11-18 사용자 지시: 좌우 왕복 탐색(sector)을 추가하고 기존 한 방향 회전(turn)과 바꿔 쓸 수
// 있게 했다. 실내 플로우 시험에서 yaw 중 플로우가 회전각만큼 가짜 이동을 쌓아(손 회전 0.56 m에서 CW
// 한 바퀴 약 45 cm, 1.7 m 환산 약 1 m) FC는 제자리라 믿는데 기체가 밀렸다. 왕복은 총 회전각을
// 줄이고 CW·CCW를 번갈아 돌려 일부 상쇄를 기대한다. 효과는 실기 확인 전이다. 구간 순서(시작 회전
// 방향 기준 부호각): +반폭 → −반폭 → … → 0(시작 방향). 각 구간 안에서는 위의 램프·돌다 멈추기·밀림
// 멈춤을 그대로 쓰고, 방향을 바꾸기 전에 kScanReversePauseS 멈춘다.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

namespace argus::gateway {

inline constexpr double kScanRampS = 1.0;
inline constexpr double kScanMinRateDegS = 2.0;
// 2026-11-13 구간을 30°에서 20°로 줄였다 (최고 속도는 manual_yaw.cpp 15°/s).
inline constexpr double kScanStepDeg = 20.0;
inline constexpr double kScanPauseS = 0.5;
// 구간 끝으로 보는 남은 각 (이 안이면 멈춘다).
inline constexpr double kScanStepToleranceDeg = 0.5;
// 밀림 멈춤 (2026-11-13 사용자 확정 조건).
inline constexpr double kScanDriftStopM = 0.15;
inline constexpr double kScanDriftResumeM = 0.10;
inline constexpr double kScanDriftResumeSpeedMS = 0.05;
inline constexpr double kScanDriftMaxWaitS = 2.0;

// 가감속 램프를 적용한 탐색 회전 속도(°/s, 부호 유지). elapsed_s: 회전 시작 후 시간,
// remaining_deg: 목표 회전각까지 남은 각.
inline double ramped_scan_rate(double rate_deg_s, double elapsed_s, double remaining_deg) {
    const double full = std::fabs(rate_deg_s);
    const double acceleration = full / kScanRampS;
    const double up = acceleration * std::max(0.0, elapsed_s);
    const double down = std::sqrt(2 * acceleration * std::max(0.0, remaining_deg));
    const double magnitude = std::max(std::min({full, up, down}), std::min(full, kScanMinRateDegS));
    return std::copysign(magnitude, rate_deg_s);
}

// 돌다 멈추기·밀림 멈춤 상태. step_end_deg: 지금 구간이 끝나는 누적 회전각, step_start_s: 지금
// 회전(램프)을 시작한 시각, pause_until_s: 이 시각 전까지는 멈춘다, drift_since_s: 밀림 멈춤을
// 시작한 시각(음수면 밀림 멈춤 아님).
struct StepScanState {
    double step_end_deg{kScanStepDeg};
    double step_start_s{0};
    double pause_until_s{-1};
    double drift_since_s{-1};
};

// 위치 유지 목표 대비 수평 오차(m)와 수평 속도(m/s). 신선한 위치가 없으면 nullopt.
struct ScanDrift {
    double error_m;
    double speed_m_s;
};

// 이번 명령의 회전 속도(°/s, 부호 유지, 멈춤이면 0). progress_deg: 실제 누적 회전각(절댓값).
inline double step_scan_rate(double rate_deg_s, double sweep_deg, double now_s, double progress_deg,
                             StepScanState& state, std::optional<ScanDrift> drift = std::nullopt) {
    if (state.drift_since_s >= 0) {
        const bool settled = drift && drift->error_m <= kScanDriftResumeM &&
                             drift->speed_m_s <= kScanDriftResumeSpeedMS;
        if (!settled && now_s - state.drift_since_s < kScanDriftMaxWaitS) {
            return 0;
        }
        state.drift_since_s = -1;
        state.step_start_s = now_s;  // 램프부터 다시
    }
    if (now_s < state.pause_until_s) {
        return 0;
    }
    if (drift && drift->error_m > kScanDriftStopM && progress_deg < sweep_deg) {
        state.drift_since_s = now_s;
        return 0;
    }
    const bool last_step = state.step_end_deg >= sweep_deg;
    if (!last_step && state.step_end_deg - progress_deg <= kScanStepToleranceDeg) {
        // 구간 끝: 멈추고, 넘친 각은 다음 구간에서 뺀다 (구간 경계는 고정 간격).
        while (state.step_end_deg - progress_deg <= kScanStepToleranceDeg &&
               state.step_end_deg < sweep_deg) {
            state.step_end_deg += kScanStepDeg;
        }
        state.pause_until_s = now_s + kScanPauseS;
        state.step_start_s = state.pause_until_s;
        return 0;
    }
    const double end = std::min(state.step_end_deg, sweep_deg);
    return ramped_scan_rate(rate_deg_s, now_s - state.step_start_s, end - progress_deg);
}

// ---- 좌우 왕복 탐색 (2026-11-18) ----
enum class ScanPattern : std::uint8_t { Turn, Sector };

// 왕복 기본 전체 폭(±60°)과 왕복 횟수, 방향 전환 정지 시간. 모두 제안값이다.
inline constexpr double kScanSectorDeg = 120.0;
inline constexpr double kScanSectorMaxDeg = 180.0;
inline constexpr int kScanSectorCycles = 1;
inline constexpr double kScanReversePauseS = 1.0;
// 2026-11-18 첫 실기(sess289)에서 구간 끝을 6~10° 넘쳐 돌았다(yaw 반응 지연). 구간 끝을 이만큼
// 앞당겨 감속·정지한다. 제안값이며 실기 넘침을 보고 조정한다.
inline constexpr double kScanSectorEndLeadDeg = 5.0;
// 이 값 이상 요청은 "한 바퀴(2π)" 기본값으로 보고 왕복 기본 폭을 쓴다 (float32 6.2832 rad 포함).
inline constexpr double kScanFullTurnRequestDeg = 359.5;

// "turn"/"sector" 문자열을 읽는다. 그 밖의 값은 nullopt.
inline std::optional<ScanPattern> parse_scan_pattern(std::string_view text) {
    if (text == "turn") {
        return ScanPattern::Turn;
    }
    if (text == "sector") {
        return ScanPattern::Sector;
    }
    return std::nullopt;
}

inline const char* scan_pattern_name(ScanPattern pattern) {
    return pattern == ScanPattern::Sector ? "sector" : "turn";
}

// 구간 수: +반폭/−반폭 왕복 cycles번과 마지막 시작 방향 복귀.
inline int sector_leg_count(int cycles) { return (2 * cycles) + 1; }

// leg번째 구간의 목표 부호각(시작 회전 방향이 +).
inline double sector_leg_target(int leg, int cycles, double half_deg) {
    if (leg >= 2 * cycles) {
        return 0.0;
    }
    return leg % 2 == 0 ? half_deg : -half_deg;
}

// 왕복 상태. leg_start_deg: 이 구간을 시작한 부호각(명목 경계).
struct SectorScanState {
    int leg{0};
    double leg_start_deg{0};
    StepScanState step{};
};

// 이번 명령의 회전 속도(°/s, 실제 yaw 부호, 멈춤이면 0). along_deg: 시작 회전 방향을 +로 한 실제
// 누적 부호각. 모든 구간이 끝나면 0을 돌려주고 state.leg == sector_leg_count(cycles)가 된다.
// continuous(2026-11-18 사용자 지시, 기본): 구간 안에서 20°마다 멈추지 않고 양 끝에서만
// 감속·정지한다. false면 구간 안에서도 기존 돌다 멈추기를 쓴다. 두 경우 모두 구간 끝을
// kScanSectorEndLeadDeg 앞당긴다.
inline double sector_scan_rate(double rate_deg_s, double width_deg, int cycles, double now_s,
                               double along_deg, SectorScanState& state,
                               std::optional<ScanDrift> drift = std::nullopt,
                               bool continuous = true) {
    const int legs = sector_leg_count(cycles);
    const double half = width_deg / 2.0;
    while (state.leg < legs) {
        const double target = sector_leg_target(state.leg, cycles, half);
        const double leg_sweep = std::fabs(target - state.leg_start_deg);
        const double direction = target >= state.leg_start_deg ? 1.0 : -1.0;
        const double leg_progress = std::max(0.0, direction * (along_deg - state.leg_start_deg));
        const double leg_end = std::max(0.0, leg_sweep - kScanSectorEndLeadDeg);
        if (leg_progress < leg_end - kScanStepToleranceDeg) {
            if (continuous) {
                // 구간 전체를 한 번의 "마지막 단계"로 두어 중간 정지를 없앤다.
                state.step.step_end_deg = std::max(state.step.step_end_deg, leg_end);
            }
            // 시작 방향 부호(rate_deg_s) × 구간 방향
            const double leg_rate = std::copysign(std::fabs(rate_deg_s), rate_deg_s * direction);
            return step_scan_rate(leg_rate, leg_end, now_s, leg_progress, state.step, drift);
        }
        // 구간 끝: 경계는 명목값으로 두어 넘친 각이 쌓이지 않게 하고, 방향을 바꾸기 전에 멈춘다.
        state.leg += 1;
        state.leg_start_deg = target;
        state.step = StepScanState{};
        state.step.pause_until_s = now_s + kScanReversePauseS;
        state.step.step_start_s = state.step.pause_until_s;
        if (state.leg < legs) {
            return 0;
        }
    }
    return 0;
}

// 현재 구간의 회전 방향 부호를 붙인 속도(표적 판정용, 멈춤 중에도 다음 방향).
inline double sector_leg_rate(double rate_deg_s, int cycles, double width_deg,
                              const SectorScanState& state) {
    if (state.leg >= sector_leg_count(cycles)) {
        return rate_deg_s;
    }
    const double target = sector_leg_target(state.leg, cycles, width_deg / 2.0);
    const double direction = target >= state.leg_start_deg ? 1.0 : -1.0;
    return std::copysign(std::fabs(rate_deg_s), rate_deg_s * direction);
}

}  // namespace argus::gateway
