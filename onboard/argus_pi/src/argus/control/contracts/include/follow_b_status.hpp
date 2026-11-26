// Coarse B status projection; preserve detailed recovery diagnostics separately.
#pragma once

#include <string>

#include "argus/control/target/include/target_memory.hpp"

namespace follow {
struct FollowBStatusInput {
    std::string mission{"IDLE"};
    bool rc_override{false};
    bool fault{false};
    bool hold_transition{false};
    bool actual_hold{false};
    bool scan_active{false};
    bool preparing{false};
    bool following{false};
    bool target_selected{false};
    bool target_confirmed{false};
    bool target_prediction_valid{false};
    TargetPhase target_phase{TargetPhase::Unselected};
    std::string recovery_reason;
    bool ambiguous{false};
};

struct FollowBStatus {
    std::string mission;
    std::string target;
    std::string target_phase;
    std::string recovery_reason;
    bool ambiguous{false};
};

// 실제로 멈춰 있는가: FC HOLD 모드, 또는 게이트웨이가 확인한 Offboard 제자리 유지.
// 2026-11-13: GPS 없는 실내의 [임무 중단]은 HOLD가 아니라 Offboard 제자리 유지로 끝난다.
// 이것을 빼면 정지가 HOLD_TRANSITION으로 남아 메인 GUI가 [스캔 시작]·[추종 시작]을 막았다
// (2026-11-12 20:10 실내 실기, GUI 녹화 track_state 5 유지).
inline bool confirmed_stop_hold(bool fc_hold, bool offboard, bool offboard_hover) {
    return fc_hold || (offboard && offboard_hover);
}

inline FollowBStatus project_follow_b_status(const FollowBStatusInput& input) {
    FollowBStatus result;
    // Manual override wins; fault wins over normal execution. HOLD_WAIT means
    // confirmed HOLD, never just a pending mode request or a command ACK.
    if (input.rc_override || input.mission == "MANUAL") {
        result.mission = "MANUAL";
    } else if (input.fault || input.mission == "FAULT") {
        result.mission = "FAULT";
    } else if (input.hold_transition || input.mission == "HOLD_TRANSITION" ||
               ((input.mission == "HOLD_WAIT" || input.mission == "HOLD_CONFIRMED") &&
                !input.actual_hold)) {
        result.mission = "HOLD_TRANSITION";
    } else if (input.scan_active) {
        result.mission = "SCAN_ACTIVE";
    } else if (input.preparing || input.mission == "OFFBOARD_PREPARING" ||
               input.mission == "OFFBOARD_WAIT" || input.mission == "PREPARING") {
        result.mission = "PREPARING";
    } else if (input.following || input.mission == "FOLLOW_ACTIVE") {
        result.mission = "FOLLOW_ACTIVE";
    } else if (input.actual_hold &&
               (input.mission == "HOLD_WAIT" || input.mission == "HOLD_CONFIRMED")) {
        result.mission = "HOLD_WAIT";
    } else {
        result.mission = "IDLE";
    }
    if (!input.target_selected) {
        result.target = "NONE";
    } else if (input.target_phase == TargetPhase::Lost) {
        result.target = "LOST";
    } else if (input.target_phase == TargetPhase::Locked && input.target_confirmed) {
        result.target = "CONFIRMED";
    } else if (input.target_prediction_valid) {
        result.target = "PREDICTED";
    } else {
        result.target = "LOST";
    }
    result.target_phase = target_phase_name(input.target_phase);
    result.recovery_reason = input.recovery_reason;
    result.ambiguous = input.ambiguous;
    return result;
}
}  // namespace follow
