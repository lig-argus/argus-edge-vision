// 실제 controller 설정/공개 표적 상태의 읽기 전용 native 조회 필드다.
#pragma once

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/control/guidance/include/yaw_scan_controller.hpp"
#include "argus/control/target/include/target_memory.hpp"
#include "argus/control/target/include/target_selector.hpp"

namespace follow {
inline bool prediction_report_valid(const TargetSelectionSnapshot& selected,
                                    Clock::time_point now) {
    const auto checked = now < selected.snapshot_at ? selected.snapshot_at : now;
    return selected.target && selected.state == TargetState::Predicted &&
           selected.target_phase != TargetPhase::Lost && selected.prediction_valid_until &&
           checked < *selected.prediction_valid_until;
}
inline NativeJson status_deadline(const std::optional<Clock::time_point>& deadline) {
    if (!deadline) {
        return nullptr;
    }
    return std::chrono::duration_cast<std::chrono::nanoseconds>(deadline->time_since_epoch())
        .count();
}
inline NativeJson native_selected_identity(const TargetSelectionSnapshot& selection) {
    if (!selection.target) {
        return nullptr;
    }
    NativeJson identity{{"generation", selection.target->generation},
                        {"track_id", selection.target->track_id}};
    if (selection.identity_class_id) {
        identity["class_id"] = *selection.identity_class_id;
        identity["label"] = selection.identity_label;
        identity["sensor"] = selection.identity_source;
    }
    return identity;
}
inline NativeJson native_tracking_status(const YawScanController& scan, TargetPhase phase,
                                         Clock::time_point now = Clock::now()) {
    const auto config = scan.config();
    const auto remaining = scan.remaining_seconds(now);
    return {{"target_phase", target_phase_name(phase)},
            {"scan_state", static_cast<int>(scan.state())},
            {"scan_state_name", yaw_scan_state_name(scan.state())},
            {"scan_remaining_s", remaining ? NativeJson(*remaining) : NativeJson(nullptr)},
            {"scan_yaw_done_rad",
             scan.active() ? NativeJson(scan.progress_deg() * 3.14159265358979323846 / 180.0)
                           : NativeJson(nullptr)},
            {"scan_config",
             {{"rate_deg_s", config.rate_deg_s},
              {"sweep_rad", config.sweep_deg * 3.14159265358979323846 / 180.0},
              {"timeout_ms", config.timeout.count()},
              {"yaw_rate_rad_s", config.rate_deg_s * 3.14159265358979323846 / 180.0},
              {"timeout_s", config.timeout.count() / 1000.0}}}};
}
}  // namespace follow
