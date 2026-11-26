// 선택 명령·FC 상태·기하·추종·단일 PX4 송신자를 한 독립 제어 tick으로 잇는다.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "argus/control/contracts/include/i_flight_mode_port.hpp"
#include "argus/control/geometry/include/calibrated_ground_geometry.hpp"
#include "argus/control/guidance/include/flight_guidance_controller.hpp"
#include "argus/control/safety/include/flight_command_guard.hpp"
#include "argus/control/safety/include/flight_control_watchdog.hpp"
#include "argus/control/target/include/target_selector.hpp"
#include "argus/vehicle/contracts/include/i_vehicle_port.hpp"
#include "argus/vehicle/contracts/include/i_vehicle_state_source.hpp"
#include "argus/vehicle/contracts/include/vehicle_flight_status.hpp"

namespace follow {

struct FlightControlRuntimeConfig {
    double ground_down_m{0};
    double hold_down_m{0};
    std::uint32_t prestream_setpoint_count{15};
    std::chrono::milliseconds prestream_interval{100};
    std::chrono::milliseconds offboard_start_timeout{3000};
    std::chrono::milliseconds hold_confirm_timeout{2000};
    bool allow_recovery_during_priming{false};
    bool require_explicit_stop_recovery{false};  // B policy; legacy SDK behavior preserved.
};

// Exact control input retained for bounded, read-only post-flight diagnostics.
struct PredictiveDecisionInput {
    Pose vehicle_pose;
    std::optional<Pose> observation_pose;
    TrackRecordV1 track;
    std::uint64_t generation{0}, frame_id{0}, command_epoch{0}, mission_target_id{0};
    std::int64_t frame_sample_ns{0};
    std::optional<std::int64_t> exposure_ns;
    std::uint32_t width{0}, height{0};
    std::string source, timestamp_source, observation_pose_origin;
    std::string range_quality, range_reason;
    bool range_valid{false}, bearing_valid{false};
    double range_m{0}, bearing_error_deg{0};
    bool ground_motion_valid{false}, short_motion_valid{false};
    double target_vn_m_s{0}, target_ve_m_s{0};
    double short_vn_m_s{0}, short_ve_m_s{0}, short_rmse_m{0};
    double hold_down_m{0};
};

struct FlightControlRuntimeStatus {
    std::string mission{"IDLE"};
    std::string reason{"awaiting_explicit_start"};
    // Diagnostic copy of the original trigger before the stop transition may replace reason.
    std::string stop_trigger_reason;
    FlightSafetyEvent safety_event{FlightSafetyEvent::None};
    bool command_sent{false};
    bool offboard_start_requested{false};
    bool offboard_stop_requested{false};
    std::uint64_t selection_command_epoch{0};
    FollowCommand command;
    std::optional<double> guidance_ms;
    double manager_advance_ms{0};
    std::int64_t command_sample_ns{0};
    std::int64_t command_generated_ns{
        0};  // snapshot 정렬 뒤 실제 guidance 입력시각. tick 시작과 구별한다.
    std::optional<RearGoal> rear_goal;
    std::optional<std::array<PidTerms, 4>> pid_terms;
    std::optional<double> altitude_target_down_m;
    std::optional<PredictiveVerticalDiagnostics> predictive_vertical;
    std::optional<V21MotionDiagnostics> v21_motion;
    std::optional<PredictiveDecisionInput> predictive_input;
    bool predictive_diagnostic_capture_failed{false};
    std::optional<std::array<double, 2>> vehicle_reference_ned;
    std::optional<Pose> guidance_pose;
    Clock::time_point command_valid_until{};
};

// step()을 호출하는 한 스레드만 IVehiclePort와 IFlightModePort에 명령할 수 있다.
class FlightControlRuntime final {
public:
    FlightControlRuntime(TargetSelector&, const IVehicleStateSource&,
                         const IVehicleFlightStatusSource&, const CalibratedGroundGeometry&,
                         FlightGuidanceController&, IVehiclePort&, IFlightModePort&,
                         FlightControlRuntimeConfig,
                         FlightSafetyEventLatch* safety_events = nullptr,
                         std::recursive_mutex* context_mutex = nullptr);

    FlightControlRuntimeStatus step(Clock::time_point now = Clock::now());
    const std::atomic<bool>& following_active_flag() const noexcept { return following_active_; }

public:
    bool inactive() const { return phase_ == Phase::Idle; }
    bool recovery_latched() const { return phase_ == Phase::Manual || phase_ == Phase::Fault; }
    // Only after the command gateway admitted a fresh explicit STOP. No mode/velocity send.
    bool recover_after_explicit_stop(Clock::time_point now);
    void refresh_mode_guard();
    bool reset_ground_context(Clock::time_point now);
    bool prepare_handoff(Clock::time_point now);
    bool handoff_ready(Clock::time_point now) const;

private:
    enum class Phase { Idle, Priming, AwaitingOffboard, Following, Holding, Manual, Fault };
    void set_phase(Phase phase);
    void set_phase(Phase, FlightControlRuntimeStatus&, const char* mission,
                   const std::string& reason);
    void begin_stop(const FlightGuidanceResult&, const TargetSelectionSnapshot&,
                    Clock::time_point now, FlightControlRuntimeStatus&);
    FlightGuidanceInput make_input(const TargetSelectionSnapshot&, VehicleFlightStatus,
                                   Clock::time_point now) const;
    void begin_start(const TargetSelectionSnapshot&, const VehicleFlightStatus&,
                     Clock::time_point now, FlightControlRuntimeStatus&);

    void handle_command_epoch(const TargetSelectionSnapshot&, const VehicleFlightStatus&,
                              Clock::time_point now, FlightControlRuntimeStatus&);
    void prepare_offboard(const TargetSelectionSnapshot&, const VehicleFlightStatus&,
                          Clock::time_point now, FlightControlRuntimeStatus&);
    void follow_target(const TargetSelectionSnapshot&, const VehicleFlightStatus&,
                       Clock::time_point now, FlightControlRuntimeStatus&);
    bool confirm_offboard(const TargetSelectionSnapshot&, const VehicleFlightStatus&,
                          Clock::time_point now, FlightControlRuntimeStatus&);
    void confirm_hold(const VehicleFlightStatus&, Clock::time_point now,
                      FlightControlRuntimeStatus&);
    // 정지 요청이 송신권 없음으로 거부됐을 때: FC 연결부가 이미 실제 Hold 또는
    // Offboard 위치 유지를 이어받았는지 송신 없이 짧게 확인한다.
    static bool existing_hold_owner(const VehicleFlightStatus&);
    void accept_existing_hold(const VehicleFlightStatus&, Clock::time_point now,
                              FlightControlRuntimeStatus&);
    void confirm_existing_hold(const VehicleFlightStatus&, Clock::time_point now,
                               FlightControlRuntimeStatus&);
    TargetSelector& selector_;
    const IVehicleStateSource& pose_source_;
    const IVehicleFlightStatusSource& flight_state_source_;
    const CalibratedGroundGeometry& geometry_;
    FlightGuidanceController& controller_;
    FlightCommandGuard command_guard_;
    IVehiclePort& vehicle_port_;
    IFlightModePort& mode_port_;
    const FlightControlRuntimeConfig config_;
    FlightSafetyEventLatch* const safety_events_;
    Phase phase_{Phase::Idle};
    bool recovered_hold_{false};
    std::atomic<bool> following_active_{false};
    std::optional<std::uint64_t> observed_command_epoch_;
    std::uint32_t prestream_sent_{0};
    Clock::time_point next_prestream_at_{};
    Clock::time_point transition_deadline_{};
    bool stop_result_pending_{false};
    bool stop_result_ok_{false};
    bool existing_hold_wait_{false};
    Clock::time_point position_handoff_until_{};
    VehicleFlightStatus entry_state(VehicleFlightStatus, Clock::time_point now) const;
};
}  // namespace follow
