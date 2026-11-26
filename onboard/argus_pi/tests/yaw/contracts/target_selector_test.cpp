// 명시 선택 고정·중복 요청·겹침 거부·세대 변경 시 자동 재개 금지를 확인한다.
#include "argus/control/target/include/target_selector.hpp"

#include <cassert>
#include <cmath>

using namespace follow;
using namespace std::chrono_literals;

namespace {
TrackRecordV1 track(std::int64_t id, double x, const char* state = "CONFIRMED",
                    std::int64_t last_real = 1000000000) {
    const bool real = std::string(state) == "CONFIRMED";
    return {id, 5, "rc_car", 0.9, x, 20, x + 60, 90, state, real, last_real};
}
TracksV1 frame(std::uint64_t number, std::uint64_t generation, std::int64_t time,
               std::vector<TrackRecordV1> tracks) {
    TracksV1 message;
    message.source = "test-camera";
    message.frame_id = number;
    message.generation = generation;
    message.sample_monotonic_ns = time;
    message.clock_domain = "test-clock";
    message.timestamp_source = "exposure";
    message.exposure_monotonic_ns = time;
    message.exposure_time_valid = true;
    message.image_width = 640;
    message.image_height = 480;
    message.tracks = std::move(tracks);
    return message;
}
}  // namespace

int main() {
    TargetSelector selector({std::chrono::milliseconds(250), 0.7, 0.65, 32});
    const auto now = Clock::now();
    assert(selector.update(frame(1, 0, 1000000000, {track(7, 20), track(8, 22)}), now));
    auto result = selector.execute({"select-overlap", TargetCommandAction::Select, 0, 7}, now);
    assert(!result.accepted && result.reason == "overlapping_candidate_ambiguous");

    assert(selector.update(
        frame(2, 0, 1100000000,
              {track(7, 20, "CONFIRMED", 1100000000), track(8, 220, "CONFIRMED", 1100000000)}),
        now + 10ms));
    result = selector.execute({"select-7", TargetCommandAction::Select, 0, 7}, now + 10ms);
    assert(result.accepted && result.status == "SELECTED_PENDING");
    assert(selector.execute({"select-7", TargetCommandAction::Select, 0, 7}, now + 10ms).accepted);
    const auto reused = selector.execute({"select-7", TargetCommandAction::Stop, 0, 0}, now + 10ms);
    assert(!reused.accepted && reused.reason == "request_id_reused_with_different_command");

    result = selector.execute({"start", TargetCommandAction::Start}, now + 10ms);
    assert(result.accepted && result.status == "FOLLOWING");
    auto following = selector.snapshot(now + 10ms);
    assert(following.target->track_id == 7 && following.command_epoch == 1);

    // 후보 순서가 바뀌어도 대상은 선택한 (generation, ID)에 고정한다.
    assert(selector.update(
        frame(3, 0, 1200000000,
              {track(8, 220, "CONFIRMED", 1200000000), track(7, 20, "PREDICTED", 1100000000)}),
        now + 20ms));
    auto state = selector.snapshot(now + 20ms);
    assert(state.target && state.target->track_id == 7 && state.state == TargetState::Predicted);
    assert(!state.fresh_real_detection);

    // 예측 TTL 초과나 새 추적 세대는 자동 대상 교체/재시작으로 이어지지 않는다.
    assert(selector.update(
        frame(4, 0, 1900000000,
              {track(7, 20, "PREDICTED", 1100000000), track(8, 220, "CONFIRMED", 1900000000)}),
        now + 30ms));
    assert(selector.snapshot(now + 30ms).state == TargetState::Lost);
    assert(selector.update(frame(5, 1, 2000000000, {track(1, 220, "CONFIRMED", 2000000000)}),
                           now + 40ms));
    state = selector.snapshot(now + 40ms);
    assert(state.target && state.target->generation == 0 && state.target->track_id == 7);
    assert(state.state == TargetState::Lost);

    result = selector.execute({"stop", TargetCommandAction::Stop}, now + 40ms);
    assert(result.accepted &&
           selector.snapshot(now + 40ms).phase == TargetSelectionSnapshot::Phase::Stopped);
    assert(selector.snapshot(now + 40ms).command_epoch == 2);
    // 방향 반전에서 회귀 위치가 실제 위치를 덮어쓰지 않는다. 속도 추정/TTL은 그대로다.
    for (bool fitted : {false, true}) {
        TargetSelectorConfig cfg;
        cfg.recovery.enabled = true;
        cfg.recovery.motion_fit_acceleration = true;
        cfg.recovery.motion_history_s = 2;
        cfg.recovery.motion_history_min_s = 1.5;
        cfg.recovery.motion_history_min_samples = 20;
        cfg.recovery.motion_fit_max_rmse_m = .1;
        cfg.use_fitted_ground_position = fitted;
        TargetSelector moving(cfg);
        double position = 0;
        for (int i = 0; i <= 40; ++i) {
            const auto stamp = 1000000000LL + i * 50000000LL;
            const auto at = Clock::time_point{std::chrono::nanoseconds(stamp)};
            position = 6 + .6 * std::min(i * .05, 1.5) - .6 * std::max(0., i * .05 - 1.5);
            auto car = track(1, 200, "CONFIRMED", stamp);
            car.ground = std::array<double, 6>{position, 0, .05, 0, .05, 0};
            assert(moving.update(frame(i + 1, 1, stamp, {car}), at));
            if (i == 0) {
                assert(moving.execute({"pick", TargetCommandAction::Select, 1, 1}, at).accepted);
                assert(moving.execute({"start", TargetCommandAction::Start}, at).accepted);
            }
        }
        const auto current = moving.snapshot(Clock::time_point{3s});
        assert(current.ground_acceleration_valid && current.fresh_real_detection &&
               current.selected_track);
        const auto error = std::abs((*current.selected_track->ground)[0] - position);
        assert(fitted ? error > .05 : error < 1e-10);
        assert(current.latest_frame->sample_monotonic_ns == 3000000000LL);
        assert(current.observation_valid_until == Clock::time_point{3250ms});
    }
    return 0;
}
