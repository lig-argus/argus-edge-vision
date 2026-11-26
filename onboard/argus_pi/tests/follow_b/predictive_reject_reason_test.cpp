// 2026-11-10 예측 추종기 거부 사유 기록과 송신 slew 충돌 줄여 보내기 시험.
// 2026-11-10 08:58 비행: 명령(후진 -0.55)과 실제 속도(0.1~0.2)가 벌어지자 결과를 버려 추종이 HOLD로
// 끊겼다.
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "argus/control/guidance/include/predictive_follower.hpp"
#ifdef _WIN32
inline int setenv(const char* name, const char* value, int) { return _putenv_s(name, value); }
#endif

using namespace follow;

namespace {

PredictiveFollowConfig config(bool reconcile) {
    PredictiveFollowConfig c;
    c.sent_command_slew = true;
    c.sent_command_slew_reconcile = reconcile;
    c.pampc_point_profile = 1;
    c.range_m = 2;
    c.min_height_m = 1.1;
    c.max_height_m = 1.7;
    c.max_speed_m_s = 1.5;
    c.max_vertical_m_s = .3;
    c.max_acceleration_m_s2 = 1;
    c.braking_acceleration_m_s2 = 1.8;
    c.camera.image_width = 640;
    c.camera.image_height = 480;
    c.camera.fx = c.camera.fy = 1083.333333333;
    c.camera.cx = 320;
    c.camera.cy = 240;
    c.camera.camera_to_body = {0, -.7071067811865475, .7071067811865476, 1, 0, 0,
                               0, .7071067811865476,  .7071067811865475};
    c.camera.camera_offset_body_m = {.125, 0, 0};
    c.camera.box_anchor_y = 1;
    return c;
}

TrackRecordV1 box() {
    TrackRecordV1 b{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
    b.ground = std::array<double, 6>{3., 0, .01, 0, .01, 0};
    b.ground_anchor = std::array<double, 2>{1, 0};
    return b;
}

Pose pose() {
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -1.6;
    return p;
}

}  // namespace

int main() {
    setenv("ARGUS_COMPARISON_ISOLATED", "1", 1);
    setenv("ARGUS_SITL_PAMPC_POINT_ONLY", "1", 1);
    setenv("ARGUS_SITL_QUATERNION_STAGE", "0", 1);
    setenv("ARGUS_SITL_OPTIMIZER_MODE", "0", 1);
    setenv("ARGUS_SITL_SENT_COMMAND_SLEW", "1", 1);

    // 1. 입력 검사 사유
    {
        PredictiveFollower f(config(false));
        auto p = pose();
        auto b = box();
        b.ground.reset();
        assert(!f.update(p, p, b, {0, 0}, .01, -1.6, .05, 1, false, 1000000000LL));
        assert(f.last_reject_reason() == "target_ground_missing");
        assert(!f.update(p, p, box(), {0, 0}, .3, -1.6, .05, 1, false, 1000000000LL));
        assert(f.last_reject_reason() == "observation_age_out_of_range");
        p.have_velocity = false;
        assert(!f.update(p, p, box(), {0, 0}, .01, -1.6, .05, 1, false, 1000000000LL));
        assert(f.last_reject_reason() == "current_velocity_missing");
        auto ok = f.update(pose(), pose(), box(), {0, 0}, .01, -1.6, .05, 1, false, 1000000000LL);
        assert(ok && f.last_reject_reason().empty() && !f.last_slew_reconciled());
    }

    // 2. 송신 slew 충돌: 직전 송신값과 실제 FC 속도가 크게 벌어짐
    struct Conflict {
        bool ok;
        std::string reason;
        bool reconciled;
        double before, after;
    };
    const auto conflict = [](bool reconcile) {
        PredictiveFollower f(config(reconcile));
        auto p = pose();
        const auto first = f.update(p, p, box(), {0, 0}, .01, -1.6, .05, 1, false, 1000000000LL);
        assert(first);
        f.complete_update(true);     // 직전 송신값 확정
        p.velocity_north_m_s = 1.5;  // FC가 송신값보다 크게 앞섬
        const auto second = f.update(p, p, box(), {0, 0}, .01, -1.6, .05, 1, false, 1000000000LL);
        return Conflict{second.has_value(), f.last_reject_reason(), f.last_slew_reconciled(),
                        first->command.vx_m_s, second ? second->command.vx_m_s : std::nan("")};
    };
    {
        const auto r = conflict(false);
        assert(!r.ok && r.reason == "sent_command_slew_physical_conflict");
        std::cout << "legacy: rejected (" << r.reason << ")" << std::endl;
    }
    {
        const auto r = conflict(true);
        assert(r.ok && r.reconciled && r.reason.empty());
        // 직전 송신값에서 변화율 한도(1 m/s^2 x 최대 0.1 s) 안으로만, 실제 속도 쪽으로 옮긴다.
        std::cout << "reconcile: before " << r.before << " after " << r.after << std::endl;
        assert(r.after >= r.before - 1e-9);
        assert(r.after <= r.before + .1 + 1e-9);
    }
    std::cout << "PASS predictive reject reasons and sent-command slew reconcile\n";
    return 0;
}
