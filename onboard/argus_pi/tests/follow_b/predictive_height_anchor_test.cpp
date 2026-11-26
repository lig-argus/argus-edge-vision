// 2026-11-09 PAMPC point 모드 고도 고정항·수직 무시 구간 시험.
// 21:50 비행에서 point 모드는 현재 높이를 기준으로 삼아 1.55~1.91m로 출렁였다.
// 고정항을 켜면 낮은 높이에서 설정 높이 쪽으로 오르고, 무시 구간은 작은 수직 속도를
// 가속 제한을 지키며 0으로 줄이는지 본다. 설정을 넣지 않으면 기존 동작과 같아야 한다.

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "argus/control/guidance/include/predictive_follower.hpp"

using namespace follow;

namespace {

PredictiveFollowConfig base_config() {
    PredictiveFollowConfig c;
    c.pampc_point_profile = 1;
    c.range_m = 2;
    c.min_height_m = 1.1;
    c.max_height_m = 2.0;
    c.max_speed_m_s = .75;
    c.max_vertical_m_s = .7;
    c.max_acceleration_m_s2 = .6;
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

PredictiveFollowConfig anchored_config() {
    auto c = base_config();
    c.height_anchor_m = 1.8;
    c.height_anchor_weight = 10;
    c.height_anchor_deadband_m = .05;
    c.vertical_deadband_m_s = .1;
    return c;
}

bool rejects(const PredictiveFollowConfig& c) {
    try {
        PredictiveFollower follower(c);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

struct Run {
    double last_vz{0};
    double min_vz{0};
    double height_reference{0};
    unsigned flags{0};
};

// 같은 관측을 반복해 넣고, FC가 명령 속도를 그대로 따른다고 보고 높이를 적분한다.
PredictiveFollowConfig outdoor_config(double target, double weight) {
    auto c = base_config();
    c.outdoor_profile = true;
    c.target_height_m = target;
    c.min_height_m = target - .5;
    c.max_height_m = target + .5;
    c.height_anchor_weight = weight;
    c.height_anchor_deadband_m = weight > 0 ? .1 : 0;
    c.vertical_deadband_m_s = weight > 0 ? .1 : 0;
    return c;
}

Run run(const PredictiveFollowConfig& c, double start_height, int ticks, bool integrate) {
    PredictiveFollower follower(c);
    Pose p;
    p.have_position = p.have_attitude = p.have_velocity = true;
    p.down_m = -start_height;
    TrackRecordV1 box{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
    box.ground = std::array<double, 6>{2., 0, .01, 0, .01, 0};
    box.ground_anchor = std::array<double, 2>{1, 0};
    Run result;
    for (int i = 0; i < ticks; ++i) {
        const auto out = follower.update(p, p, box, {0, 0}, .01, -1.9, .05, 1, false,
                                         box.last_real_detection_monotonic_ns);
        assert(out);
        follower.complete_update(true);
        result.last_vz = out->command.vz_m_s;
        result.min_vz = std::min(result.min_vz, out->command.vz_m_s);
        result.height_reference = out->vertical_diagnostics.height_reference_m;
        result.flags |= out->vertical_diagnostics.safety_flags;
        p.velocity_down_m_s = out->command.vz_m_s;
        if (integrate) {
            p.down_m += out->command.vz_m_s * .05;
        }
        box.last_real_detection_monotonic_ns += 50000000;
    }
    result.height_reference = integrate ? -p.down_m : result.height_reference;
    return result;
}

}  // namespace

int main() {
    // 1. 설정 검사: point 모드가 아니거나, 고정 높이가 범위 끝 0.1m 안이면 거부한다.
    {
        auto c = anchored_config();
        c.pampc_point_profile = 0;
        c.vertical_deadband_m_s = 0;
        assert(rejects(c));
        c = anchored_config();
        c.height_anchor_m = 1.95;
        assert(rejects(c));
        c = anchored_config();
        c.height_anchor_weight = 0;
        assert(rejects(c));
        c = anchored_config();
        c.vertical_deadband_m_s = .35;  // max_vertical의 절반 이상
        assert(rejects(c));
        c = base_config();
        c.pampc_point_profile = 0;
        c.vertical_deadband_m_s = .1;
        assert(rejects(c));
        assert(!rejects(anchored_config()));
        assert(!rejects(base_config()));
    }

    // 2. 기존 동작: 높이 기준은 현재 높이(1.5m)이고 1.8m로 끌어올리지 않는다.
    const Run legacy = run(base_config(), 1.5, 40, false);
    assert(std::abs(legacy.height_reference - 1.5) < 1e-9);

    // 3. 고정항: 높이 기준이 1.8m이고, 1.5m에서는 상승(NED vz<0) 명령이 나온다.
    const Run anchored = run(anchored_config(), 1.5, 40, false);
    std::cout << "legacy vz " << legacy.last_vz << " anchored vz " << anchored.last_vz << '\n';
    assert(std::abs(anchored.height_reference - 1.8) < 1e-9);
    assert(anchored.last_vz < -.1);
    assert(anchored.last_vz < legacy.last_vz - .1);

    // 4. FC가 명령을 따른다고 보고 10초 적분하면 1.8m ± 무시 구간 근처로 수렴한다.
    const Run settled = run(anchored_config(), 1.5, 200, true);
    std::cout << "settled height " << settled.height_reference << " vz " << settled.last_vz << '\n';
    assert(std::abs(settled.height_reference - 1.8) < .12);
    assert(std::abs(settled.last_vz) < .1 + 1e-9);

    // 5. 무시 구간: 고정 높이에서 출발하면 작은 수직 속도는 0으로 보내고(16번 표시),
    //    가속 제한(0.6m/s² × 0.05s = 0.03m/s)을 넘겨 한 번에 바꾸지 않는다.
    {
        PredictiveFollower follower(anchored_config());
        Pose p;
        p.have_position = p.have_attitude = p.have_velocity = true;
        p.down_m = -1.8;
        p.velocity_down_m_s = .09;
        TrackRecordV1 box{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
        box.ground = std::array<double, 6>{2., 0, .01, 0, .01, 0};
        box.ground_anchor = std::array<double, 2>{1, 0};
        double previous = .09;
        bool reached_zero = false;
        for (int i = 0; i < 20; ++i) {
            const auto out = follower.update(p, p, box, {0, 0}, .01, -1.9, .05, 1, false,
                                             box.last_real_detection_monotonic_ns);
            assert(out);
            follower.complete_update(true);
            const double vz = out->command.vz_m_s;
            assert(std::abs(vz - previous) <= .030001);
            if ((out->vertical_diagnostics.safety_flags & 16) != 0) {
                assert(std::abs(vz) <= std::abs(previous) + 1e-9);  // 0 쪽으로만 줄인다
                assert(vz * previous >= 0);
            }
            reached_zero = reached_zero || std::abs(vz) < 1e-12;
            previous = vz;
            p.velocity_down_m_s = vz;
            box.last_real_detection_monotonic_ns += 50000000;
        }
        assert(reached_zero);
    }
    // 5b. 실제 Pi 설정(v32_sent_command_slew=true)에서도 무시 구간은 0 쪽으로만 줄인다.
    //     송신 기준 slew는 호스트 경과 시간을 쓰므로 여기서는 0까지 도달 여부는 보지 않는다.
    {
        auto c = anchored_config();
        c.sent_command_slew = true;
        PredictiveFollower follower(c);
        Pose p;
        p.have_position = p.have_attitude = p.have_velocity = true;
        p.down_m = -1.8;
        p.velocity_down_m_s = .09;
        TrackRecordV1 box{1, 2, "car", .9, 270, 190, 370, 290, "CONFIRMED", true, 1000000000LL};
        box.ground = std::array<double, 6>{2., 0, .01, 0, .01, 0};
        box.ground_anchor = std::array<double, 2>{1, 0};
        double previous = .09;
        for (int i = 0; i < 20; ++i) {
            const auto out = follower.update(p, p, box, {0, 0}, .01, -1.9, .05, 1, false,
                                             box.last_real_detection_monotonic_ns);
            assert(out);
            follower.complete_update(true);
            const double vz = out->command.vz_m_s;
            if ((out->vertical_diagnostics.safety_flags & 16) != 0) {
                assert(std::abs(vz) <= std::abs(previous) + 1e-9);
                assert(vz * previous >= 0);
            }
            previous = vz;
            p.velocity_down_m_s = vz;
            box.last_real_detection_monotonic_ns += 50000000;
        }
    }

    // 7. 2026-11-16 15:20 비행: 2.08m(기준 1.8 + 0.28, 최대 2.0 초과)에서 작은 하강이 0으로 잘려
    //    높이가 내려오지 않았다. 해제(기본 0.1m)는 작은 하강도 보낸다. 선택 상한은 기본 꺼짐.
    {
        auto c = anchored_config();
        c.vertical_release_max_m_s = .05;  // 데드밴드보다 작으면 거부
        assert(rejects(c));
        c = anchored_config();
        c.vertical_deadband_release_m = .6;
        assert(rejects(c));
        const Run released = run(anchored_config(), 2.08, 40, false);
        auto off = anchored_config();
        off.vertical_deadband_release_m = 0;
        const Run legacy_db = run(off, 2.08, 40, false);
        std::cout << "2.08m released vz " << released.last_vz << " flags " << released.flags
                  << " / release off vz " << legacy_db.last_vz << " flags " << legacy_db.flags
                  << '\n';
        assert(released.last_vz > .05);
        assert(released.last_vz >= legacy_db.last_vz - 1e-9);  // 기존보다 덜 내려가지 않는다
        assert((legacy_db.flags & 32) == 0);
        auto capped = anchored_config();
        capped.vertical_release_max_m_s = .2;
        assert(run(capped, 2.08, 40, false).last_vz <= .2 + 1e-9);
        // FC가 따른다고 보고 적분하면 기준 근처로 내려온다.
        const Run down = run(anchored_config(), 2.08, 200, true);
        std::cout << "2.08m settled height " << down.height_reference << '\n';
        assert(std::abs(down.height_reference - 1.8) < .2);
    }

    // 6. 실외 3m·4m: 기존 target_height_m을 고정 높이로 쓰고 가중치만 키운다.
    for (const double target : {3.0, 4.0}) {
        auto bad = outdoor_config(target, 10);
        bad.height_anchor_m = target;  // 실외는 target_height_m 하나만 쓴다
        assert(rejects(bad));
        assert(!rejects(outdoor_config(target, 10)));
        const Run weak = run(outdoor_config(target, 0), target - .3, 40, false);
        const Run strong = run(outdoor_config(target, 10), target - .3, 40, false);
        std::cout << "outdoor " << target << "m weak vz " << weak.last_vz << " strong vz "
                  << strong.last_vz << '\n';
        assert(std::abs(weak.height_reference - target) < 1e-9);
        assert(std::abs(strong.height_reference - target) < 1e-9);
        assert(strong.last_vz < -.1);
        assert(strong.last_vz < weak.last_vz - .05);
    }
    std::cout << "predictive height anchor and vertical deadband PASS\n";
    return 0;
}
