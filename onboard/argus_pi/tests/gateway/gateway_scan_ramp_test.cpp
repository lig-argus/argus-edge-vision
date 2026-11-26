// 2026-11-13 탐색 회전 가감속 램프 시험 (FC·소켓 없음). 10°/s 360° 탐색을 50 Hz로 적분한다.
#include <cassert>
#include <cmath>
#include <iostream>

#include "argus/gateway/follow/include/scan_ramp.hpp"

namespace gw = argus::gateway;

int main() {
    for (const double rate : {10.0, 15.0, -10.0}) {
        const double dt = .02;
        double t = 0, progress = 0, peak = 0, max_step = 0, previous = 0;
        bool reached_full = false;
        while (progress < 360 && t < 120) {
            const double command = gw::ramped_scan_rate(rate, t, 360 - progress);
            assert(std::copysign(1.0, command) == std::copysign(1.0, rate));
            peak = std::max(peak, std::fabs(command));
            max_step = std::max(max_step, std::fabs(command - previous));
            reached_full = reached_full || std::fabs(std::fabs(command) - std::fabs(rate)) < 1e-9;
            previous = command;
            progress += std::fabs(command) * dt;
            t += dt;
        }
        const double ideal = 360 / std::fabs(rate);
        std::cout << "rate " << rate << ": done " << progress << " deg in " << t << " s (ideal "
                  << ideal << " s), peak " << peak << ", max step per tick " << max_step << '\n';
        assert(progress >= 360);  // 끝까지 돈다 (최저 속도 덕분에 멈추지 않는다)
        assert(reached_full);     // 중간에는 목표 속도로 돈다
        assert(peak <= std::fabs(rate) + 1e-9);
        assert(t <= ideal + 2.5);  // 램프로 늘어나는 시간은 약 1초 (+ 최저 속도 꼬리)
        // 시작은 0에서가 아니라 최저 속도(2°/s)에서 출발하고, 이후 틱당 변화는 가속도×dt 이하.
        assert(max_step <= gw::kScanMinRateDegS + 1e-9);
    }
    // 시작 직후·끝 직전은 느리고, 중간은 목표 속도다.
    assert(std::fabs(gw::ramped_scan_rate(10, 0, 360) - 2) < 1e-9);
    assert(std::fabs(gw::ramped_scan_rate(10, .5, 360) - 5) < 1e-9);
    assert(std::fabs(gw::ramped_scan_rate(10, 5, 180) - 10) < 1e-9);
    assert(std::fabs(gw::ramped_scan_rate(10, 30, 1.25) - 5) < 1e-9);  // √(2·10·1.25)=5
    std::cout << "PASS scan ramp: 1 s ramp up/down, full rate between, completes sweep\n";

    // 돌다 멈추기: 최고 15°/s, 20°씩 돌고 0.5초 멈춘다. 기체 yaw는 명령을 0.2초 1차 지연으로
    // 따라간다.
    for (const double rate : {15.0, -15.0}) {
        gw::StepScanState step;
        const double dt = .02;
        double t = 0, progress = 0, actual = 0, peak = 0, stopped_s = 0;
        int pauses = 0;
        bool paused = false;
        while (progress < 360 && t < 60) {
            const double command = gw::step_scan_rate(rate, 360, t, progress, step);
            if (command == 0 && !paused) {
                ++pauses;
            }
            paused = command == 0;
            stopped_s += paused ? dt : 0;
            peak = std::max(peak, std::fabs(command));
            actual += (std::fabs(command) - actual) * dt / .2;
            progress += actual * dt;
            t += dt;
        }
        std::cout << "step scan " << rate << ": done " << progress << " deg in " << t
                  << " s, pauses " << pauses << ", stopped " << stopped_s << " s, peak " << peak
                  << '\n';
        assert(progress >= 360);
        assert(pauses == 17);  // 20°마다 멈춤, 마지막 구간 뒤에는 멈추지 않음
        assert(std::fabs(stopped_s - 17 * .5) < .3);
        assert(peak <= 15 + 1e-9);
        assert(t < 70);  // 기본 제한 시간 90초 안에 끝난다
    }
    std::cout << "PASS step scan: 20 deg steps, 0.5 s pauses, completes 360 deg\n";

    // 밀림 멈춤: 회전 중에는 회전 속도에 비례해 밀리고(15°/s에서 0.09 m/s), 멈추면 오차가 0.12
    // m/s로 줄며 속도는 0.03 m/s로 본다.
    {
        gw::StepScanState step;
        const double dt = .02;
        double t = 0, progress = 0, error = 0, max_error = 0;
        int holds = 0;
        bool holding = false;
        while (progress < 360 && t < 90) {
            const double speed = holding ? .03 : .08;
            const double command =
                gw::step_scan_rate(15, 360, t, progress, step, gw::ScanDrift{error, speed});
            const bool now_holding = step.drift_since_s >= 0;
            holds += now_holding && !holding;
            holding = now_holding;
            error = command != 0 ? error + .006 * std::fabs(command) * dt
                                 : std::max(0.0, error - .12 * dt);
            max_error = std::max(max_error, error);
            progress += std::fabs(command) * dt;
            t += dt;
        }
        std::cout << "drift hold: done " << progress << " deg in " << t << " s, holds " << holds
                  << ", max error " << max_error << " m\n";
        assert(progress >= 360 && holds > 0);
        assert(max_error < gw::kScanDriftStopM + .01);  // 0.15 m 넘으면 바로 멈춘다
    }
    // 밀림 멈춤이 끝나지 않아도 최대 2초 뒤에는 다시 돈다.
    {
        gw::StepScanState step;
        assert(gw::step_scan_rate(15, 360, 10, 5, step, gw::ScanDrift{.2, .2}) == 0);
        assert(gw::step_scan_rate(15, 360, 11, 5, step, gw::ScanDrift{.2, .2}) == 0);
        assert(gw::step_scan_rate(15, 360, 12.01, 5, step, gw::ScanDrift{.12, .01}) != 0);
        // 위치 정보가 없으면 밀림 판단을 하지 않는다.
        gw::StepScanState free;
        assert(gw::step_scan_rate(15, 360, 1, 5, free, std::nullopt) != 0);
    }
    std::cout << "PASS drift hold: stop above 0.15 m, resume within 0.10 m or after 2 s\n";

    // 2026-11-19 한 방향 회전(manual_yaw.cpp): 오른쪽 180°.
    // 구간 끝을 전체 범위로 두어 중간 정지 없이 돌고, 180° 근처에서 감속해 멈춘다.
    // 기체 yaw는 명령을 0.2초 1차 지연으로 따라간다.
    {
        gw::StepScanState step{180.0, 0.0, -1.0, -1.0};
        const double dt = .02;
        double t = 0, actual = 0, progress = 0, peak = 0;
        int stops_before_end = 0;
        while (progress < 180 && t < 60) {
            const double command = gw::step_scan_rate(15, 180, t, progress, step, std::nullopt);
            if (command == 0 && progress < 175) {
                ++stops_before_end;
            }
            assert(command >= 0);
            actual += (command - actual) * dt / .2;
            progress += actual * dt;
            peak = std::max(peak, actual);
            t += dt;
        }
        assert(stops_before_end == 0);
        assert(progress >= 180 && progress < 182);
        assert(peak > 14 && peak <= 15);
        assert(t > 12 && t < 16);
        std::cout << "PASS turn 180 right: no intermediate stop, " << t << " s, end " << progress
                  << " deg\n";
    }

    // 2026-11-18 좌우 왕복 탐색: 120° 폭(±60°), 1회 왕복 후 시작 방향 복귀. 기체 yaw는 명령을 0.2초
    // 1차 지연으로 따라간다. 시작 회전 방향이 +인 부호각(along)으로 판정한다.
    assert(gw::parse_scan_pattern("turn") == gw::ScanPattern::Turn);
    assert(gw::parse_scan_pattern("sector") == gw::ScanPattern::Sector);
    assert(!gw::parse_scan_pattern("") && !gw::parse_scan_pattern("Sector"));
    assert(gw::sector_leg_count(1) == 3);
    assert(gw::sector_leg_target(0, 1, 60) == 60 && gw::sector_leg_target(1, 1, 60) == -60 &&
           gw::sector_leg_target(2, 1, 60) == 0);
    // 2026-11-18: 연속 왕복(기본)은 양 끝에서만 멈추고, 구간 끝을 kScanSectorEndLeadDeg 앞당긴다.
    for (const bool continuous : {true, false}) {
        for (const double rate : {15.0, -15.0}) {
            gw::SectorScanState sector;
            const double dt = .02;
            double t = 0, yaw = 0, actual = 0, path = 0, max_along = 0, min_along = 0, peak = 0;
            int reversals = 0, pauses = 0;
            double last_sign = 0;
            bool paused = false;
            const int legs = gw::sector_leg_count(gw::kScanSectorCycles);
            while (sector.leg < legs && t < 90) {
                const double along = std::copysign(1.0, rate) * yaw;
                const double command =
                    gw::sector_scan_rate(rate, gw::kScanSectorDeg, gw::kScanSectorCycles, t, along,
                                         sector, std::nullopt, continuous);
                if (command == 0 && !paused && t > 0) {
                    ++pauses;
                }
                paused = command == 0;
                if (command != 0) {
                    const double sign = std::copysign(1.0, command);
                    reversals += last_sign != 0 && sign != last_sign;
                    last_sign = sign;
                }
                peak = std::max(peak, std::fabs(command));
                actual += (command - actual) * dt / .2;
                yaw += actual * dt;
                path += std::fabs(actual) * dt;
                max_along = std::max(max_along, std::copysign(1.0, rate) * yaw);
                min_along = std::min(min_along, std::copysign(1.0, rate) * yaw);
                t += dt;
            }
            std::cout << "sector scan " << (continuous ? "continuous " : "steps ") << rate
                      << ": legs " << sector.leg << " in " << t << " s, along range " << min_along
                      << ".." << max_along << " deg, final yaw " << yaw << ", path " << path
                      << " deg, reversals " << reversals << ", pauses " << pauses << ", peak "
                      << peak << '\n';
            const double lead = gw::kScanSectorEndLeadDeg;
            assert(sector.leg == legs);  // 세 구간 모두 마침
            assert(reversals == 2);      // +60 → −60 → 0: 방향 전환 2번
            // 구간 끝을 lead만큼 앞당겨 멈춘다 (모의 지연 넘침 2° 이내)
            assert(max_along > 60 - lead - 1 && max_along < 60 - lead + 2);
            assert(min_along < -(60 - lead - 1) && min_along > -(60 - lead + 2));
            assert(std::fabs(yaw) < lead + 2);  // 시작 방향 근처로 돌아온다
            assert(path > 4 * (60 - lead) - 4 && path < 4 * (60 - lead) + 8);
            assert(peak <= 15 + 1e-9);
            if (continuous) {
                assert(pauses == 3);  // 양 끝 방향 전환 2번 + 마지막 정지 1번뿐
                assert(t < 25);       // 중간 정지가 없어 빨리 끝난다
            } else {
                assert(pauses > 2);  // 구간 안 20° 정지도 남아 있다
                assert(t < 60);
            }
        }
    }
    // 방향 전환 직후에는 kScanReversePauseS 동안 멈춘다.
    {
        gw::SectorScanState sector;
        assert(gw::sector_scan_rate(15, 120, 1, 10, 60 - gw::kScanSectorEndLeadDeg, sector) == 0);
        assert(sector.leg == 1 && sector.leg_start_deg == 60);
        assert(gw::sector_scan_rate(15, 120, 1, 10.5, 60, sector) == 0);  // 멈춤 중
        assert(gw::sector_leg_rate(15, 1, 120, sector) < 0);              // 다음은 반대 방향
        assert(gw::sector_scan_rate(15, 120, 1, 11.1, 60, sector) < 0);   // 멈춤 뒤 반대로 돈다
    }
    std::cout << "PASS sector scan: +60/-60/0 legs with early stop, continuous pauses only at "
                 "reversals\n";
    return 0;
}
