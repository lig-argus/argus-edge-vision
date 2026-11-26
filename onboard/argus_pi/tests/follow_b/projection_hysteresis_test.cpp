// 2026-11-10 auto 투영 모드 전환 히스테리시스 시험 (실내 17:07~17:18 fc-ground↔camera 반복 전환).
#include "argus/runtime/follow/include/projection_hysteresis.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>

using follow::ProjectionHysteresis;

namespace {
constexpr std::int64_t ms = 1000000;

// 프레임마다 FC 투영 성공 여부를 넣고 발행된 투영 모드와 전환 횟수를 센다(기존 코드 흐름과 같게).
struct Sim {
    ProjectionHysteresis h;
    std::string previous;
    std::int64_t last_published = 0;
    int switches = 0;
    void frame(std::int64_t now, bool fc_ok) {
        bool camera = false;
        if (!fc_ok) {
            if (h.hold_fc_ground_on_failure(previous, last_published, now)) {
                return;  // 이 프레임만 버림
            }
            camera = true;
        } else if (!h.accept_fc_ground(previous, now)) {
            camera = true;
        }
        const std::string current = camera ? "camera" : "fc-ground";
        if (!previous.empty() && previous != current) {
            ++switches;
        }
        previous = current;
        last_published = now;
    }
};
}  // namespace

int main() {
    // 1. 짧은 실패(0.5초)가 반복돼도 fc-ground를 유지한다
    {
        Sim s;
        std::int64_t t = 0;
        for (int round = 0; round < 10; ++round) {
            for (int i = 0; i < 30; ++i, t += 66 * ms) {
                s.frame(t, true);  // 2초 정상
            }
            for (int i = 0; i < 8; ++i, t += 66 * ms) {
                s.frame(t, false);  // 0.5초 실패
            }
        }
        std::cout << "short failures: switches " << s.switches << " final " << s.previous << '\n';
        assert(s.switches == 0 && s.previous == "fc-ground");
    }
    // 2. 긴 실패(2초)는 카메라로 넘어가고, 회복은 1초 연속 성공 뒤에만
    {
        Sim s;
        std::int64_t t = 0;
        for (int i = 0; i < 30; ++i, t += 66 * ms) {
            s.frame(t, true);
        }
        for (int i = 0; i < 30; ++i, t += 66 * ms) {
            s.frame(t, false);
        }
        assert(s.previous == "camera" && s.switches == 1);
        // 0.5초 성공 뒤 한 번 실패 → 아직 카메라, 안정 시간 다시 시작
        for (int i = 0; i < 8; ++i, t += 66 * ms) {
            s.frame(t, true);
        }
        assert(s.previous == "camera");
        s.frame(t, false);
        t += 66 * ms;
        for (int i = 0; i < 14; ++i, t += 66 * ms) {
            s.frame(t, true);  // 0.92초
        }
        assert(s.previous == "camera");
        for (int i = 0; i < 3; ++i, t += 66 * ms) {
            s.frame(t, true);  // 1초 넘김(16프레임째 990ms)
        }
        std::cout << "long failure: switches " << s.switches << " final " << s.previous << '\n';
        assert(s.previous == "fc-ground" && s.switches == 2);
    }
    // 3. 시작 직후 첫 성공은 바로 fc-ground
    {
        Sim s;
        s.frame(0, true);
        assert(s.previous == "fc-ground");
    }
    std::cout << "PASS projection hysteresis: short FC misses keep IDs, return needs 1s stable\n";
    return 0;
}
