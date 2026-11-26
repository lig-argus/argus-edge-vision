// 2026-11-12 탐색 표적 판정 시험: 시작 때 보이던 표적 무시, 새 표적은 화면 가운데에서 정지,
// 빠져나가는 표적의 ID 재부여 무시, 가운데 전 놓침·과회전 시 정지. FC·프로세스는 열지 않는다.
#include <exception>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "argus/gateway/bridge/include/scan_target_gate.hpp"

namespace gw = argus::gateway;

namespace {
void require(bool value, const char* message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}

constexpr double kWidth = 640.0;

gw::ScanTrack track(std::int64_t id, double x) { return gw::ScanTrack{1, id, x}; }

// 오른쪽(+20°/s)으로 돌며 30 fps로 본다. 정지 표적은 1°에 약 19 px 왼쪽으로 흐른다.
void ignores_target_visible_at_start() {
    gw::ScanTargetGate gate;
    gate.begin({track(4, 330)});
    for (int i = 0; i < 30; ++i) {
        const double x = 330 - (i * 13.0);
        require(gate.observe({track(4, x)}, kWidth, 20, i * .67, i / 30.0) ==
                    gw::ScanTargetDecision::Continue,
                "visible target must be ignored");
    }
    require(!gate.centering(), "ignored target must not lock");
}

void stops_when_new_target_reaches_center() {
    gw::ScanTargetGate gate;
    gate.begin({});
    gw::ScanTargetDecision last = gw::ScanTargetDecision::Continue;
    int frame = 0;
    for (; frame < 60; ++frame) {
        const double x = 600 - (frame * 13.0);  // 오른쪽 끝에서 들어와 왼쪽으로
        last = gate.observe({track(7, x)}, kWidth, 20, frame * .67, frame / 30.0);
        if (last != gw::ScanTargetDecision::Continue) {
            break;
        }
        require(x > kWidth / 2, "must not stop before center");
    }
    require(last == gw::ScanTargetDecision::Centered, "new target must stop at center");
    require(600 - (frame * 13.0) <= kWidth / 2, "stops once center is passed");
}

void left_rotation_mirrors() {
    gw::ScanTargetGate gate;
    gate.begin({});
    require(gate.observe({track(2, 60)}, kWidth, -20, 0, 0) == gw::ScanTargetDecision::Continue,
            "left entry continues");
    require(gate.observe({track(2, 330)}, kWidth, -20, 15, .7) == gw::ScanTargetDecision::Centered,
            "left rotation stops past center");
}

void ignores_reidentified_leaving_target() {
    gw::ScanTargetGate gate;
    gate.begin({track(4, 200)});
    // 시작 표적이 왼쪽으로 빠지다 새 ID 9로 다시 잡힌다: 이미 가운데를 지난 쪽이므로 무시
    require(gate.observe({track(9, 120)}, kWidth, 20, 4, .2) == gw::ScanTargetDecision::Continue,
            "leaving re-id must be ignored");
    require(!gate.centering(), "leaving re-id must not lock");
    require(gate.observe({track(9, 40)}, kWidth, 20, 8, .4) == gw::ScanTargetDecision::Continue,
            "stays ignored");
}

void stops_if_lost_or_overrotated() {
    gw::ScanTargetGate lost;
    lost.begin({});
    require(lost.observe({track(3, 600)}, kWidth, 20, 0, 0) == gw::ScanTargetDecision::Continue,
            "lock");
    require(lost.observe({}, kWidth, 20, 10, .9) == gw::ScanTargetDecision::Continue,
            "short miss waits");
    require(lost.observe({}, kWidth, 20, 21, 1.05) == gw::ScanTargetDecision::Found,
            "lost for 1s stops");

    gw::ScanTargetGate far;
    far.begin({});
    require(far.observe({track(3, 600)}, kWidth, 20, 100, 0) == gw::ScanTargetDecision::Continue,
            "lock");
    require(far.observe({track(3, 500)}, kWidth, 20, 141, .5) == gw::ScanTargetDecision::Found,
            "over 40 deg stops");
}

void inactive_gate_never_stops() {
    gw::ScanTargetGate gate;
    require(gate.observe({track(1, 320)}, kWidth, 20, 0, 0) == gw::ScanTargetDecision::Continue,
            "inactive continues");
    gate.begin({});
    gate.reset();
    require(!gate.active(), "reset deactivates");
}
}  // namespace

int main() try {
    ignores_target_visible_at_start();
    stops_when_new_target_reaches_center();
    left_rotation_mirrors();
    ignores_reidentified_leaving_target();
    stops_if_lost_or_overrotated();
    inactive_gate_never_stops();
    std::cout << "PASS scan target gate\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
