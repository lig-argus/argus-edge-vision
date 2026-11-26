// 2026-11-10 auto 투영 모드 전환 히스테리시스.
// fc-ground ↔ camera 전환은 UCMC 추적기와 세대(generation)를 초기화해 모든 ID를 바꾸고 GUI 선택을
// 푼다. 실내 17:07~17:18 기록에서 FC 자세 정렬 실패(50ms 브래킷·시계 정렬)가 짧게 반복되며 약
// 10분에 27~29번 전환됐다. 짧은 실패는 그 프레임만 버리고 fc-ground를 유지하고, 카메라에서 돌아올
// 때는 FC 투영이 일정 시간 계속 성공해야 돌아간다. 실패가 길면 기존처럼 카메라 전용으로 넘어간다.
#pragma once

#include <cstdint>
#include <string>

namespace follow {

struct ProjectionHysteresis {
    // 마지막 fc-ground 발행 뒤 이 시간까지는 FC 투영이 실패해도 카메라로 넘어가지 않는다(기존
    // 250ms).
    std::int64_t fallback_grace_ns{1500000000};
    // 카메라 전용에서 fc-ground로 돌아가려면 FC 투영이 이 시간 동안 계속 성공해야 한다(기존 즉시).
    std::int64_t return_stable_ns{1000000000};
    std::int64_t fc_ok_since_ns{0};

    // FC 투영 실패. true면 fc-ground를 유지하고 이 프레임만 버린다. false면 카메라 전용으로
    // 넘어간다.
    bool hold_fc_ground_on_failure(const std::string& previous_geometry,
                                   std::int64_t last_published_sample_ns, std::int64_t now_ns) {
        fc_ok_since_ns = 0;
        return previous_geometry == "fc-ground" && last_published_sample_ns > 0 &&
               now_ns >= last_published_sample_ns &&
               now_ns - last_published_sample_ns < fallback_grace_ns;
    }

    // FC 투영 성공. true면 fc-ground 결과를 쓴다. false면 아직 안정 확인 전이라 이번 프레임은
    // 카메라로 처리한다.
    bool accept_fc_ground(const std::string& previous_geometry, std::int64_t now_ns) {
        if (previous_geometry != "camera") {
            fc_ok_since_ns = 0;
            return true;  // 시작 직후이거나 이미 fc-ground
        }
        if (fc_ok_since_ns == 0) {
            fc_ok_since_ns = now_ns;
        }
        return now_ns - fc_ok_since_ns >= return_stable_ns;
    }
};

}  // namespace follow
