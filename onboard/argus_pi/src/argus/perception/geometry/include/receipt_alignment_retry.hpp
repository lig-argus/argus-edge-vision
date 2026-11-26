#pragma once

#include <chrono>
#include <stdexcept>
#include <string>

#include "argus/perception/geometry/include/pi_ground_projector.hpp"

namespace follow {
// FC 표본이 아직 도착하지 않아 50ms 정렬에 실패한 경우만 새 FC 표본을 기다린다.
// 수신 시각 방식과 SDK 시각 방식 모두 해당한다. project()의 나이·정렬 검사는 그대로 유지한다.
inline bool is_waitable_alignment_failure(const std::string& reason) {
    return reason == "FC receipt alignment exceeds 50ms" ||
           reason == "SDK/FC clock alignment exceeds 50ms" ||
           reason == "FC attitude interpolation requires fresh brackets within 50ms";
}

// Projector는 PiGroundProjector와 같은 project(pixels, fc, now)를 제공해야 한다 (시험용 대역 허용).
template <class Projector, class Reader, class Now, class Pause>
auto project_with_receipt_retry(const Projector& projector, const NativeJson& pixels,
                                Reader read_fc, Now now, Pause pause,
                                std::chrono::milliseconds maximum_wait)
    -> decltype(projector.project(pixels, read_fc(), now())) {
    if (maximum_wait.count() < 0 || maximum_wait.count() > 50) {
        throw std::invalid_argument("receipt retry must be between 0 and 50ms");
    }
    const auto started = now();
    const auto deadline_ns = maximum_wait.count() * 1000000;
    auto fc = read_fc();
    const auto epoch = fc.at("epoch");
    const auto session = fc.value("onboard_session", std::string{});
    if (maximum_wait.count() > 0 && session.empty()) {
        throw std::runtime_error("FC receipt retry context missing");
    }
    while (true) {
        if (fc.at("epoch") != epoch || fc.value("onboard_session", std::string{}) != session) {
            throw std::runtime_error("FC context changed during receipt retry");
        }
        std::string reason;
        try {
            return projector.project(pixels, fc, now());
        } catch (const std::runtime_error& error) {
            reason = error.what();
            if (!is_waitable_alignment_failure(reason) || now() - started >= deadline_ns) {
                throw;
            }
        }
        pause();
        if (now() - started > deadline_ns) {
            // 원래 거부 사유를 그대로 남긴다.
            throw std::runtime_error(reason);
        }
        fc = read_fc();
    }
}
}  // namespace follow
