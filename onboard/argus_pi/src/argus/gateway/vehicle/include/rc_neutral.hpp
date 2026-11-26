#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <set>

#include "argus/gateway/common/include/python_compat.hpp"

namespace argus::gateway {
class NeutralObservation {
public:
    NeutralObservation(double dwell = 1.0, double gap = 0.4, double freshness = 0.4)
        : dwell_(dwell), gap_(gap), freshness_(freshness) {
        if (!positive(dwell_) || !positive(gap_) || !positive(freshness_)) {
            throw ValueError("invalid observation timing");
        }
    }
    nlohmann::json reset(const std::string& reason) {
        since_.reset();
        previous_.reset();
        return {{"ready", false}, {"reason", reason}};
    }
    nlohmann::json update(double now, double stamp, const nlohmann::json& channels,
                          const nlohmann::json& parameters, bool healthy) {
        if (!std::isfinite(now) || !std::isfinite(stamp) || stamp > now ||
            now - stamp > freshness_ || !healthy) {
            return reset("RC 신선도·건강 상태 미확인");
        }
        if (!channels.is_object() || !parameters.is_object()) {
            return reset("RC 채널 또는 설정 미수신");
        }
        constexpr std::array<const char*, 4> kAxes{"ROLL", "PITCH", "YAW", "THROTTLE"};
        std::array<int, 4> mappings{};
        std::set<int> unique;
        for (std::size_t i = 0; i < kAxes.size(); ++i) {
            const auto raw = number(parameters, std::string("RC_MAP_") + kAxes.at(i));
            if (!raw || *raw != std::floor(*raw) || *raw < 1 || *raw > 18) {
                return reset(std::string("RC 축 매핑 미확인: ") + kAxes.at(i));
            }
            mappings.at(i) = static_cast<int>(*raw);
            unique.insert(mappings.at(i));
        }
        if (unique.size() != kAxes.size()) {
            return reset("RC 축 매핑 중복");
        }
        for (std::size_t i = 0; i < mappings.size(); ++i) {
            const auto channel = std::to_string(mappings.at(i));
            const auto prefix = "RC" + channel;
            const auto value = number(channels, "chan" + channel + "_raw");
            auto trim = number(parameters, prefix + "_TRIM");
            const auto dz = number(parameters, prefix + "_DZ");
            const auto lower = number(parameters, prefix + "_MIN");
            const auto upper = number(parameters, prefix + "_MAX");
            if (!value || !trim || !dz || !lower || !upper) {
                return reset("RC 보정값 또는 채널 미확인");
            }
            if (i == kAxes.size() - 1 && (*trim == *lower || *trim == *upper)) {
                const auto reverse = number(parameters, prefix + "_REV");
                if (!reverse) {
                    return reset("RC 스로틀 반전 보정값 미확인");
                }
                if ((*reverse >= 0 && *trim == *lower) || (*reverse < 0 && *trim == *upper)) {
                    if (*lower < 800 || *upper > 2200 || *lower >= *upper) {
                        return reset("RC 보정값 또는 채널 범위 오류");
                    }
                    trim = (static_cast<int>(*lower) + static_cast<int>(*upper)) / 2;
                }
            }
            if (*lower < 800 || *lower >= *trim || *trim >= *upper || *upper > 2200 || *dz < 0 ||
                *dz >= std::min(*trim - *lower, *upper - *trim) || *value < *lower ||
                *value > *upper) {
                return reset("RC 보정값 또는 채널 범위 오류");
            }
            // 2026-11-09 사용자 확정: FC RCx_DZ(10µs, 반행정의 약 2%)만으로는 손으로 중립을
            // 맞추기 어려워 RC→GUI 복귀가 계속 거부됐다. 반행정의 15%까지 중립으로 본다
            // (PX4 COM_RC_STICK_OV 30%보다 작다). DZ가 더 크면 DZ를 그대로 쓴다.
            const double tolerance =
                std::max(*dz, kNeutralFraction * std::min(*trim - *lower, *upper - *trim));
            if (std::abs(*value - *trim) > tolerance) {
                return reset("RC 스틱이 보정 중립 범위 밖");
            }
        }
        if (previous_ && (stamp < *previous_ || stamp - *previous_ > gap_)) {
            since_.reset();
        }
        since_ = since_.value_or(stamp);
        previous_ = stamp;
        const auto elapsed = stamp - *since_;
        const bool ready = elapsed >= dwell_;
        return {{"ready", ready},
                {"reason", ready ? "RC 중립 지속 확인" : "RC 중립 지속 확인 중"},
                {"neutral_duration_s", elapsed}};
    }

private:
    static constexpr double kNeutralFraction = .15;  // 중립 허용: 반행정 대비 비율
    static bool positive(double value) { return std::isfinite(value) && value > 0; }
    static std::optional<double> number(const nlohmann::json& data, const std::string& key) {
        const auto item = data.find(key);
        if (item == data.end() || !item->is_number()) {
            return std::nullopt;
        }
        const auto value = item->get<double>();
        return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
    }
    double dwell_;
    double gap_;
    double freshness_;
    std::optional<double> since_;
    std::optional<double> previous_;
};
}  // namespace argus::gateway
