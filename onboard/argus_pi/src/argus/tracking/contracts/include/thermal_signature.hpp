// 2026-11-09 열 외형(같은 대상 확인 보조). 14bit RAW 박스의 배경 대비와 밝기 분포를 담는다.
// 배열 순서: [0] 주변 배경 RAW 중앙값, [1] 박스 대비 95%, [2] 뜨거운 절반의 평균 대비,
// [3..18] 대비/대비95% 를 0~1.2로 자른 16칸 정규화 히스토그램. 절대 온도 보정값이 아니다.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace follow {

inline constexpr std::size_t kThermalHistogramBins = 16;
inline constexpr std::size_t kThermalSignatureSize = 3 + kThermalHistogramBins;
inline constexpr double kThermalHistogramRange = 1.2;
using ThermalSignature = std::array<double, kThermalSignatureSize>;

struct ThermalSimilarity {
    double contrast_ratio{1};  // 작은 대비 / 큰 대비 (0~1)
    double histogram{1};       // 히스토그램 교집합 (0~1)
};

inline bool valid_thermal_signature(const ThermalSignature& signature) {
    double total = 0;
    for (std::size_t i = 0; i < kThermalSignatureSize; ++i) {
        if (!std::isfinite(signature.at(i))) {
            return false;
        }
        if (i >= 3) {
            if (signature.at(i) < 0 || signature.at(i) > 1) {
                return false;
            }
            total += signature.at(i);
        }
    }
    return signature.at(1) > 0 && std::abs(total - 1) < 1e-3;
}

inline ThermalSimilarity compare_thermal(const ThermalSignature& a, const ThermalSignature& b) {
    ThermalSimilarity result;
    result.contrast_ratio = std::min(a.at(1), b.at(1)) / std::max(a.at(1), b.at(1));
    result.histogram = 0;
    for (std::size_t i = 3; i < kThermalSignatureSize; ++i) {
        result.histogram += std::min(a.at(i), b.at(i));
    }
    return result;
}

// 추적기가 감지 JSON의 "thermal"을 트랙에 실어도 되는지 본다 (숫자 19개, 형식 유효).
template <typename Json>
bool thermal_wire_valid(const Json& value) {
    if (!value.is_array() || value.size() != kThermalSignatureSize) {
        return false;
    }
    ThermalSignature signature{};
    for (std::size_t i = 0; i < kThermalSignatureSize; ++i) {
        if (!value.at(i).is_number()) {
            return false;
        }
        signature.at(i) = value.at(i).template get<double>();
    }
    return valid_thermal_signature(signature);
}

}  // namespace follow
