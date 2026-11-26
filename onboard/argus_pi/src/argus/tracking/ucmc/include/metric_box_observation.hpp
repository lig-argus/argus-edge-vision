// 수정 D2의 픽셀 박스와 FC 정렬 지면 관측. GT는 이 자료형의 입력이 아니다.
#pragma once
#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
namespace follow {
struct MetricBoxObservation {
    std::array<double, 5> box{};
    Eigen::Vector2d point{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d covariance{Eigen::Matrix2d::Zero()};
    std::array<double, 2> anchor{};
};
struct UcmcAssociation {
    std::int64_t id{0};
    std::size_t detection{0};
};
}  // namespace follow
