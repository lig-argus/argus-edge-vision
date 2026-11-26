// Space의 FramePoseAlignment 역할. 현재 SITL ATTITUDE/RPY 보간 chain을 명시한다.
#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
namespace follow {
struct AlignedFCPose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()}, rpy{Eigen::Vector3d::Zero()};
    double sensor_boot_s{0}, alignment_error_ms{0};
    std::uint64_t epoch{0};
};
struct FCPositionSample {
    std::uint32_t boot_ms{0};
    std::int64_t received_ns{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()}, velocity{Eigen::Vector3d::Zero()};
};
class FramePoseAlignment {
public:
    void position(std::uint32_t boot_ms, const Eigen::Vector3d&, const Eigen::Vector3d&,
                  std::int64_t received_ns);
    void attitude(std::uint32_t boot_ms, const Eigen::Vector3d&);
    AlignedFCPose aligned(double sensor_s, double max_delta_s, std::int64_t deadline_ns) const;
    std::optional<double> current_sim_s() const;
    std::optional<FCPositionSample> latest_position() const;
    std::optional<Eigen::Vector3d> latest_rpy() const;

private:
    struct Sample {
        double stamp;
        Eigen::Vector3d value;
    };
    double unwrap(std::uint32_t, bool attitude);
    mutable std::mutex mutex_;
    std::deque<Sample> positions_, attitudes_;
    std::optional<FCPositionSample> latest_;
    std::optional<std::uint32_t> last_position_ms_, last_attitude_ms_;
    std::uint64_t position_wraps_{0}, attitude_wraps_{0}, epoch_{0};
    bool epoch_invalid_{false};
};
}  // namespace follow
