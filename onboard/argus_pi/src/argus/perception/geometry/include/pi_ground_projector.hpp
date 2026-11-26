#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/perception/geometry/include/device_clock_alignment.hpp"
#include "argus/perception/geometry/include/frame_transform.hpp"

namespace follow {
struct PiMetricDetection {
    std::int32_t class_id;
    std::string label;
    MetricBoxObservation metric;
    NativeJson thermal;  // 2026-11-09 열 외형 (없으면 null)
};
struct PiProjectedFrame {
    AlignedFCPose pose;
    std::int64_t sample_ns;
    std::vector<PiMetricDetection> detections;
    NativeJson diagnostic;  // Optional observation evidence, never used for admission.
    bool diagnostic_lost{false};
    std::int64_t motion_sample_ns{0};
    std::string alignment_clock{"host_receipt"};
    std::int64_t attitude_extrapolated_ns{0};  // 2026-11-16: 자세를 외삽했으면 그 시간 (진단용)
    // 2026-11-19 PAMPC #5: 지면 투영에 실제로 쓴 자세(rad, 시각 보정과 pitch 배율 적용 뒤)와
    // 투영 평면 down(m, NED, EKF z와 같은 좌표). pose와 기존 필드의 의미는 바꾸지 않는다.
    Eigen::Vector3d projection_rpy{Eigen::Vector3d::Zero()};
    double ground_plane_down_m{0};
};
// Receipt alignment is explicitly an estimate, never certified exposure alignment.
class PiGroundProjector final {
public:
    PiGroundProjector(NativeJson geometry, NativeJson measurement, std::string expected_uid);
    PiProjectedFrame project(const NativeJson& pixels, const NativeJson& fc,
                             std::int64_t now_ns) const;
    // 2026-11-10 카메라·FC 시계 정렬 추정을 FC 세대 변경 때와 같게 처음부터 다시 잡는다.
    // 실내 17:36~17:49: 데이터는 정상(새 추정 p95 16ms)인데 추적기 안의 추정만 어긋나 모든
    // 프레임이 "SDK/FC clock alignment exceeds 50ms"로 실패했고, 추적기 재시작으로만 풀렸다.
    void reset_clock_alignment() const {
        camera_clock_ = DeviceClockAlignment{};
        fc_clock_ = DeviceClockAlignment{};
        clock_source_.clear();
    }
    // 2026-11-12 진단: 카메라·FC 시계 정렬 추정 상태 (판단에는 쓰지 않는다).
    NativeJson clock_alignment_state() const {
        const auto state = [](const DeviceClockAlignment& clock) {
            return NativeJson{{"ready", clock.ready()},
                              {"offset_ns", clock.offset_ns()},
                              {"minimum_offset_ns", clock.minimum_offset_ns()},
                              {"samples", clock.sample_count()}};
        };
        return NativeJson{
            {"camera", state(camera_clock_)},
            {"fc", state(fc_clock_)},
            {"attitude_correction",
             {{"pitch_scale", pitch_scale_},
              {"attitude_offset_ms", static_cast<double>(attitude_offset_ns_) / 1e6}}},
            {"attitude_extrapolation",
             {{"max_ms", static_cast<double>(max_extrapolation_ns_) / 1e6},
              {"max_rate_deg_s", max_extrapolation_rate_rad_s_ * 180. / 3.141592653589793}}}};
    }
    // 2026-11-16 사용자 지시(외삽 2단계): 노출 뒤 FC 자세가 아직 없으면 기다리지 않고 직전 자세와
    // 몸체 각속도로 외삽한다. 0ms면 꺼짐(기존 동작). 1단계 근거로 상한 0..33ms, 각속도 5..180°/s.
    void set_attitude_extrapolation(double max_ms, double max_rate_deg_s) {
        if (!std::isfinite(max_ms) || max_ms < 0. || max_ms > 33.) {
            throw std::invalid_argument("attitude extrapolation must be 0..33 ms");
        }
        if (!std::isfinite(max_rate_deg_s) || max_rate_deg_s < 5. || max_rate_deg_s > 180.) {
            throw std::invalid_argument("attitude extrapolation rate limit must be 5..180 deg/s");
        }
        max_extrapolation_ns_ = static_cast<std::int64_t>(std::llround(max_ms * 1e6));
        max_extrapolation_rate_rad_s_ = max_rate_deg_s * 3.141592653589793 / 180.;
    }
    bool attitude_extrapolation_active() const { return max_extrapolation_ns_ > 0; }
    // 2026-11-12 임시 소프트웨어 보정(사용자 지시): 지면 투영에 쓰는 자세만 바꾼다.
    // 결정 로그 Replay에서 영상이 FC 몸체 pitch보다 약 1.4배 크게·약 50ms 앞서 기울었다
    // (정지 표적 위치 흔들림 실외 34–54%, 실내 72–82%로 감소). 기본 1.0/0ms = 기존 동작.
    // 발행하는 pose·레이저 높이 기울기 보정에는 쓰지 않는다. 카메라 장착 확인 전 임시책.
    void set_attitude_correction(double pitch_scale, std::int64_t attitude_offset_ns) {
        if (!std::isfinite(pitch_scale) || pitch_scale < .5 || pitch_scale > 2.) {
            throw std::invalid_argument("camera pitch scale must be 0.5..2.0");
        }
        if (attitude_offset_ns < -200000000LL || attitude_offset_ns > 200000000LL) {
            throw std::invalid_argument("camera attitude offset must be -200..200 ms");
        }
        pitch_scale_ = pitch_scale;
        attitude_offset_ns_ = attitude_offset_ns;
    }
    bool attitude_correction_active() const {
        return pitch_scale_ != 1. || attitude_offset_ns_ != 0;
    }

private:
    NativeJson geometry_, measurement_;
    std::string expected_uid_;
    mutable DeviceClockAlignment camera_clock_, fc_clock_;
    mutable std::string clock_source_;
    mutable std::uint64_t camera_serial_{0}, clock_epoch_{0};
    double pitch_scale_{1.};
    std::int64_t attitude_offset_ns_{0};
    std::int64_t max_extrapolation_ns_{0};  // 2026-11-16: 0 = 외삽 꺼짐
    double max_extrapolation_rate_rad_s_{0.};
};
}  // namespace follow
