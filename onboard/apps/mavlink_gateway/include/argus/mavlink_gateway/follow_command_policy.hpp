// 통제기 추종 명령(TRACK_START, SCAN_START)의 매개변수 변환과 mavlink_gateway 쪽 검사.
// 입력 구조체만 받고 MAVLink 송수신, 시계, bus 를 모른다
// 규약: docs/spec/icd.md 4.1 (검사 분담, 정수 매개변수), 4.2 (③ FOLLOWABLE, ④ 최신 트랙, 범위),
//      6장 (FAULT, MANUAL 에서의 복구)
// 사용: TrackStartHandler, ScanStartHandler, TrackStopHandler, FollowCommandTracker (스캔 진행률)
#ifndef ARGUS_ONBOARD_FOLLOW_COMMAND_POLICY_HPP_
#define ARGUS_ONBOARD_FOLLOW_COMMAND_POLICY_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "argus/mavlink.h"
#include "argus/mavlink_gateway/sensor_source_map.hpp"
#include "argus/perception_api/types.hpp"
#include "argus/vehicle_api/types.hpp"

namespace argus::mavlink_gateway {

// 명령 매개변수로 정확하게 전달되는 정수의 최댓값 2^24 - 1 (icd 4.1)
inline constexpr std::uint32_t kMaxCommandInteger = 16'777'215;

// TRACK_START ④ 의 최신 트랙: 프레임 시각이 명령 수신 시각 기준 이 시간 이내 (icd 4.2)
inline constexpr std::int64_t kLatestTrackMaxAgeNs = 250'000'000;

// SCAN_START 의 NaN 매개변수에 넣는 기본값과 범위 (icd 4.2)
inline constexpr double kDefaultScanYawRangeRad = 2.0 * 3.14159265358979323846;  // 2π, 상한
inline constexpr double kDefaultScanYawRateRadS =
    10.0 * 3.14159265358979323846 / 180.0;  // 시계 방향
inline constexpr double kDefaultScanTimeoutS = 60.0;
inline constexpr double kMaxScanYawRateRadS = 75.0 * 3.14159265358979323846 / 180.0;

// mavlink_gateway 가 추종 제어에 요청하지 않고 직접 보내는 결과
struct CommandRejection {
    MAV_RESULT result = MAV_RESULT_DENIED;  // DENIED 또는 TEMPORARILY_REJECTED
    std::string reason;
};

// TRACK_START 의 매개변수 (p1 track_id, p2 sensor, p3 class_id, p4 generation)
struct TrackStartParams {
    ARGUS_SENSOR sensor = ARGUS_SENSOR_UNKNOWN;  // IR 또는 EO
    std::uint32_t track_id = 0;                  // 1~16,777,215
    std::int32_t class_id = 0;                   // 0~16,777,215
    std::uint32_t generation = 0;                // 1~16,777,215
};

// p1, p4 가 1~16,777,215 의 정수, p3 가 0~16,777,215 의 정수, p2 가 IR 또는 EO 이면 변환한 값.
// 그 밖에는 DENIED
std::variant<TrackStartParams, CommandRejection> parse_track_start(
    const mavlink_command_long_t& command);

// p2 센서의 설정. sensors 에 없으면 nullopt
std::optional<SensorSource> find_sensor(const SensorSourceMap& sensors, ARGUS_SENSOR sensor);

// TRACK_START ③ ④ 의 입력
struct TrackSelection {
    TrackStartParams params;

    // p2 센서의 FOLLOWABLE (ARGUS_SENSOR_STATUS 와 같은 기준, compose_sensor_status)
    bool followable = false;

    // p2 센서의 최신 tracking/tracks. 받지 못했으면 nullopt
    std::optional<perception_api::Tracks> tracks;
    std::int64_t received_ns = 0;  // 명령 수신 시각 (monotonic)
};

// - ③ followable 이 false → TEMPORARILY_REJECTED
// - ④ tracks 가 없거나 프레임 시각이 received_ns 기준 kLatestTrackMaxAgeNs 를 넘음 → DENIED.
//   generation 이 다름(추적기 재시작) → DENIED. (track_id, class_id) 인 CONFIRMED 트랙이 없음 →
//   DENIED
// - 통과하면 추종 제어에 보낼 표적. label 은 그 트랙의 값
std::variant<vehicle_api::TrackTarget, CommandRejection> select_track_target(
    const TrackSelection& selection);

// NaN 인 매개변수를 기본값으로 바꾼 뒤 범위를 검사한다. p1 이 0 초과 2π 이하, p2 가 0 이 아니고
// 절댓값이 kMaxScanYawRateRadS 이하, p3 가 p1 / |p2| 초과이면 세 값을 모두 넣은 요청. 그 밖에는
// DENIED
std::variant<vehicle_api::ScanRequest, CommandRejection> parse_scan_start(
    const mavlink_command_long_t& command);

// TRACK_STOP 에서 추종 제어에 stop 을 요청할지: follow/status 를 now_ns 에서 kFollowStatusTimeoutNs
// 이내에 받았고 mission_state 가 IDLE, HOLD_WAIT 이외이면 true. 추종, 스캔의 정지뿐 아니라 MANUAL,
// FAULT 에서 HOLD_WAIT 로의 복구도 추종 제어가 stop 을 받아 처리한다 (state_machine.md)
bool follow_stop_needed(const std::optional<vehicle_api::ReceivedFollowStatus>& latest,
                        std::int64_t now_ns);

// SCAN_START 의 IN_PROGRESS progress: 회전한 yaw(progress_deg) / yaw_range_rad × 100, 0~100
std::uint8_t scan_progress_percent(double progress_deg, double yaw_range_rad);

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_FOLLOW_COMMAND_POLICY_HPP_
