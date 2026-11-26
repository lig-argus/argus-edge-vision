// 실기체 관제 연결부 (Pi tools/real_fc/vehicle.py FlightLink와 같은 규칙).
// 한 스레드만 MAVLink 링크를 읽고 쓴다. HTTP·메인 GUI 연결부는 mutex()를 잡고 같은 상태를 본다.
//
// 원본처럼 다른 구성요소(추종 송신부, 메인 GUI 연결, 수동 yaw, RC 회수)가 판정 함수와 상태를
// 직접 쓰므로 판정 함수는 공개하고 상태는 state()로 연다. 이 함수들은 호출하는 쪽이 mutex()를
// 잡고 있어야 한다 (원본 RLock과 같다). start/step/stop과 HTTP 진입 함수는 스스로 잡는다.
#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "argus/gateway/events/include/event_log.hpp"
#include "argus/gateway/mavlink/include/mavlink_codec.hpp"
#include "argus/gateway/mavlink/include/mavlink_link.hpp"
#include "argus/gateway/ownership/include/command_owner.hpp"
#include "argus/gateway/settings/include/flight_limits.hpp"
#include "argus/gateway/vehicle/include/follow_supervisor.hpp"
#include "argus/gateway/vehicle/include/mag_interference.hpp"

namespace argus::gateway {

using Json = nlohmann::json;
// 원본 job dict. 같은 객체를 pending·last_job·jobs가 함께 가리킨다.
using Job = std::shared_ptr<Json>;

struct FlightLinkOptions {
    std::string expected_uid;
    FlightLimits limits{};
    std::function<double()> clock;         // 원본 time.monotonic (초)
    std::filesystem::path event_metadata;  // fc-events.json
    std::string command_owner_path;        // ARGUS_COMMAND_OWNER_FILE (빈 값이면 잠금 없음)
    std::string profile_state_path;        // ARGUS_PROFILE_REBOOT_FILE (빈 값이면 없음)
    LinkFactory link_factory;              // 기본 open_link
    std::function<std::string()> new_id;   // 원본 uuid.uuid4().hex
};

struct Sample {
    Json data;
    double at{0.0};
};

// 원본 FlightLink 인스턴스 속성
struct FlightState {
    std::unique_ptr<ILink> link;
    std::unique_ptr<MavlinkSender> mav;      // 245/191 (원본 link.mav)
    std::unique_ptr<MavlinkSender> gcs_mav;  // 244/190 (원본 gcs_mav)
    std::unique_ptr<EventLog> fc_events;
    double last_tx{0.0};
    double last_heartbeat_tx{0.0};
    double last_gcs_tx{-std::numeric_limits<double>::infinity()};
    double next_param_read_retry{-std::numeric_limits<double>::infinity()};
    double last_probe{-std::numeric_limits<double>::infinity()};
    double lease_at{-std::numeric_limits<double>::infinity()};
    double gcs_at{-std::numeric_limits<double>::infinity()};
    std::unique_ptr<IFollowSupervisor> follow_b;
    Job last_job;
    Job pending;
    // 원본처럼 같은 객체인지(is)로 목표 교체를 판단하므로 공유 포인터로 둔다
    std::shared_ptr<Json> local_target;
    std::string connection;
    std::string profile_state_path;
    std::optional<std::string> uid;
    std::optional<std::string> firmware;
    std::optional<std::string> cmc_stream_environment;
    std::optional<std::string> owner;
    std::optional<std::string> gcs_client;
    std::map<std::string, Sample> samples;
    std::map<std::string, std::deque<Json>> pose_history;
    std::map<std::string, Json> sensor_stamps;
    std::map<std::string, std::int64_t> boot_stamps;
    std::map<std::string, Json> params;
    std::map<std::string, int> param_types;
    std::map<std::string, double> param_read_retry_at;
    std::deque<Json> events;
    std::deque<Job> jobs;
    MavlinkParser parser;
    int pose_epoch{0};
    bool profile_reboot_required{false};
    bool holding{false};
    bool external{false};
    bool owned_flight{false};
    bool landing{false};
    bool landing_mode_seen{false};
    bool config_requested{false};
    bool param_bytewise{true};
    bool owned_mode_confirmed{false};
};

class FlightLink {
public:
    static constexpr std::size_t kMaxEvents = 60;
    static constexpr std::size_t kMaxPoseHistory = 64;
    static constexpr int kMaxMessagesPerStep = 128;

    explicit FlightLink(FlightLinkOptions options);
    FlightLink(const FlightLink&) = delete;
    FlightLink& operator=(const FlightLink&) = delete;
    FlightLink(FlightLink&&) = delete;
    FlightLink& operator=(FlightLink&&) = delete;
    virtual ~FlightLink();

    // 원본 ARGUS_* 환경 변수와 설치 경로로 기본 선택값을 만든다.
    static FlightLinkOptions default_options(const std::string& expected_uid,
                                             const std::filesystem::path& event_metadata);

    // ---- 스스로 잠그는 진입 함수 (원본 공개 메서드)
    void start();  // 10ms 주기 소유 스레드 시작
    void step();   // 소유 스레드 한 바퀴 (시험은 직접 부른다)
    void stop();   // 지상·무장 해제 확인 후 스레드 종료
    void keepalive(const Json& client, bool ground_station = false);
    void request_connect(const Json& address, const Json& baud, const Json& client);
    virtual Json submit(const Json& action, const Json& client, const Json& height);
    virtual Json snapshot();
    // 2026-11-16: HOLD 안정 판단 입력만 snapshot()과 같은 식으로 만든다 (주기 갱신용, 가벼움).
    // fresh·identity_ok·mode·armed·ground·local_ok·height_m·height_age_ms·flight_environment
    Json hold_view();
    Json diagnostics();
    Json observations();
    virtual Json yaw_handoff();

    std::recursive_mutex& mutex() { return lock_; }
    FlightState& state() { return state_; }
    const FlightState& state() const { return state_; }
    const std::string& expected_uid() const { return expected_uid_; }
    const FlightLimits& limits() const { return limits_; }
    double now() const { return clock_(); }
    std::string new_id() const { return new_id_(); }

    // ---- 판정·동작 (호출하는 쪽이 mutex()를 잡는다)
    void record_event(const std::string& text);
    const Json* sample(const std::string& kind) const;
    double age(const std::string& kind) const;
    bool fresh(const std::string& kind, double ttl) const;
    bool armed() const;
    std::optional<std::string> mode() const;
    std::optional<bool> ground() const;
    bool imu_ok() const;
    std::optional<bool> prearm_ok() const;
    bool local_ok() const;
    std::string position_accuracy_reason() const;
    bool global_ok() const;
    bool matches_profile(const ParameterProfile& profile) const;
    std::optional<std::string> environment() const;
    bool baro_ok() const;
    std::string outdoor_navigation_reason() const;
    std::string navigation_reason() const;
    std::optional<Json> home_signature() const;
    std::string height_sample_kind() const;
    std::pair<std::optional<double>, std::string> height() const;
    bool flow_ok() const;
    bool profile_ok() const;
    std::string reason(const std::string& action) const;
    bool gcs_active() const;
    double validate_height(const Json& height) const;

    // 2026-11-18 데모 모드: 선택 또는 실행 중 추종 프로필이 요구하는 이륙/고도 변경 범위 m.
    // 비어 있으면 기존 한계만 쓴다. 원천은 FC 링크 잠금 안에서 불리므로 이 잠금을 잡지 않는다.
    struct HeightBand {
        double minimum;
        double maximum;
        std::string message;  // 범위 밖 거부 문구
    };
    using HeightBandSource = std::function<std::optional<HeightBand>()>;
    // 스스로 mutex()를 잡는다 (실행 중에 바꿀 수 있다)
    void set_height_band_source(HeightBandSource source);
    // 호출하는 쪽이 mutex()를 잡는다
    std::optional<HeightBand> height_band() const;

    // 2026-11-18 데모 위치 고정: 실행 중 추종 프로필이 실내 demo인가 (참, 거짓, 모르면 비어 있음).
    // 추종 송신부가 송신할 때 부른다. 원천은 FC 링크 잠금 안에서 불리므로 이 잠금을 잡지 않는다.
    using DemoFollowSource = std::function<std::optional<bool>()>;
    // 스스로 mutex()를 잡는다 (실행 중에 바꿀 수 있다)
    void set_demo_follow_source(DemoFollowSource source);
    // 호출하는 쪽이 mutex()를 잡는다. 원천이 없으면 비어 있다 (데모로 보지 않는다).
    std::optional<bool> demo_follow() const;

    // 2026-11-18 이륙 조건 보강: FC 강한 자기장 간섭 경고 (호출하는 쪽이 mutex()를 잡는다).
    const MagInterferenceMonitor& mag_interference() const { return mag_interference_; }
    // ARM 거부 문구 (경고 활성일 때), 해당 없으면 빈 값
    std::string mag_arm_reason() const;
    // 이륙 거부 문구 (무장 순간 경고 활성이었을 때, 무장 해제까지), 해당 없으면 빈 값
    std::string mag_takeoff_reason() const;

    void send_command(std::uint16_t command, const std::vector<double>& args = {});
    // 원본 link.mav.set_position_target_local_ned_send(0, 1, 1, frame, mask, *values)
    void send_position_target(std::uint8_t frame, std::uint16_t mask,
                              const std::array<float, 11>& values);
    void send_mode(const std::string& mode);
    void zero();
    void cancel(const std::string& message);
    double ramped_z() const;
    void local_setpoint();
    virtual void finish(const std::string& status, std::string message);
    virtual void close_link();
    // 원본 reclaim_context = None (RC 복귀가 없는 링크는 할 일이 없다)
    virtual void clear_reclaim_context() {}

protected:
    void stop_worker();
    bool stopping() const { return quit_.load(); }
    void store_profile_reboot_requirement();
    void advance_profile(const Job& job);
    virtual void dispatch(const Job& job);
    void issue(const Job& job);
    bool observed(const Json& job) const;
    // 2026-11-24: 이륙 상승 멈춤 판정 (대기 중 이륙 작업에 stall_since를 기록한다)
    bool takeoff_climb_stalled();
    virtual void ingest(const InboundMessage& message);
    Json cmc_stream_status() const;
    virtual void configure();
    void refresh_missing_params();
    virtual void supervise();
    // 원본 _run 한 바퀴의 몸통 (잠금은 step()이 잡는다)
    void run_once();
    void write(const Bytes& bytes) const;
    bool follow_busy() const;
    // 2026-11-18: 자기장 간섭 경고 주기 판정 (run_once가 부른다)
    void update_mag_interference();

private:
    void thread_main();
    // 신선한 heartbeat의 무장 상태 (만료면 비어 있음)
    std::optional<bool> known_armed() const;
    void record_mag_change(MagInterferenceMonitor::Change change);

    const std::string expected_uid_;
    const FlightLimits limits_;
    const std::function<double()> clock_;
    const std::filesystem::path event_metadata_;
    const LinkFactory link_factory_;
    const std::function<std::string()> new_id_;
    std::recursive_mutex lock_;
    FlightState state_;
    CommandOwner command_owner_;
    HeightBandSource height_band_source_;
    DemoFollowSource demo_follow_source_;
    MagInterferenceMonitor mag_interference_;
    std::atomic<bool> quit_{false};
    std::thread thread_;

public:
    const CommandOwner& command_owner() const { return command_owner_; }
    CommandOwner& command_owner() { return command_owner_; }
};

// 원본 MODES (주 모드, 부 모드)
std::optional<std::pair<int, int>> mode_numbers(const std::string& mode);

}  // namespace argus::gateway
