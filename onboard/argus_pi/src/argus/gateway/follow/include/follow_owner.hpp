// B 추종 송신부 (Pi tools/real_fc/follow_b_owner.py FollowBOwner, manual_yaw.py ManualYaw와 같은
// 규칙). FlightLink의 MAVLink 소유 스레드에서만 돈다. 두 번째 MAVLink 연결·자동 ARM·지상 우회가
// 없다. 추종 제어기와는 ZMQ(PUB fc/state·fc/attitude·fc/mode_event, REP RPC, SUB control/setpoint·
// follow/status, SUB tracking/tracks)로 msgpack을 주고받는다 (2026-11-25 batch 2 부터 tracks 는
// 팀 [이름][protobuf], tracks_protobuf.hpp 로 예전 msgpack 을 되살린다). 전송은 IFollowTransport로
// 바꿔 끼운다. 2026-11-25 batch 6a: fc/ 토픽은 이 클래스가 예전처럼 msgpack 을 만들고 실제 전송
// (ZmqFollowTransport) 이 팀 [이름][protobuf] (6370) 로 바꿔 보낸다 (fc_protobuf.hpp).
// 2026-11-25 batch 6b: control/setpoint, follow/status 는 실제 전송이 팀 [이름][protobuf] (6372) 를
// 받아 예전 msgpack 으로 되살려 넘긴다 (control_protobuf.hpp). 이 클래스의 검사는 그대로다.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"
#include "argus/gateway/follow/include/cv_journal.hpp"
#include "argus/gateway/follow/include/cv_observation.hpp"
#include "argus/gateway/follow/include/scan_ramp.hpp"
#include "argus/gateway/ownership/include/operator_authority.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"
#include "argus/gateway/vehicle/include/follow_supervisor.hpp"

namespace argus::gateway {

using Parts = std::vector<Bytes>;

// 원본의 ZMQ 소켓 4개. receive는 막지 않는다 (poll(0)이 거짓이면 nullopt).
class IFollowTransport {
public:
    IFollowTransport() = default;
    IFollowTransport(const IFollowTransport&) = delete;
    IFollowTransport& operator=(const IFollowTransport&) = delete;
    IFollowTransport(IFollowTransport&&) = delete;
    IFollowTransport& operator=(IFollowTransport&&) = delete;
    virtual ~IFollowTransport() = default;

    virtual void publish(const Parts& parts) = 0;        // PUB
    virtual std::optional<Parts> rpc_receive() = 0;      // REP
    virtual void rpc_send(const Parts& parts) = 0;       // REP 응답
    virtual std::optional<Parts> command_receive() = 0;  // SUB control/setpoint, follow/status
    virtual bool has_tracks() const = 0;                 // 추적 SUB를 열었는지
    virtual std::optional<Parts> track_receive() = 0;    // SUB tracking/tracks
    virtual void close() = 0;
};

class FollowBOwner;

class ManualYaw final : public IManualYaw {
public:
    ManualYaw(FollowBOwner& owner, FlightLink& vehicle);

    bool active() const override { return !state_.is_null(); }
    Json status() const override;
    std::optional<double> position_rate() override;
    Json request(const Json& body) override;

    std::string fault() const;
    std::string ready_reason() const;
    bool valid_goal() const;
    // Offboard 제자리 회전 탐색 중인지 (2026-11-09)
    bool scanning() const;
    void ingest(const std::string& kind, const Json& data);
    void tick();
    void cancel(const std::string& reason) override;
    void finish(const std::string& reason);

private:
    Json scan_request(const Json& body);
    std::string direct_reason();
    void end_direct(const std::string& reason);
    void send(double rate);
    void mode(const std::string& name);
    bool can_stop() const;
    void stop(const std::string& reason);
    void retain_offboard_hold(const std::string& reason);
    double state_number(const char* key) const;
    bool sector_scan() const;
    double sector_rate(double now, double signed_progress, std::optional<ScanDrift> drift);

    FollowBOwner& owner_;
    FlightLink& v_;
    Json state_;                  // 원본 state dict (null이면 시험 없음)
    std::shared_ptr<Json> goal_;  // 원본 state['goal'] (같은 객체인지 비교)
    std::shared_ptr<Json> zero_goal_;
    Json last_{{"phase", "idle"}, {"reason", "not_started"}};
    std::vector<std::pair<std::string, std::string>> requests_;  // 원본 OrderedDict
    // 2026-11-18 탐색 방식 기본값. ARGUS_SCAN_PATTERN=turn|sector (없으면 turn = 기존 한 방향
    // 회전).
    ScanPattern default_scan_pattern_{ScanPattern::Turn};
    // 2026-11-18 왕복 중간 정지 여부. ARGUS_SCAN_SECTOR_CONTINUOUS=0 이면 구간 안 20° 정지를 다시
    // 쓴다.
    bool sector_continuous_{true};
    // 2026-11-18 왕복 시작 방향 (요청 속도가 없을 때). ARGUS_SCAN_SECTOR_FIRST=left|right, 기본
    // right.
    bool sector_first_left_{false};
    // 2026-11-18 왕복 기본 전체 폭(°). ARGUS_SCAN_SECTOR_WIDTH_DEG (0, 180], 기본 kScanSectorDeg.
    double sector_width_deg_{kScanSectorDeg};
};

struct FollowOwnerOptions {
    bool offboard_hover{false};
    bool cv_gap_enabled{false};
    std::optional<std::filesystem::path> journal_directory;  // 원본 ROOT/run/follow_b
    OperatorAuthority* authority{nullptr};                   // 원본 vehicle.operator_emergency
};

class FollowBOwner final : public IFollowSupervisor {
public:
    FollowBOwner(FlightLink& vehicle, std::unique_ptr<IFollowTransport> transport,
                 const FollowOwnerOptions& options);
    FollowBOwner(const FollowBOwner&) = delete;
    FollowBOwner& operator=(const FollowBOwner&) = delete;
    FollowBOwner(FollowBOwner&&) = delete;
    FollowBOwner& operator=(FollowBOwner&&) = delete;
    ~FollowBOwner() override;

    // ---- IFollowSupervisor (FlightLink가 부른다)
    void ingest(const std::string& kind, const Json& data) override;
    IManualYaw& manual_yaw() override { return manual_yaw_; }
    const IManualYaw& manual_yaw() const override { return manual_yaw_; }
    bool token_active() const override { return token_.has_value() && !token_->empty(); }
    bool bench_active() const override { return bench_.has_value(); }
    void clear_bench() override { bench_.reset(); }
    void invalidate(const std::string& reason) override;
    bool offboard_hover_enabled() const override { return offboard_hover_enabled_; }
    bool establish_hover(const std::string& reason, bool connection_lost) override;
    void tick() override;
    bool rc_override() const override { return rc_override_; }
    void set_rc_override(bool value) override { rc_override_ = value; }
    void prepare_handoff() override;
    bool legacy_mode_handoff() const override { return false; }

    // ---- 게이트웨이·메인 GUI 연결부가 쓰는 원본 속성·메서드
    Json bench_request(const Json& body);
    void close();
    const std::string& session() const { return session_; }
    const std::string& last_reason() const { return last_reason_; }
    bool pending_active() const { return pending_.has_value(); }
    ManualYaw& yaw() { return manual_yaw_; }
    // 원본 request / mode_request / receive_setpoint / receive_status / publish
    Wire request(const Wire& d);
    Wire mode_request(const Wire& wire);
    bool receive_setpoint(const Wire& data);
    bool receive_status(const Wire& data);
    void publish();
    // 비교 시험용 내부 상태
    Wire debug_state() const;
    // 2026-11-18 데모 위치 고정 상태 (demo_position_hold, anchor_x/y/z, anchor_epoch).
    // 비교 시험 상태(debug_state)와 fc/state 규격은 바꾸지 않으려고 따로 둔다.
    Wire demo_hold_status() const;

private:
    struct Context {
        std::int64_t epoch{0};
        std::int64_t generation{0};
        Wire track;
        bool operator==(const Context& other) const;
    };
    struct Pending {
        Wire request_id;
        std::shared_ptr<Wire> reply;
        Wire context;
        std::string mode;
        bool acked{false};
        std::int64_t sent_at{0};
        std::int64_t deadline{0};
    };
    struct Bench {
        std::int64_t until{0};
        double rate{0};
        std::int64_t last{0};
    };
    struct Handoff {
        std::shared_ptr<Json> goal;
        int epoch{0};
        std::int64_t until{0};
    };
    struct CacheEntry {
        std::string client;
        std::string request_id;
        Bytes fingerprint;
        std::shared_ptr<Wire> reply;
    };
    // 2026-11-18 데모 위치 고정 기준점 (LOCAL_NED, 추종 소유권마다 한 번 잡는다)
    struct DemoAnchor {
        double x{0};
        double y{0};
        double z{0};  // 현재 z 목표 (vz 적분, 데모 고도 범위로 자름)
        int epoch{0};
        std::optional<std::int64_t> last_send;
        bool height_unknown_logged{false};  // 높이를 몰라 z를 멈춘 것을 한 번만 남긴다
    };

    std::int64_t now_ns() const;
    void cv_trace(const std::string& event, const Wire* command, const std::string& reason = "",
                  const std::string& proof = "");
    void read_cv_observations();
    // 2026-11-09: fc/state·fc/attitude 공통 신원 필드와 fc/attitude 본문
    Wire common_wire() const;
    Wire attitude_wire(const Wire& common) const;
    // 2026-11-24: 표본 하나(data, 수신 시각)로 fc/attitude 본문을 만든다
    static Wire attitude_wire(const Wire& common, const Json* data, std::int64_t stamp);
    // 새 ATTITUDE를 받았으면 30ms 주기를 기다리지 않고 fc/attitude만 바로 보낸다
    void publish_new_attitude();
    // 2026-11-24: 마지막 송신 뒤 이력에 들어온 ATTITUDE를 수신 순서대로 모두 보낸다. 보낸 수.
    std::size_t publish_attitudes_since_last(const Wire& common);
    std::string fault() const;
    bool handoff_valid() const;
    bool hover_ready() const;
    bool bench_safe() const;
    std::int64_t validate(const Wire& d, bool velocity);
    void authority(const Wire& d) const;
    void motion_valid(const Wire& d, std::int64_t now, bool safety);
    void publish_message(const std::string& topic, Wire data);
    // 2026-11-16 perf: data를 소비하지 않고 보낸다 (보낸 뒤 cmc_pose_history 행을 재사용하려고)
    void publish_message_in_place(const std::string& topic, Wire& data);
    void mode_event(const Wire& context, const std::string& event, const std::string& reason = "",
                    const Wire& diagnostics = Wire());
    void safety_stop(const std::string& reason);
    CacheEntry* cached(const std::string& client, const Wire& request_id);
    void zero_body_setpoint(double yaw_rate);
    // 2026-11-18 데모 위치 고정: 이번 송신을 데모(참)로 보낼지 기존 속도(거짓)로 보낼지.
    // 인계 대기 중 프로필을 모르면 비어 있고 이번 송신을 미룬다. 소유권 안 변경은 ValueError.
    std::optional<bool> demo_send_decision();
    // 제어기 명령을 LOCAL_NED 위치 유지 + yaw rate로 보낸다 (거부는 ValueError)
    void send_demo_hold(const Wire& d, std::int64_t now);
    void reset_demo_anchor(const std::string& reason);

    FlightLink& v_;
    std::unique_ptr<IFollowTransport> transport_;
    OperatorAuthority* authority_;
    bool cv_gap_enabled_;
    bool offboard_hover_enabled_;
    bool local_hover_active_{false};
    std::optional<int> passive_hover_epoch_;
    std::string session_;
    std::optional<std::string> token_;
    std::optional<std::string> client_;
    std::int64_t revision_{0};
    std::int64_t seq_{0};
    std::optional<Context> command_context_;
    std::optional<std::pair<Context, std::int64_t>> last_real_command_source_;
    std::vector<std::pair<Context, std::int64_t>> real_command_sources_;
    CvObservationLedger cv_observations_;
    std::unique_ptr<CvJournal> cv_journal_;
    bool offboard_seen_{false};
    std::vector<CacheEntry> cache_;
    std::optional<Wire> setpoint_;
    std::optional<std::int64_t> prestream_at_;
    std::optional<std::int64_t> last_stream_;
    // 2026-11-16 이벤트 송신 (ARGUS_GW_SETPOINT_EVENT_MIN_MS, 0=꺼짐). 아직 안 보낸 새 명령 표시.
    std::int64_t setpoint_event_min_interval_ns_{0};
    bool setpoint_fresh_{false};
    std::optional<Pending> pending_;
    std::optional<Bench> bench_;
    ManualYaw manual_yaw_;
    std::int64_t last_publish_{0};
    std::int64_t last_attitude_stamp_{0};  // 마지막으로 보낸 ATTITUDE 수신 시각 (2026-11-09)
    // 2026-11-16 perf: cmc_pose_history 행 변환 결과 재사용. 행은 추가 뒤 바뀌지 않는다.
    // 열쇠 (종류, 수신 ns, FC time_boot_ms), FC epoch이 바뀌면 비운다.
    std::map<std::string, std::map<std::pair<std::int64_t, std::int64_t>, Wire>> history_cache_;
    int history_cache_epoch_{-1};
    std::int64_t pub_seq_{0};
    std::optional<Wire> last_mode_event_;
    std::int64_t last_event_publish_{0};
    std::optional<std::int64_t> last_follow_status_;
    std::optional<double> gui_loss_since_;  // 추종 중 GUI가 끊긴 시각 (2026-11-09)
    std::string last_hover_failure_;        // 마지막 제자리 유지 실패 사유 (2026-11-09)
    // 원본 Python int 그대로 (msgpack uint64까지 크기 비교가 정확해야 한다)
    Wire follow_status_seq_{-1};
    std::string last_reason_{"fc_not_connected"};
    bool rc_override_{false};
    std::optional<Handoff> handoff_;
    bool handoff_pending_{false};
    std::optional<DemoAnchor> demo_anchor_;
    // 이 소유권의 첫 송신에서 정한 방식 (참: 데모 위치 유지, 거짓: 기존 BODY_NED 속도)
    std::optional<bool> demo_mode_;
    int epoch_{0};
    bool closed_{false};
};

// FlightLink 상태 JSON을 순서 보존 메시지 값으로 (숫자 형식·NaN 유지)
Wire to_wire(const Json& value);
// Python type(value).__name__ (AttributeError 문구용)
std::string python_type_name(const Wire& value);

}  // namespace argus::gateway
