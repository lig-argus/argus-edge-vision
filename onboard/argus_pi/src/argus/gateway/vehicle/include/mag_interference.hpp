// 2026-11-18 이륙 조건 보강: PX4 강한 자기장 간섭 경고(arming_check) 상태.
// PX4는 사전 검사 결과가 바뀔 때만 요약 이벤트(commander_arming_check_summary) 뒤에 활성 검사마다
// 이벤트 하나를 거의 같은 FC 시각(ms 경계를 넘을 수 있음), 이어진 순번으로 보낸다. 무장 중에는
// 자기장 검사를 돌리지 않으므로 무장 중 보고에는 경고가 빠진다. 이 클래스는 가장 새 보고 묶음에
// 경고가 있는지로 상태를 정한다. FC 링크 잠금 안에서 FlightLink가 부른다 (스스로 잠그지 않는다).
//
// 알려진 한계 (문서화만 함):
// - 게이트웨이 재시작, 재연결 뒤 이력(최근 32개)에 요약 묶음이 없으면 상태는 모름이고 막지 않는다
//   (fail-open). PX4는 결과가 바뀔 때만 보고하므로 다음 변화 전까지 모름이 이어질 수 있다.
// - 무장 해제 뒤 kDisarmStableS 동안은 이전 상태를 유지하므로 해제 직후 경고 해제가 최대 약 3초
//   늦다 (막는 쪽으로만 늦다).
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>

namespace argus::gateway {

class MagInterferenceMonitor {
public:
    // 원시 ID는 이름의 해시라서 펌웨어 빌드가 달라도 같다. 메타데이터 해석 전에도 쓴다.
    static constexpr std::uint32_t kMagInterferenceId = 0x01efc45fU;
    static constexpr std::uint32_t kArmingSummaryId = 0x01a893e0U;
    static constexpr std::uint32_t kArmedById = 0x0190caf3U;
    static constexpr std::uint32_t kDisarmedById = 0x01f46777U;
    static constexpr const char* kMagInterferenceName = "check_estimator_mag_interference";
    static constexpr const char* kArmingSummaryName = "commander_arming_check_summary";
    static constexpr const char* kArmedByName = "commander_armed_by";
    static constexpr const char* kDisarmedByName = "commander_disarmed_by";
    // 보고 묶음을 받은 뒤 판정까지 기다리는 시간 s (늦게 오는 묶음 구성원, ARM heartbeat 지연)
    static constexpr double kSettleS = 2.0;
    // 무장 해제가 이만큼 이어진 뒤에만 보고로 상태를 바꾼다 s (해제 직후 새 보고 대기)
    static constexpr double kDisarmStableS = 3.0;
    // 묶음 구성원으로 보는 요약과의 FC 시각 차 ms (sess275 묶음은 0.22 ms 걸쳐 ms 경계를 넘었다)
    static constexpr std::int64_t kBurstToleranceMs = 50;
    // 무장 이벤트(commander_armed_by) 뒤 무장 heartbeat를 기다리는 최대 시간 s.
    // 그동안 무장 해제 heartbeat만 오면 지난 이력이거나 바로 해제된 무장으로 본다.
    static constexpr double kArmedByTimeoutS = 5.0;
    // 빠진 구성원을 이만큼 기다려도 오지 않으면 상태를 모름으로 둔다 s
    static constexpr double kIncompleteTimeoutS = 10.0;
    // 한 묶음으로 볼 최대 순번 폭과 기억할 이벤트 수
    static constexpr int kMaxBurst = 64;
    static constexpr std::size_t kMaxEntries = 256;

    // kUnknown: 보고 묶음이 끝내 불완전해서 상태를 모름으로 바꿨다 (묶음마다 한 번)
    enum class Change { kNone, kSet, kCleared, kUnknown };
    // 빠진 순번을 FC에 다시 요청 중인가 (EventLog). 없으면 kIncompleteTimeoutS만 쓴다.
    using PendingQuery = std::function<bool(int sequence)>;

    // FC 이벤트 기록 구간(epoch)을 새로 시작한다. 상태는 모름, 무장 고정 표시는 해제.
    void reset(int epoch);
    void set_pending_query(PendingQuery query) { pending_query_ = std::move(query); }
    // FC HEARTBEAT 수신 (도착 시각). 묶음 판정은 그 묶음보다 늦게 도착한 heartbeat가 필요하다.
    void observe_heartbeat(bool armed, double now);
    // EventLog가 새로 받아들인 EVENT 하나. name은 해석된 이벤트 이름(모르면 빈 값).
    // armed는 지금 FC 무장 상태(모르면 비어 있음). 무장 해제 중 경고면 바로 활성으로 바꾼다.
    Change observe(int epoch, int sequence, std::uint32_t id, const std::string& name,
                   std::int64_t fc_time_ms, std::optional<bool> armed, double now);
    // 주기 판정. latest_sequence는 FC가 알린 가장 새 이벤트 순번(모르면 비어 있음).
    Change update(int epoch, std::optional<bool> armed, std::optional<int> latest_sequence,
                  double now);

    // 참: 경고 활성, 거짓: 경고 없음 확인, 비어 있음: 모름 (막지 않는다)
    std::optional<bool> active() const { return active_; }
    // 무장한 순간 경고가 활성이었는가 (무장 해제 때 풀린다)
    bool armed_with_warning() const { return armed_with_warning_; }
    nlohmann::json status() const;

    static bool is_mag_interference(std::uint32_t id, const std::string& name);
    static bool is_arming_summary(std::uint32_t id, const std::string& name);

private:
    struct Entry {
        int sequence{0};
        std::int64_t fc_time_ms{0};
        bool mag{false};
        bool summary{false};
    };
    struct Burst {
        bool complete{false};
        bool has_mag{false};
        int missing{0};  // 덜 왔을 때 처음 빠진 순번
    };

    const Entry* find(int sequence) const;
    void track_armed(std::optional<bool> armed, double now);
    // 가장 새 요약 묶음: 다음 요약 또는 FC 시각 차가 kBurstToleranceMs를 넘는 이벤트까지
    Burst newest_burst(std::optional<int> latest_sequence) const;
    // 무장 이벤트, 해제 이벤트 중 가장 새 것인가
    bool newest_arm_event(int sequence) const;

    PendingQuery pending_query_;
    int epoch_{0};
    std::deque<Entry> entries_;
    std::optional<int> summary_sequence_;
    std::int64_t summary_fc_ms_{0};
    double summary_at_{0.0};
    bool summary_given_up_{false};
    std::optional<bool> active_;
    std::string reason_;  // 모름의 까닭 (빈 값이면 없음)
    std::optional<std::int64_t> last_event_fc_ms_;
    std::optional<bool> last_armed_;
    std::optional<double> disarmed_since_;
    std::optional<double> last_heartbeat_at_;
    std::optional<int> arm_event_sequence_;
    std::optional<double> armed_by_at_;  // 무장 이벤트 뒤 무장 heartbeat 대기 중이면 도착 시각
    bool armed_with_warning_{false};
};

}  // namespace argus::gateway
