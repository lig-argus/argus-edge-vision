// 2026-11-18 이륙 조건 보강: PX4 강한 자기장 간섭 경고 상태 (규칙은 헤더 설명).
#include "argus/gateway/vehicle/include/mag_interference.hpp"

#include <cstdlib>

namespace argus::gateway {

namespace {

// 16bit 순번 차이 (from에서 to까지 앞으로 센 값)
int forward_distance(int from, int to) { return (to - from) & 65535; }

bool newer(int sequence, int than) {
    const int delta = forward_distance(than, sequence);
    return delta > 0 && delta < 32768;
}

bool is_armed_by(std::uint32_t id, const std::string& name) {
    return id == MagInterferenceMonitor::kArmedById || name == MagInterferenceMonitor::kArmedByName;
}

bool is_disarmed_by(std::uint32_t id, const std::string& name) {
    return id == MagInterferenceMonitor::kDisarmedById ||
           name == MagInterferenceMonitor::kDisarmedByName;
}

}  // namespace

bool MagInterferenceMonitor::is_mag_interference(std::uint32_t id, const std::string& name) {
    return id == kMagInterferenceId || name == kMagInterferenceName;
}

bool MagInterferenceMonitor::is_arming_summary(std::uint32_t id, const std::string& name) {
    return id == kArmingSummaryId || name == kArmingSummaryName;
}

void MagInterferenceMonitor::reset(int epoch) {
    epoch_ = epoch;
    entries_.clear();
    summary_sequence_.reset();
    summary_fc_ms_ = 0;
    summary_at_ = 0.0;
    summary_given_up_ = false;
    active_.reset();
    reason_.clear();
    last_event_fc_ms_.reset();
    last_armed_.reset();
    disarmed_since_.reset();
    last_heartbeat_at_.reset();
    arm_event_sequence_.reset();
    armed_by_at_.reset();
    armed_with_warning_ = false;
}

const MagInterferenceMonitor::Entry* MagInterferenceMonitor::find(int sequence) const {
    for (const Entry& entry : entries_) {
        if (entry.sequence == sequence) {
            return &entry;
        }
    }
    return nullptr;
}

void MagInterferenceMonitor::observe_heartbeat(bool /*armed*/, double now) {
    // 무장 값은 update()의 armed(신선한 heartbeat 표본)로 받는다. 여기서는 도착 시각만 둔다.
    last_heartbeat_at_ = now;
}

bool MagInterferenceMonitor::newest_arm_event(int sequence) const {
    return !arm_event_sequence_ || newer(sequence, *arm_event_sequence_);
}

MagInterferenceMonitor::Change MagInterferenceMonitor::observe(
    int epoch, int sequence, std::uint32_t id, const std::string& name, std::int64_t fc_time_ms,
    std::optional<bool> armed, double now) {
    if (epoch != epoch_) {
        reset(epoch);
    }
    const bool mag = is_mag_interference(id, name);
    const bool summary = is_arming_summary(id, name);
    sequence &= 65535;
    // 같은 순번을 다시 받으면 내용을 바꿔 둔다 (재요청 응답)
    bool replaced = false;
    for (Entry& entry : entries_) {
        if (entry.sequence == sequence) {
            entry = Entry{sequence, fc_time_ms, mag, summary};
            replaced = true;
            break;
        }
    }
    if (!replaced) {
        entries_.push_back(Entry{sequence, fc_time_ms, mag, summary});
        while (entries_.size() > kMaxEntries) {
            entries_.pop_front();
        }
    }
    if (summary && (!summary_sequence_ || newer(sequence, *summary_sequence_))) {
        summary_sequence_ = sequence;
        summary_fc_ms_ = fc_time_ms;
        summary_at_ = now;
        summary_given_up_ = false;
    }
    if (is_armed_by(id, name) && newest_arm_event(sequence)) {
        // 무장 이벤트는 heartbeat보다 먼저 온다. 무장 heartbeat 전까지 무장으로 보고 고정한다
        // (heartbeat가 늦으면 ARM 직후 경고 없는 묶음을 무장 해제 보고로 잘못 볼 수 있다).
        arm_event_sequence_ = sequence;
        if (last_armed_ != std::optional<bool>(true)) {
            armed_with_warning_ = active_ == std::optional<bool>(true);
            armed_by_at_ = now;
        }
        return Change::kNone;
    }
    if (is_disarmed_by(id, name) && newest_arm_event(sequence)) {
        arm_event_sequence_ = sequence;
        armed_by_at_.reset();
        armed_with_warning_ = false;
        return Change::kNone;
    }
    if (!mag) {
        return Change::kNone;
    }
    // 가장 새 묶음(또는 요약이 늦게 오는 더 새 묶음)의 경고인가. 예전 묶음 이력은 무시한다.
    if (summary_sequence_ && !newer(sequence, *summary_sequence_)) {
        return Change::kNone;
    }
    last_event_fc_ms_ = fc_time_ms;
    // 무장 해제 중 경고는 바로 활성으로 둔다 (막는 쪽). 무장 중이면 update()의 규칙을 따른다.
    if (armed == std::optional<bool>(false) && !armed_by_at_ &&
        active_ != std::optional<bool>(true)) {
        active_ = true;
        reason_.clear();
        return Change::kSet;
    }
    return Change::kNone;
}

void MagInterferenceMonitor::track_armed(std::optional<bool> armed, double now) {
    if (!armed) {
        // heartbeat 만료: 무장 해제가 이어졌다고 보지 않는다. 마지막 무장 상태는 기억한다.
        disarmed_since_.reset();
        return;
    }
    if (*armed) {
        if (last_armed_ == std::optional<bool>(false) && !armed_by_at_) {
            // 무장 이벤트 없이 heartbeat로 처음 본 무장 (이벤트 유실 대비)
            armed_with_warning_ = active_ == std::optional<bool>(true);
        }
        armed_by_at_.reset();  // 무장 heartbeat 확인: 이후는 heartbeat 규칙
        disarmed_since_.reset();
        last_armed_ = true;
        return;
    }
    if (armed_by_at_) {
        if (now - *armed_by_at_ < kArmedByTimeoutS) {
            // 무장 이벤트 뒤 아직 무장 heartbeat 전: 늦은 무장 해제 heartbeat는 무장으로 본다
            disarmed_since_.reset();
            return;
        }
        armed_by_at_.reset();  // 지난 이력이거나 바로 해제된 무장
    }
    armed_with_warning_ = false;
    if (!disarmed_since_) {
        disarmed_since_ = now;
    }
    last_armed_ = false;
}

MagInterferenceMonitor::Burst MagInterferenceMonitor::newest_burst(
    std::optional<int> latest_sequence) const {
    const int first = *summary_sequence_;
    Burst burst;
    for (int offset = 1; offset < kMaxBurst; ++offset) {
        const int sequence = (first + offset) & 65535;
        const Entry* entry = find(sequence);
        if (entry != nullptr) {
            if (entry->summary ||
                std::llabs(entry->fc_time_ms - summary_fc_ms_) > kBurstToleranceMs) {
                burst.complete = true;  // 다음 요약이나 다른 시각의 이벤트: 묶음 끝
                return burst;
            }
            burst.has_mag = burst.has_mag || entry->mag;
            continue;
        }
        // 빠진 순번: FC가 아직 만들지 않은 순번이면 묶음 끝, 아니면 덜 받았다 (재요청 대기)
        if (latest_sequence && forward_distance(first, *latest_sequence) < offset) {
            burst.complete = true;
            return burst;
        }
        burst.missing = sequence;
        return burst;
    }
    burst.complete = true;
    return burst;
}

MagInterferenceMonitor::Change MagInterferenceMonitor::update(int epoch, std::optional<bool> armed,
                                                              std::optional<int> latest_sequence,
                                                              double now) {
    if (epoch != epoch_) {
        reset(epoch);
    }
    track_armed(armed, now);
    // 무장 해제가 이어지고, 가장 새 요약 묶음을 받은 지 충분히 지났고, 그 뒤에 도착한 heartbeat가
    // 무장 해제를 알릴 때만 판정한다. 무장 중 보고(경고 빠짐)는 무장 해제 뒤 새 보고가 없으면
    // FC의 현재 결과와 같다.
    if (!summary_sequence_ || summary_given_up_ || armed != std::optional<bool>(false) ||
        armed_by_at_ || !disarmed_since_ || now - *disarmed_since_ < kDisarmStableS ||
        now - summary_at_ < kSettleS || !last_heartbeat_at_ || *last_heartbeat_at_ <= summary_at_) {
        return Change::kNone;
    }
    const Burst burst = newest_burst(latest_sequence);
    if (!burst.complete) {
        const bool abandoned = pending_query_ && !pending_query_(burst.missing);
        if (!abandoned && now - summary_at_ < kIncompleteTimeoutS) {
            return Change::kNone;
        }
        // 빠진 구성원을 더 받을 수 없다: 막지도 풀지도 못하므로 모름 (fail-open, 화면에 표시)
        summary_given_up_ = true;
        active_.reset();
        reason_ = "incomplete_report";
        return Change::kUnknown;
    }
    if (active_ == std::optional<bool>(burst.has_mag)) {
        return Change::kNone;
    }
    const bool was_active = active_ == std::optional<bool>(true);
    active_ = burst.has_mag;
    reason_.clear();
    if (burst.has_mag) {
        last_event_fc_ms_ = summary_fc_ms_;
        return Change::kSet;
    }
    return was_active ? Change::kCleared : Change::kNone;
}

nlohmann::json MagInterferenceMonitor::status() const {
    return {
        {"active", active_ ? nlohmann::json(*active_) : nlohmann::json()},
        {"armed_with_warning", armed_with_warning_},
        {"reason", reason_.empty() ? nlohmann::json() : nlohmann::json(reason_)},
        {"last_event_fc_ms",
         last_event_fc_ms_ ? nlohmann::json(*last_event_fc_ms_) : nlohmann::json()},
        {"last_report_fc_ms",
         summary_sequence_ ? nlohmann::json(summary_fc_ms_) : nlohmann::json()},
        {"armed_event_pending", armed_by_at_.has_value()},
        {"epoch", epoch_},
    };
}

}  // namespace argus::gateway
