// FC 이벤트 기록 (Pi tools/real_fc/events.py의 EventLog와 같은 규칙).
// 같은 펌웨어의 정의로만 해석하고, 빠진 순번은 한 번에 최대 32개까지 다시 요청한다.
#pragma once

#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "argus/gateway/events/include/event_parser.hpp"

namespace argus::gateway {

class EventLog {
public:
    using Clock = std::function<double()>;

    static constexpr std::size_t kMaxRecords = 160;
    static constexpr std::size_t kMaxSeen = 256;
    static constexpr std::size_t kMaxPending = 32;

    // metadata_path: fc-events.json (펌웨어별 이벤트 정의 묶음)
    EventLog(Clock clock, const std::filesystem::path& metadata_path);

    // AUTOPILOT_VERSION과 COMPONENT_INFORMATION이 메타데이터와 같은 FC·펌웨어인지 확인한다.
    void verify(const nlohmann::json& version, const nlohmann::json& info);
    void reset(const std::string& reason);
    // SYSTEM_TIME 부팅 시각. 재부팅이면 true (32bit wrap과 구분).
    bool observe_boot(std::int64_t milliseconds);
    // CURRENT_EVENT_SEQUENCE / RESPONSE_EVENT_ERROR / EVENT (pymavlink to_dict 형식)
    void ingest(const std::string& kind, const nlohmann::json& data);
    // 지금 다시 요청할 연속 순번 구간 (first, last). 없으면 nullopt.
    std::optional<std::pair<int, int>> due();
    nlohmann::json snapshot() const;
    // 원본 _decode (검증 전이면 미해석 표시)
    nlohmann::json decode(std::uint32_t event_id, const Bytes& arguments) const;
    // 2026-11-18: 같은 펌웨어로 확인된 정의의 이벤트 이름 (검증 전이거나 모르면 빈 값).
    // 기록(records)과 snapshot 형식은 바꾸지 않는다.
    std::string event_name(std::uint32_t event_id) const;
    // 2026-11-18: 이 순번을 FC에 다시 요청 중인가 (만료, unavailable이면 거짓)
    bool is_pending(int sequence) const;
    // 비교 시험용 내부 상태 (latest, pending 순서·시각, boot_ms 등)
    nlohmann::json debug_state() const;

    bool verified() const { return verified_; }
    int epoch() const { return epoch_; }
    const std::string& status() const { return status_; }
    // 원본 fc_events.latest (아직 순번을 모르면 없음)
    std::optional<int> latest() const { return latest_; }
    // 원본 fc_events.records[-1] (없으면 nullptr)
    const nlohmann::json* last_record() const {
        return records_.empty() ? nullptr : &records_.back();
    }
    // 원본이 FC 재부팅 때 직접 바꾸는 verified·status
    void set_verification(bool verified, std::string status) {
        verified_ = verified;
        status_ = std::move(status);
    }

private:
    struct Pending {
        int sequence{0};
        double due{0.0};
        int attempts{0};
        bool history{false};
    };

    void request(int first, int last, bool history);
    bool seen(int sequence) const;
    Pending* find_pending(int sequence);

    Clock clock_;
    std::deque<nlohmann::json> records_;
    std::deque<std::pair<int, int>> seen_;
    std::vector<Pending> pending_;  // 원본 dict 삽입 순서
    std::optional<int> latest_;
    int epoch_{0};
    std::optional<std::int64_t> boot_ms_;
    bool reset_flag_{false};
    int unavailable_{0};
    int expired_{0};
    bool verified_{false};
    std::string status_{"FC 메타데이터 확인 대기"};
    std::unique_ptr<EventParser> parser_;
    nlohmann::json metadata_ = nlohmann::json::object();
};

}  // namespace argus::gateway
