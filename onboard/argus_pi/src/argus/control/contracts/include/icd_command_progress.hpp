// MAVLink 생성/ACK 송신은 통신 담당자 소유. 내부 작업의 진행·최종 결과만 제공한다.
#pragma once

#include <deque>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/control/contracts/include/command_outcome.hpp"

namespace follow {
class IcdCommandProgress final {
public:
    void begin(const NativeJson& request, bool pending) {
        for (auto& row : rows_) {
            if (row.at("pending") == true) {
                finish(row, false, "superseded_by_explicit_command");
            }
        }
        if (rows_.size() >= 32) {
            rows_.pop_front();
        }
        NativeJson row{{"command_id", request.at("command_id")},
                       {"action", request.at("action")},
                       {"pending", pending},
                       {"accepted", true},
                       {"result", pending ? "pending" : "accepted"},
                       {"reason", pending ? "awaiting_execution_confirmation" : "already_idle"}};
        for (const auto* key : {"generation", "class_id", "track_id", "sensor"}) {
            if (request.contains(key)) {
                row[key] = request.at(key);
            }
        }
        row["request_accepted"] = true;
        add_command_outcome(row);
        rows_.push_back(std::move(row));
    }

    // Rejection records must not supersede an admitted command still executing.
    void reject(const NativeJson& request, const std::string& reason) {
        if (rows_.size() >= 32) {
            auto completed = rows_.begin();
            while (completed != rows_.end() && completed->at("pending") == true) {
                ++completed;
            }
            if (completed == rows_.end()) {
                return;
            }
            rows_.erase(completed);
        }
        NativeJson row{{"command_id", request.at("command_id")},
                       {"action", request.at("action")},
                       {"pending", false},
                       {"accepted", false},
                       {"result", "failed"},
                       {"request_accepted", false},
                       {"reason", reason}};
        add_command_outcome(row);
        rows_.push_back(std::move(row));
    }

    // 2026-11-17: 받아들여 진행 중인 명령을 실행하지 못하고 끝낼 때 (예: 클릭한 대상이 기다리는
    // 동안 다시 잡히지 않음). 같은 command_id의 진행 중 행만 실패로 닫는다.
    void fail_pending(const NativeJson& request, const std::string& reason) {
        if (!request.contains("command_id")) {
            return;
        }
        for (auto& row : rows_) {
            if (row.at("pending") == true && row.at("command_id") == request.at("command_id")) {
                finish(row, false, reason);
            }
        }
    }

    void update(const std::string& mission, bool actual_offboard, bool actual_hold, bool inactive,
                bool scan_active, bool heading_pending, const std::string& scan_state,
                const std::string& scan_reason, const NativeJson& selected,
                bool offboard_hover = false) {
        for (auto& row : rows_) {
            if (row.at("pending") != true) {
                continue;
            }
            const auto action = row.at("action").get<std::string>();
            if (mission == "FAULT" || mission == "MANUAL") {
                finish(row, false, "execution_interrupted_" + mission);
            } else if (action == "track_start" && mission == "FOLLOW_ACTIVE" && actual_offboard) {
                const bool same =
                    selected.is_object() &&
                    selected.value("generation", NativeJson{}) ==
                        row.value("generation", NativeJson{}) &&
                    selected.value("track_id", NativeJson{}) == row.value("track_id", NativeJson{});
                finish(row, same, same ? "actual_offboard_confirmed" : "target_context_changed");
            } else if (action == "track_start" &&
                       (mission == "HOLD_TRANSITION" || mission == "HOLD_WAIT")) {
                finish(row, false, "start_interrupted_before_offboard_confirmation");
            } else if (action == "stop" && inactive && !scan_active && !heading_pending &&
                       (actual_hold || (actual_offboard && offboard_hover))) {
                finish(row, true,
                       offboard_hover ? "offboard_hover_and_stop_confirmed"
                                      : "actual_hold_and_stop_confirmed");
            } else if (action == "search") {
                if (scan_state == "Fault") {
                    finish(row, false, scan_reason);
                } else if (!scan_active && !heading_pending && actual_hold &&
                           (scan_state == "Completed" || scan_state == "TimedOut" ||
                            scan_state == "Candidate" || scan_state == "Stopped")) {
                    finish(row, true, scan_reason);
                }
            }
        }
    }

    NativeJson snapshot() const {
        auto result = NativeJson::array();
        for (const auto& row : rows_) {
            result.push_back(row);
        }
        return result;
    }

private:
    static void finish(NativeJson& row, bool success, const std::string& reason) {
        row["pending"] = false;
        row["accepted"] = success;
        row["result"] = success ? "accepted" : "failed";
        row["reason"] = reason;
        add_command_outcome(row);
    }
    std::deque<NativeJson> rows_;
};
}  // namespace follow
