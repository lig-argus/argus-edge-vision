#pragma once

#include <atomic>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "argus/gateway/ownership/include/operator_authority.hpp"
#include "argus/gateway/vehicle/include/flight_link.hpp"
#include "argus/gateway/vehicle/include/rc_neutral.hpp"

namespace argus::gateway {
// The gateway adapter supplies the existing bounded operator RPC in the assembly stage.
struct ReclaimController {
    std::function<bool()> present;
    std::function<Json(const Json&, int timeout_ms)> request;
    // 원본 self.quit.wait(seconds): 기다린 뒤 종료 중이면 true. 비어 있으면 실제로 잠든다.
    // 비교 시험은 이 지점에서 Python 원본과 같은 순서로 복귀 절차를 한 칸씩 진행시킨다.
    std::function<bool(double seconds)> wait;
    // 복귀 절차 스레드가 끝날 때 (비교 시험용, 비어 있어도 된다)
    std::function<void()> finished;
};

struct StopCompletion {
    bool pending{false};
    bool accepted{false};
    std::string reason;
};
// 원본 main_track_adapter.completion (형식이 틀리면 ValueError)
StopCompletion stop_completion(const Json& status, const std::string& command_id,
                               const Json& controller_session, const Json& onboard_session,
                               const Json& fc_epoch);

class ReclaimFlightLink final : public FlightLink {
public:
    ReclaimFlightLink(FlightLinkOptions options, ReclaimController controller,
                      OperatorAuthority* authority = nullptr);
    ~ReclaimFlightLink() override;
    ReclaimFlightLink(const ReclaimFlightLink&) = delete;
    ReclaimFlightLink& operator=(const ReclaimFlightLink&) = delete;
    ReclaimFlightLink(ReclaimFlightLink&&) = delete;
    ReclaimFlightLink& operator=(ReclaimFlightLink&&) = delete;

    Json reclaim(const Json& client);
    Json submit(const Json& action, const Json& client, const Json& height) override;
    Json snapshot() override;
    Json yaw_handoff() override;
    void close_link() override;
    void clear_reclaim_context() override { context_.reset(); }
    // 원본 gateway /rc-gui-handoff의 vehicle._reclaim_reason(ignore_emergency=True)
    std::string handoff_reason() const { return reclaim_reason(false, true); }
    void finish(const std::string& status, std::string message) override;
    // 2026-11-10 실내 RC→GUI 복귀(사용자 확정). GPS 없는 PX4는 HOLD(AUTO_LOITER)를 거부하므로
    // 실내에서는 HOLD 없이 현재 위치 Offboard 제자리 유지를 먼저 잡고 추종기 STOP을 확인한다.
    // 기본 꺼짐(원본·시험 probe와 같은 HOLD 경로). 운영 gateway_main이 켠다.
    void set_indoor_offboard_reclaim(bool enabled) { indoor_offboard_reclaim_ = enabled; }

protected:
    void configure() override;
    void ingest(const InboundMessage& message) override;
    void dispatch(const Job& job) override;
    void supervise() override;

private:
    struct Workflow {
        std::string id;
        std::string client;
        int epoch;
        bool prior_rc;
        bool cancelled{false};
        bool offboard_first{false};  // 실내: HOLD 대신 Offboard 제자리 유지 먼저
    };
    struct Context {
        int epoch;
        std::optional<std::string> origin_mode;
        bool holding{false};
    };
    void observe_rc();
    void check_epoch();
    // 처음 일괄 요청에서 빠진 RC 보정값만 천천히 다시 읽는다(읽기 전용).
    std::vector<std::string> missing_rc_calibration() const;
    void retry_rc_calibration();
    void reset_rc_retry();
    bool rc_override() const;
    std::string reclaim_reason(bool allow_workflow = false, bool ignore_emergency = false) const;
    void workflow_guard(const Workflow& workflow);
    void reclaim_sequence(const std::shared_ptr<Workflow>& workflow);
    void confirm_controller_stop(const Workflow& workflow);
    void clear_reclaim_target();
    bool ending() const;
    bool pause(double seconds);

    ReclaimController controller_;
    OperatorAuthority* authority_;
    NeutralObservation neutral_;
    Json observation_{{"ready", false}, {"reason", "RC 중립 관측 대기"}};
    Json calibration_ = Json::object();
    std::map<std::string, double> rc_retry_at_;
    double next_rc_retry_{-std::numeric_limits<double>::infinity()};
    std::optional<int> rc_epoch_;
    std::optional<Context> context_;
    std::shared_ptr<Workflow> workflow_;
    bool indoor_offboard_reclaim_{false};
    std::atomic<bool> closing_{false};
    std::thread worker_;
};
}  // namespace argus::gateway
