// FlightLink가 추종 송신부(Python follow_b_owner.FollowBOwner)에 기대는 동작.
// 4단계에서 C++ 추종 송신부가 구현한다. 없으면(nullptr) 원본의 follow_b=None과 같다.
#pragma once

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace argus::gateway {

// 원본 follow_b.manual_yaw
class IManualYaw {
public:
    IManualYaw() = default;
    IManualYaw(const IManualYaw&) = delete;
    IManualYaw& operator=(const IManualYaw&) = delete;
    IManualYaw(IManualYaw&&) = delete;
    IManualYaw& operator=(IManualYaw&&) = delete;
    virtual ~IManualYaw() = default;

    virtual bool active() const = 0;
    virtual nlohmann::json status() const = 0;
    // 위치 setpoint에 실을 yaw rate (없으면 기존 yaw 유지)
    virtual std::optional<double> position_rate() = 0;
    virtual nlohmann::json request(const nlohmann::json& body) = 0;
    // 원본 manual_yaw.cancel(reason) (메인 GUI 연결부가 송신을 내려놓을 때)
    virtual void cancel(const std::string& reason) = 0;
};

class IFollowSupervisor {
public:
    IFollowSupervisor() = default;
    IFollowSupervisor(const IFollowSupervisor&) = delete;
    IFollowSupervisor& operator=(const IFollowSupervisor&) = delete;
    IFollowSupervisor(IFollowSupervisor&&) = delete;
    IFollowSupervisor& operator=(IFollowSupervisor&&) = delete;
    virtual ~IFollowSupervisor() = default;

    virtual void ingest(const std::string& kind, const nlohmann::json& data) = 0;
    virtual IManualYaw& manual_yaw() = 0;
    virtual const IManualYaw& manual_yaw() const = 0;
    // 원본 bool(follow_b.token), bool(follow_b.bench)
    virtual bool token_active() const = 0;
    virtual bool bench_active() const = 0;
    virtual void clear_bench() = 0;
    virtual void invalidate(const std::string& reason) = 0;
    virtual bool offboard_hover_enabled() const = 0;
    virtual bool establish_hover(const std::string& reason, bool connection_lost) = 0;
    virtual void tick() = 0;
    virtual bool rc_override() const = 0;
    virtual void set_rc_override(bool value) = 0;
    virtual void prepare_handoff() = 0;
    virtual bool legacy_mode_handoff() const = 0;
};

}  // namespace argus::gateway
