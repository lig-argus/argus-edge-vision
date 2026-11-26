#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/runtime/contracts/include/i_follow_backend.hpp"

namespace follow {

struct ZeroMqFollowBackendConfig {
    std::string telemetry_endpoint{"tcp://127.0.0.1:6370"};
    std::string rpc_endpoint{"tcp://127.0.0.1:6371"};
    std::string control_endpoint{"tcp://127.0.0.1:6372"};
    std::string expected_uid;
    int rpc_timeout_ms{100};
    int telemetry_ttl_ms{200};
    int command_ttl_ms{250};
    int source_max_age_ms{250};
    bool read_only{false};
    // PR26 모드 요청 (schema argus.control.mode_request.v1) + 완료 이벤트. 구 RPC는 명시 선택만.
    // 2026-11-25 batch 7: 두 방식 모두 6371 의 팀 [control/mode_request][protobuf] 로 보낸다
    bool mode_events{true};
    int mode_result_timeout_ms{2000};
};

// Local reference protocol: all sockets belong to one actor, never to callers.
// No MAVSDK/UART connection and no automatic reacquisition after revocation.
class ZeroMqFollowBackend final : public IFollowBackend {
public:
    explicit ZeroMqFollowBackend(ZeroMqFollowBackendConfig config);
    ~ZeroMqFollowBackend() override;
    IVehicleStateSource& pose() override;
    IVehicleFlightStatusSource& flight_state() override;
    IVehiclePort& vehicle() override;
    IFlightModePort& mode() override;
    bool identity_permit() const override;
    std::string verified_uid() const override;
    bool acquire() override;
    bool owns() const override;
    void release() override;
    void set_writer_guard(std::function<bool()> permit) override;
    void update_context(const FollowCommandContext& context) override;
    bool send_heading(double yaw_rad, Clock::time_point valid_until, bool stopping) override;
    NativeJson observations() const;
    // 2026-11-25 batch 6c: 6370 에서 받고 버린 FC 메시지 수 (해석 실패, 본문 상한 초과, 본문 뒤
    // 프레임). ZMQ_MAXMSGSIZE 를 두지 않아 연결은 그대로이고 이 수만 오른다
    std::uint64_t telemetry_decode_failures() const;
    bool remote_fc_owner() const override { return true; }
    bool validate_operator_request(const NativeJson& request) const override;
    void publish_status(const NativeJson& status) override;
    std::vector<NativeJson> take_hold_diagnostics() override;
    std::string controller_session() const override;
    NativeJson operator_context() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace follow
