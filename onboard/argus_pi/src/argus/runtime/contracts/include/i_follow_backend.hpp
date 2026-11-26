// B안 실행부는 이 경계만 사용한다. FC SDK/UART는 통신 소유자에만 둔다.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/common/time/include/time.hpp"
#include "argus/control/contracts/include/i_flight_mode_port.hpp"
#include "argus/vehicle/contracts/include/i_vehicle_port.hpp"
#include "argus/vehicle/contracts/include/i_vehicle_state_source.hpp"
#include "argus/vehicle/contracts/include/vehicle_flight_status.hpp"

namespace follow {

struct FollowCommandContext {
    std::uint64_t command_epoch{0};
    std::uint64_t generation{0};
    std::int64_t track_id{0};
    std::int64_t source_sample_monotonic_ns{0};
    bool bench{false};
    bool cv_gap_active{false};
    std::int64_t cv_origin_sample_ns{0}, cv_gap_deadline_monotonic_ns{0};
};

class IFollowBackend {
public:
    virtual ~IFollowBackend() = default;
    virtual IVehicleStateSource& pose() = 0;
    virtual IVehicleFlightStatusSource& flight_state() = 0;
    virtual IVehiclePort& vehicle() = 0;
    virtual IFlightModePort& mode() = 0;
    virtual bool identity_permit() const = 0;
    virtual std::string verified_uid() const = 0;
    virtual bool acquire() = 0;
    virtual bool owns() const = 0;
    virtual void release() = 0;
    // SDK 반복 송신을 포함한 최종 출구에서도 확인한다. 미설정 시 송신 불가.
    virtual void set_writer_guard(std::function<bool()> permit) = 0;
    virtual void update_context(const FollowCommandContext& context) = 0;
    // HOLD 내 Scan은 절대 heading 제어. wire rad 변환은 경계에서 한 번만 한다.
    virtual bool send_heading(double yaw_rad, Clock::time_point valid_until, bool stopping) = 0;
    // 진단 전용 상태. 이 발행이 제어 lease나 setpoint TTL을 연장하지 않는다.
    virtual void publish_status(const NativeJson&) {}
    // Best-effort diagnostics only; never influence command admission or delivery.
    virtual std::vector<NativeJson> take_hold_diagnostics() { return {}; }
    virtual bool remote_fc_owner() const { return false; }
    virtual std::string controller_session() const { return {}; }
    virtual NativeJson operator_context() const { return NativeJson::object(); }
    // B에서는 onboard 세션·FC epoch를 검사한다. status 조회는 명령 권한이 아니다.
    virtual bool validate_operator_request(const NativeJson&) const { return true; }
};

}  // namespace follow
