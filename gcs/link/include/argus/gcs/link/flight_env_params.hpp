// 비행 환경(실내, 실외) 파라미터의 읽기와 쓰기 (icd 5.2).
// - 읽기: FC 가 연결되면 두 묶음의 이름을 check 마다 하나씩 PARAM_REQUEST_READ 로 묻고, 돌아온
//   PARAM_VALUE 를 묶음과 비교해 지금 환경을 정한다. 응답이 빠진 이름은 다시 묻는다
// - 쓰기: 값마다 PARAM_SET 을 보내고 같은 값의 PARAM_VALUE 로 확인한 뒤 다음 값을 보낸다. 확인이
//   없으면 다시 보낸다 (icd 4.1 의 간격과 횟수)
// 소켓을 갖지 않는다. 보낼 메시지는 Send 로 내보내고, 받은 메시지는 onMessage 로 받는다. 시각은
// 부르는 쪽이 monotonic ns 로 넘긴다
#ifndef ARGUS_GCS_LINK_FLIGHT_ENV_PARAMS_HPP_
#define ARGUS_GCS_LINK_FLIGHT_ENV_PARAMS_HPP_

#include <QString>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "argus/gcs/link/link_protocol.hpp"
#include "argus/gcs/link/log_entry.hpp"
#include "argus/gcs/link/param_profile.hpp"
#include "argus/gcs/link/vehicle_state.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

class FlightEnvParams {
public:
    using Send = std::function<void(const mavlink_message_t&)>;
    using Log = std::function<void(const LogEntry&)>;

    // 쓰기: PARAM_VALUE 가 이 시간 안에 같은 값으로 오지 않으면 다시 보낸다 (최대 kMaxSends 번)
    static constexpr std::int64_t kWriteRetryNs = kRetryIntervalNs;

    // 읽기: 응답이 빠진 이름은 이 간격으로 다시 묻는다 (처음을 포함해 최대 kMaxSends 번).
    // 이름 수만큼 check 를 거쳐야 한 번의 읽기가 끝나므로 쓰기보다 길게 둔다
    static constexpr std::int64_t kReadRetryNs = 3 * kNsPerSecond;

    FlightEnvParams(Send send, Log log);

    // 이 통제기의 sysid (보내는 PARAM_* 의 sysid)
    void setSysid(std::uint8_t sysid) { sysid_ = sysid; }

    void setProfiles(std::vector<ParamProfile> profiles);
    bool hasProfile(FlightEnv env) const;  // 값이 하나 이상 있는 그 환경의 묶음이 있다

    // 두 묶음의 이름을 모두 읽기 시작한다 (FC 연결, 재연결 때)
    void queueReads(std::int64_t nowNs);

    // 주기 처리: 읽기 하나 보내기, 빠진 이름 다시 묻기, 쓰기의 재전송과 포기
    void check(std::int64_t nowNs, LinkState fcLink);

    // 받은 메시지. FC(sysid 1, compid 1)의 PARAM_VALUE 만 쓰고 나머지는 무시한다
    void onMessage(const mavlink_message_t& message, std::int64_t nowNs);

    // 쓸 수 없는 이유. 쓸 수 있으면 nullopt. 지상이고 시동이 꺼져 있고 추종, 스캔 중이 아닐 때만
    // 쓴다 (icd 5.2 의 PARAM_SET 전달 조건. mavlink_gateway 가 전달하지 않을 PARAM_SET 은 보내지
    // 않는다)
    std::optional<QString> writeRefusal(FlightEnv env, const VehicleState& state) const;

    // 그 환경의 묶음을 쓰기 시작한다. 쓸 수 없으면 쓰지 않고 이유를 기록한 뒤 돌려준다
    std::optional<QString> apply(FlightEnv env, const VehicleState& state, std::int64_t nowNs);

    // FC 가 재부팅을 받아들였다: 그 전 값을 버리고 다시 묻는다
    void onRebooting(std::int64_t nowNs);

    // FC 값이 그 묶음과 모두 같으면 그 환경
    std::optional<FlightEnv> current() const;

    // 화면 표시: "실내", "실외", "읽기 중", "섞임: …", "미확인: …", FC 링크가 없으면 "-"
    QString text(LinkState fcLink, std::int64_t nowNs) const;

    std::optional<FlightEnv> writing() const;  // 쓰는 중인 환경

    // 바꿨고 아직 FC 를 재부팅하지 않았다 (EKF2_HGT_REF 등은 재부팅해야 반영된다)
    bool rebootNeeded() const { return rebootNeeded_; }

private:
    struct Write {
        FlightEnv env{FlightEnv::Indoor};
        std::vector<ParamValue> values;
        std::size_t index{0};  // 확인을 기다리는 값
        std::int32_t sends{0};
        std::int64_t sentNs{0};
    };

    std::vector<QString> names() const;  // 두 묶음의 이름 (중복 없이, 묶음 순서)
    std::vector<QString> missingNames() const;
    bool matches(const ParamProfile& profile) const;
    void sendNextRead();
    void requeueMissing(std::int64_t nowNs);
    void checkWrite(std::int64_t nowNs);
    void sendSet(const ParamValue& value, std::int64_t nowNs);
    void confirmWrite(const ParamValue& received, std::int64_t nowNs);
    QString mixedText() const;

    Send send_;
    Log log_;
    std::uint8_t sysid_{kGcsSysidDefault};
    std::vector<ParamProfile> profiles_;
    std::map<QString, ParamValue> fcValues_;  // FC 가 PARAM_VALUE 로 알려 준 값
    std::deque<QString> reads_;               // 보낼 PARAM_REQUEST_READ (check 마다 하나)
    std::int32_t readRounds_{0};              // FC 연결 뒤 읽기를 시작한 횟수
    std::int64_t lastReadNs_{0};
    std::optional<Write> write_;
    bool rebootNeeded_{false};
};

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_FLIGHT_ENV_PARAMS_HPP_
