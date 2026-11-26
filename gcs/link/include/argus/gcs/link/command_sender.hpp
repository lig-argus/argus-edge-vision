// 명령 송신 절차 (icd 4.1)
// - 응답(COMMAND_ACK)이 1 s 안에 없으면 같은 명령을 다시 보내고, 처음을 포함해 3 번 보낸 뒤에도
// 없으면
//   응답 없음
// - IN_PROGRESS 를 받으면 재전송을 멈추고 최종 응답을 기다린다. 마지막 IN_PROGRESS 뒤 3 s 안에 다음
//   응답이 없으면 응답 없음
// - 응답은 target_system, target_component 가 이 통제기이고, 보낸 쪽 compid 와 command 가 기다리는
//   명령과 같을 때만 처리한다
// - 같은 대상(compid)의 같은 MAV_CMD 는 응답을 기다리는 동안 하나만 둔다. 다른 MAV_CMD 는 함께 둘
// 수
//   있다 (진행 중인 TRACK_START 를 TRACK_STOP 으로 취소하는 경우)
// - 명령 번호(icd 4.1)는 takeNumber() 로 받아 param7 에 넣는다. 재전송은 같은 값을 그대로 보낸다
// 송신은 IMavlinkConnection 으로 한다. sysid, compid, seq 는 연결이 채운다.
#ifndef ARGUS_GCS_LINK_COMMAND_SENDER_HPP_
#define ARGUS_GCS_LINK_COMMAND_SENDER_HPP_

#include <QString>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <utility>

#include "argus/gcs/link/command.hpp"
#include "argus/gcs/link/command_traits.hpp"
#include "argus/gcs/link/log_entry.hpp"
#include "argus/gcs/mavlink_connection/mavlink_connection.hpp"
#include "argus/mavlink.h"

namespace argus::gcs::link {

// 명령 하나의 진행. 화면의 명령 진행 표시가 읽는다
struct CommandTrace {
    enum class Stage : std::uint8_t {
        None,        // 보낸 적 없음
        Waiting,     // 보내고 응답 대기 (재전송 포함)
        InProgress,  // IN_PROGRESS 를 받고 최종 응답 대기
        Accepted,
        Failed,  // 거부, 실패, 취소, 응답 없음
    };

    Stage stage{Stage::None};
    std::int32_t sends{0};         // 보낸 횟수
    std::int32_t progressPct{-1};  // IN_PROGRESS 의 진행률 0~100. 모르면 -1
    std::int64_t sentNs{0};        // 처음 보낸 시각
    std::int64_t stageNs{0};       // 지금 단계에 들어간 시각
    QString result;                // Failed 의 결과 문구
};

// 보낼 명령 하나 (COMMAND_LONG 의 값)
struct OutgoingCommand {
    Command command{};
    QString name;  // 기록에 쓰는 명령 이름
    CommandTarget target{CommandTarget::Fc};
    std::uint16_t mavCmd{0};
    std::array<float, 7> params{};
};

class CommandSender {
public:
    using LogHandler = std::function<void(const LogEntry&)>;
    // 최종 응답을 받았을 때 (응답 없음은 부르지 않는다)
    using FinishHandler = std::function<void(Command, MAV_RESULT)>;

    // firstNumber: 명령 번호의 시작값. 통제기 기동 시각(ms)을 2^24 로 나눈 나머지 (icd 4.1)
    CommandSender(mavlink_connection::IMavlinkConnection& connection, std::uint32_t firstNumber,
                  LogHandler log, FinishHandler finished);

    // 이 통제기의 sysid. 응답의 target_system 과 비교한다
    void setGcsSysid(std::uint8_t sysid) { gcsSysid_ = sysid; }

    // 명령 번호 하나를 받는다. 받을 때마다 1 증가하고 2^24 에서 0 으로 돌아간다
    std::uint32_t takeNumber();

    // 보낸다. 같은 대상, 같은 MAV_CMD 의 명령이 응답 대기 중이면 그 명령을 포기하고 새로 보낸다
    void send(const OutgoingCommand& command, std::int64_t nowNs);

    // 재전송과 시간 초과 확인 (주기 작업)
    void tick(std::int64_t nowNs);

    // COMMAND_ACK 를 처리한다. 기다리던 명령의 응답이면 true
    bool onCommandAck(const mavlink_message_t& message, std::int64_t nowNs);

    bool awaiting(CommandTarget target, std::uint16_t mavCmd) const;
    bool awaitingCommand(Command command) const;
    bool awaitingAny(CommandTarget target) const;
    CommandTrace trace(Command command) const;

private:
    struct Pending {
        OutgoingCommand command;
        std::int32_t sends{0};
        std::int64_t lastEventNs{0};  // 마지막 송신 또는 마지막 IN_PROGRESS 시각
        bool inProgress{false};
    };
    using Key = std::pair<CommandTarget, std::uint16_t>;

    void transmit(Pending& pending, std::int64_t nowNs);
    void finishFailed(const Pending& pending, const QString& result, std::int64_t nowNs);
    void setStage(Command command, CommandTrace::Stage stage, std::int64_t nowNs,
                  const QString& result = {});

    mavlink_connection::IMavlinkConnection& connection_;
    LogHandler log_;
    FinishHandler finished_;
    std::uint8_t gcsSysid_{0};
    std::uint32_t nextNumber_;
    std::map<Key, Pending> pending_;
    std::map<Command, CommandTrace> traces_;
};

// MAV_RESULT 의 화면 이름
QString resultName(std::uint8_t result);

}  // namespace argus::gcs::link

#endif  // ARGUS_GCS_LINK_COMMAND_SENDER_HPP_
