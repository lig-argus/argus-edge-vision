// 통제기가 보낸 MAVLink 명령(COMMAND_LONG) 하나를 처리하는 인터페이스
// 규약: docs/spec/icd.md 4.1 (COMMAND_ACK, 재전송), 4.2 (TRACK_START, TRACK_STOP, SCAN_START)
// 구현: TrackStartHandler, TrackStopHandler, ScanStartHandler (다음 PR)
// 사용: MavlinkCommandReceiver 가 MAV_CMD 번호로 handler 를 찾아 부른다
#ifndef ARGUS_ONBOARD_MAVLINK_COMMAND_HANDLER_HPP_
#define ARGUS_ONBOARD_MAVLINK_COMMAND_HANDLER_HPP_

#include <cstdint>
#include <exception>
#include <functional>
#include <string>
#include <utility>

#include "argus/mavlink.h"

namespace argus::onboard {

// 받은 명령과 보낸 쪽
struct CommandRequest {
    std::uint8_t source_system = 0;     // 보낸 쪽 sysid. 제어권 판정(icd 4.2 조건 ①)에 쓴다
    std::uint8_t source_component = 0;  // 보낸 쪽 compid
    mavlink_command_long_t command{};
};

// COMMAND_ACK.progress 의 "진행률 모름" 값 (MAVLink: UINT8_MAX)
inline constexpr std::uint8_t kProgressUnknown = UINT8_MAX;

// COMMAND_ACK 로 보낼 결과
struct CommandAck {
    MAV_RESULT result = MAV_RESULT_FAILED;
    // result 가 IN_PROGRESS 일 때 진행 백분율(0~100). 모르면 kProgressUnknown. 그 외 result 에서는
    // 쓰지 않는다
    std::uint8_t progress = kProgressUnknown;
    // DENIED, FAILED 의 원인. MavlinkCommandReceiver 가 STATUSTEXT 로 보내 통제기 화면에 표시한다.
    // 비어 있으면 STATUSTEXT 를 보내지 않는다
    std::string reason;
};

// 명령 하나의 응답 통로. handler 가 받아 두고 결과가 정해질 때마다 send() 한다.
// - IN_PROGRESS 는 여러 번 보낼 수 있다 (진행률이 바뀔 때).
// - IN_PROGRESS 가 아닌 결과는 최종이며 한 번만 보낸다. 그 뒤의 send() 는 무시한다.
// - 최종 결과를 보내지 않고 사라지면 소멸자가 FAILED 를 보낸다. 통제기가 끝없이 기다리지 않게 한다.
// - 이동만 가능하다. 복사가 안 되므로 두 곳에서 보낼 수 없다. 추종 제어의 응답 콜백과 follow/status
//   콜백처럼 std::function 여러 개가 함께 들어야 하면 shared_ptr 로 싼다 (std::function 은 복사
//   가능한 함수 객체만 담는다). 마지막 shared_ptr 가 사라질 때 소멸자가 돈다.
class CommandResponder {
public:
    // 결과를 실제로 보내는 함수. MavlinkCommandReceiver 가 만든다 (COMMAND_ACK, STATUSTEXT 송신)
    using Send = std::function<void(const CommandAck&)>;

    // 아무 명령도 나타내지 않는 빈 responder. send() 는 아무것도 하지 않는다
    CommandResponder() = default;
    explicit CommandResponder(Send send) : send_(std::move(send)) {}

    ~CommandResponder() { fail_if_incomplete(); }

    CommandResponder(const CommandResponder&) = delete;
    CommandResponder& operator=(const CommandResponder&) = delete;

    // 이동하면 응답 통로가 새 객체로 넘어가고, 원래 객체는 빈 responder 가 된다
    CommandResponder(CommandResponder&& other) noexcept
        : send_(std::exchange(other.send_, nullptr)),
          completed_(std::exchange(other.completed_, false)) {}
    CommandResponder& operator=(CommandResponder&& other) noexcept {
        if (this != &other) {
            fail_if_incomplete();
            send_ = std::exchange(other.send_, nullptr);
            completed_ = std::exchange(other.completed_, false);
        }
        return *this;
    }

    // 결과를 보낸다. 최종 결과를 이미 보냈거나 빈 responder 면 아무것도 하지 않는다
    void send(const CommandAck& ack) {
        if (!send_ || completed_) {
            return;
        }
        completed_ = ack.result != MAV_RESULT_IN_PROGRESS;
        send_(ack);
    }

    // IN_PROGRESS 가 아닌 결과를 send() 했으면 true. 그 뒤 send() 는 무시된다
    bool completed() const { return completed_; }

private:
    // Send 는 예외를 던지면 안 된다. 던지면 프로그램을 멈춘다 (소멸자에서 불리기 때문)
    void fail_if_incomplete() noexcept {
        if (!send_ || completed_) {
            return;
        }
        completed_ = true;
        try {
            send_(CommandAck{MAV_RESULT_FAILED, kProgressUnknown, "handler gave no final result"});
        } catch (...) {
            std::terminate();
        }
    }

    Send send_;
    bool completed_ = false;
};

class IMavlinkCommandHandler {
public:
    IMavlinkCommandHandler() = default;
    virtual ~IMavlinkCommandHandler() = default;
    IMavlinkCommandHandler(const IMavlinkCommandHandler&) = delete;
    IMavlinkCommandHandler& operator=(const IMavlinkCommandHandler&) = delete;
    IMavlinkCommandHandler(IMavlinkCommandHandler&&) = delete;
    IMavlinkCommandHandler& operator=(IMavlinkCommandHandler&&) = delete;

    // 이 handler 가 처리하는 MAV_CMD 번호
    virtual std::uint16_t command() const = 0;

    // 명령 하나를 처리한다. 결과는 반환값이 아니라 responder 로 보낸다. 통제기에 보내는 첫
    // 응답(IN_PROGRESS 또는 DENIED)도 추종 제어의 응답을 받은 뒤에 정해지므로 process_events() 의
    // 다음 호출에서 나올 수 있다. 같은 명령이 진행 중에 다시 오면 MavlinkCommandReceiver 가
    // TEMPORARILY_REJECTED 로 답하고 handler 를 부르지 않는다 (MAVLink command protocol). 완료
    // 직후의 재전송에는 마지막 최종 결과를 다시 보낸다.
    virtual void handle(const CommandRequest& request, CommandResponder responder) = 0;
};

}  // namespace argus::onboard

#endif  // ARGUS_ONBOARD_MAVLINK_COMMAND_HANDLER_HPP_
