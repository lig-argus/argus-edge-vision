// 통제기 명령 중 최종 결과를 보내지 않은 것이 있는지 (icd 5.1 의 "진행 중인 명령")
// 규약: docs/spec/icd.md 4.1, 5.1
// 사용: MavlinkCommandReceiver 가 CommandActivity 의 값을 바꾸고, ControlAuthority 가
//      ICommandActivity 로 읽는다. MavlinkCommandReceiver 와 ControlAuthority 가 서로를 생성 인자로
//      받지 않도록, 값을 두 객체 밖(main)에 둔다
#ifndef ARGUS_ONBOARD_COMMAND_ACTIVITY_HPP_
#define ARGUS_ONBOARD_COMMAND_ACTIVITY_HPP_

namespace argus::mavlink_gateway {

class ICommandActivity {
public:
    ICommandActivity() = default;
    virtual ~ICommandActivity() = default;
    ICommandActivity(const ICommandActivity&) = delete;
    ICommandActivity& operator=(const ICommandActivity&) = delete;
    ICommandActivity(ICommandActivity&&) = delete;
    ICommandActivity& operator=(ICommandActivity&&) = delete;

    // 통제기 명령 중 최종 결과(IN_PROGRESS 가 아닌 COMMAND_ACK)를 보내지 않은 것이 있으면 true
    virtual bool command_in_progress() const = 0;
};

class CommandActivity : public ICommandActivity {
public:
    bool command_in_progress() const override { return in_progress_; }
    void set_command_in_progress(bool in_progress) { in_progress_ = in_progress; }

private:
    bool in_progress_ = false;
};

}  // namespace argus::mavlink_gateway

#endif  // ARGUS_ONBOARD_COMMAND_ACTIVITY_HPP_
