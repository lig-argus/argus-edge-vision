// 추종, 스캔 명령을 받는 인터페이스
// 규약: docs/spec/process_interfaces.md 4장, 5.2 follow/command, 5.2.1 follow/result
// 구현: 추종 제어(FollowController)
// 사용: onboard 의 ZeroMQ 어댑터 FollowCommandServer
#ifndef ARGUS_VEHICLE_API_FOLLOW_COMMAND_HPP_
#define ARGUS_VEHICLE_API_FOLLOW_COMMAND_HPP_

#include "argus/vehicle_api/types.hpp"

namespace argus::vehicle_api {

// 공통 계약
// - 반환값은 요청을 받은 시점에 정해지는 결과다 (수락하면 InProgress, 조건 불충족이면 Rejected).
//   그 뒤의 진행(Succeeded, Failed)은 follow/status 의 command_results 에서 같은 command_id 로
//   찾는다.
// - 같은 command_id 로 다시 부르면 재실행하지 않고 저장된 결과를 반환한다. 단, 세션,
//   valid_until_monotonic_ns, 명령 내용은 다시 검사하므로 내용이 바뀐 요청이나 stop 뒤에 온 예전
//   track_start 는 Rejected 가 될 수 있다.
// - 표적 존재/상태/FC 조건 검사는 구현이 한다.
//   통제기 제어권과 매개변수 범위 검사는 onboard 가 부르기 전에 한다.
// - FC 제어 명령 송신권(process_interfaces.md 6장)의 확인은 onboard 의 FC 명령 검사가,
//   요청의 취소는 onboard 의 ZeroMQ 어댑터가 맡는다. 구현은 요청의 공통 필드(command_id,
//   session_id, seq, fc_epoch)를 FollowResult 에 그대로 남긴다.
// - ScanRequest: 지정된 값이 무한대가 아닌지와 허용 범위 안인지는 onboard 가 부르기 전에
//   검사한다. 기본값을 적용한 뒤 yaw 범위, yaw 속도, 제한 시간의 조합은 구현이 검사하고
//   맞지 않으면 Rejected. 무한대나 범위 밖의 값을 기본값으로 바꾸지 않는다.
class IFollowCommand {
public:
    IFollowCommand() = default;
    virtual ~IFollowCommand() = default;
    IFollowCommand(const IFollowCommand&) = delete;
    IFollowCommand& operator=(const IFollowCommand&) = delete;
    IFollowCommand(IFollowCommand&&) = delete;
    IFollowCommand& operator=(IFollowCommand&&) = delete;

    // FC 가 실제로 Hold 일 때만 수락한다 (docs/spec/icd.md 4.2 조건 ⑥)
    virtual FollowResult track_start(const RequestHeader& header, const TrackTarget& target) = 0;
    virtual FollowResult stop(const RequestHeader& header, StopReason reason) = 0;
    // Scan(Hold 상태 yaw 회전 탐색) 시작. follow/command 메시지의 action 이름은 "search" 이며
    // 변환은 ZeroMQ 어댑터가 한다
    virtual FollowResult scan_start(const RequestHeader& header, const ScanRequest& scan) = 0;
};

}  // namespace argus::vehicle_api

#endif  // ARGUS_VEHICLE_API_FOLLOW_COMMAND_HPP_
