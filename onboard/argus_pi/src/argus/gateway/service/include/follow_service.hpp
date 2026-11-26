// 추종기·추적기 준비 (Pi tools/real_fc/follow_service.py prepare_vmy·_status와 같은 규칙).
// 명시적 준비만 한다. 비행·추적 시작 명령은 보내지 않는다.
#pragma once

#include <filesystem>
#include <functional>
#include <mutex>
#include <string>

#include "argus/gateway/bridge/include/profile_coordinator.hpp"
#include "argus/gateway/common/include/msgpack_codec.hpp"

namespace argus::gateway {

struct FollowServiceOptions {
    std::filesystem::path root;  // 원본 ROOT
    // 원본 forward_vmy(body, uid, timeout_ms)
    std::function<Wire(const Wire& body, const std::string& uid, int timeout_ms)> forward;
    // 원본 supervisor.ensure(root, timeout) (시험은 바꿔 넣는다)
    std::function<Wire(const std::filesystem::path& root, double timeout_s)> preview;
    double timeout_s{45};
};

class FollowService {
public:
    explicit FollowService(FollowServiceOptions options);

    // 원본 _status(uid): 응답 UID·승인 확인 후 validate_observation_status
    Wire status(const std::string& uid) const;

    // 원본 prepare_vmy(expected_uid, environment, controller, reconfigure, admission)
    Wire prepare(const std::string& uid, const std::string& environment,
                 const std::string& controller, bool reconfigure,
                 const std::function<void()>& admission);

    // FollowProfileCoordinator에 넣을 함수
    ProfileServices services();

private:
    Wire prepare_locked(Wire config, const std::string& uid, const std::string& environment,
                        const std::string& controller, bool reconfigure,
                        const std::function<void()>& admission) const;

    FollowServiceOptions options_;
    std::mutex lock_;  // 원본 _LOCK
};

// 원본 _uses_profile(pid, flag, expected): 마지막 flag 다음 인수가 expected인지
bool uses_profile(int pid, const std::string& flag, const std::string& expected);

}  // namespace argus::gateway
