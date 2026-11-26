// Pi setpoint 송신자 잠금 (Pi tools/real_fc/command_owner.py와 같은 파일·같은 flock).
// GUI 임대나 PID 파일이 아니라 커널 잠금이다. Python 게이트웨이·yaw 송신자와 같은 파일을 공유한다.
#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace argus::gateway {

class CommandOwner {
public:
    // 빈 경로면 잠금을 쓰지 않는다 (항상 획득 성공, status는 enabled=false).
    explicit CommandOwner(std::string path);
    // ARGUS_COMMAND_OWNER_FILE 환경 변수의 경로를 쓴다.
    static CommandOwner from_environment();

    CommandOwner(const CommandOwner&) = delete;
    CommandOwner& operator=(const CommandOwner&) = delete;
    CommandOwner(CommandOwner&& other) noexcept;
    CommandOwner& operator=(CommandOwner&&) = delete;
    ~CommandOwner();

    // 다른 프로세스가 쥐고 있으면 false. 쥐면 {"role": "fc", "pid": N}을 기록한다.
    bool acquire();
    void release();
    // {"enabled", "role", "available"} (원본 status()와 같은 키·값)
    nlohmann::json status() const;
    void close();

    bool held() const { return held_; }
    const std::string& path() const { return path_; }

private:
    std::string path_;
    int fd_{-1};
    bool held_{false};
};

}  // namespace argus::gateway
