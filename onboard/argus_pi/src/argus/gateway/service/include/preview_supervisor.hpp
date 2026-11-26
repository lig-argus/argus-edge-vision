// Pi 영상 준비 (src/argus/runtime/streaming/preview_producer_supervisor.py ensure와
// main_gcs_video_manager.py ensure와 같은 규칙). FC를 열지 않는다. 명시적 준비 요청에서만 부른다.
#pragma once

#include <filesystem>
#include <set>
#include <string>
#include <vector>

#include "argus/gateway/common/include/msgpack_codec.hpp"

namespace argus::gateway {

// 원본 RuntimeError (follow_service가 영상 준비 실패로만 남긴다)
class PreviewError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// 원본 udp_inodes(port): /proc/net/udp·udp6에서 그 로컬 포트의 소켓 inode
std::set<std::string> udp_inodes(int port);

// 원본 port_owners(inodes): 그 소켓을 가진 (pid, cmdline 공백 연결)
std::vector<std::pair<int, std::string>> port_owners(const std::set<std::string>& inodes);

// 원본 main_gcs_video_manager.ensure(root, bus, tracks, python): UDP 25002 중계를 한 번만 띄운다
Wire ensure_main_gcs_video(const std::filesystem::path& root, const std::string& bus,
                           const std::string& tracks, const std::string& python);

// 원본 preview_producer_supervisor.ensure(root, timeout): 영상 버스·생산자를 준비하고 실제 프레임
// 하나를 확인한 뒤 메인 GUI 영상 중계를 맞춘다. 실패하면 PreviewError.
Wire ensure_preview(const std::filesystem::path& root, double timeout_s);

}  // namespace argus::gateway
