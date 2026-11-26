// 정비용 프로세스 목록·종료와 IR 지상 진단 (Pi src/argus/diagnostics/tracking/
// maintenance_processes.py, ground_diagnostic_service.py와 같은 규칙). FC 명령은 보내지 않는다.
//
// 원본과 다른 점: C++ 게이트웨이(bin/argus_fc_gateway, 작업 폴더 ROOT)도 원본 Python 게이트웨이처럼
// 보호되는 "IR FC 연결부"로 본다 (교체 뒤에도 정비 화면에서 끌 수 없게).
#pragma once

#include <filesystem>
#include <functional>
#include <string>

#include "argus/gateway/common/include/msgpack_codec.hpp"

namespace argus::gateway {

// C++ 게이트웨이 실행 파일 이름 (ROOT/bin 아래)
inline constexpr const char* kGatewayBinaryName = "argus_fc_gateway";

struct MaintenanceOptions {
    std::filesystem::path root;  // 원본 ROOT (설치 폴더)
    // 원본 forward_vmy(body, uid, timeout_ms)
    std::function<Wire(const Wire& body, const std::string& uid, int timeout_ms)> forward;
    // 원본 sys.executable (지상 진단 기록기를 띄울 Python)
    std::string python;
    int gateway_port{8768};
    // 2026-11-24: FC 연결 장치(settings device.uart_device). 비어 있으면 FC 미연결 판정을 하지
    // 않는다.
    std::string uart_device;
};

class Maintenance {
public:
    explicit Maintenance(MaintenanceOptions options);

    // 원본 maintenance_processes.inventory()
    Wire inventory() const;
    // 원본 maintenance_processes.stop(pid, ticks)
    // 2026-11-24 사용자 지시: vehicle_ground가 거짓이면 FC 장치가 없을 때 카메라/탐지 프로세스만
    // 끈다.
    Wire stop(long pid, long ticks, bool vehicle_ground = true) const;
    // 2026-11-24: 전체 재시작(강제). maintenance_processes.py restart로 작업자를 띄운다 (FC 상태
    // 무관).
    Wire restart() const;
    // 최근 전체 재시작 상태 (run/connection/restart-all.json, 없거나 깨졌으면 null)
    Wire restart_status() const;
    // FC 연결 장치 파일(uart_device)이 없으면 참 (FC USB 빠짐)
    bool fc_link_absent() const;
    // FC 없이도 끌 수 있는 카메라/탐지 프로세스 이름인지
    static bool perception_only(const std::string& name);
    // 원본 ground_diagnostic_service.status() / request(action, state)
    Wire diagnostic_status() const;
    Wire diagnostic_request(const Wire& action, const Wire& state) const;

    // 설치 여부 (원본은 이 Python 모듈 파일이 있어야 쓴다)
    bool processes_installed() const;
    bool diagnostic_installed() const;

    // 원본 maintenance_processes.confirmed_ground. 1.2 s 동안 endpoints.fc_state 의 fc/state 를
    // 보고 착지, disarm, 신선(500 ms) 이면 참. 2026-11-25 batch 6a: 팀 [이름][protobuf] 본문을
    // 받는다 (시험에서 직접 부르도록 공개)
    static bool confirmed_ground(const Wire& config);

private:
    std::optional<int> diagnostic_pid() const;

    MaintenanceOptions options_;
};

}  // namespace argus::gateway
