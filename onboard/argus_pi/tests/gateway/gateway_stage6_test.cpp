// 게이트웨이 6단계 단위 시험: HTTP 서버(실제 루프백 소켓)와 프로세스 확인 도우미.
// 요청 줄·머리·본문 읽기, HTTP/1.0 응답과 연결 종료, 처리기 예외 때 응답 없이 닫기,
// TCP 듣기 확인, 실행 파일로 프로세스 찾기, 소켓 소유 확인, 로그를 붙이는 실행.
// HTTP API 내용과 추종기 준비는 tests/gateway/differential/stage6_diff.py, stage6b_diff.py가
// 맡는다.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>

#include "argus/gateway/http/include/http_server.hpp"
#include "argus/gateway/service/include/process_supervisor.hpp"

namespace gw = argus::gateway;

namespace {

int& failures() {
    static int count = 0;
    return count;
}

void check(bool condition, const std::string& what) {
    if (!condition) {
        ++failures();
        std::cerr << "실패: " << what << '\n';
    }
}

// 요청을 그대로 보내고 연결이 닫힐 때까지 받은 응답 전체
std::string exchange(int port, const std::string& request) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        throw std::runtime_error("socket");
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    // POSIX 소켓 API는 sockaddr*를 받는다
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&address),  // NOLINT(*-reinterpret-cast)
                  sizeof(address)) != 0) {
        ::close(fd);
        throw std::runtime_error("connect");
    }
    ::send(fd, request.data(), request.size(), 0);
    std::string response;
    std::array<char, 4096> buffer{};
    for (;;) {
        const ssize_t got = ::recv(fd, buffer.data(), buffer.size(), 0);
        if (got <= 0) {
            break;
        }
        response.append(buffer.data(), static_cast<std::size_t>(got));
    }
    ::close(fd);
    return response;
}

std::string body_of(const std::string& response) {
    const auto at = response.find("\r\n\r\n");
    return at == std::string::npos ? "" : response.substr(at + 4);
}

void http() {
    gw::HttpServer server(0, [](const gw::HttpRequest& request) {
        if (request.path == "/boom") {
            throw std::runtime_error("처리기 예외");
        }
        const std::string length = request.header("content-length").value_or("0");
        const std::string body = request.read(static_cast<std::size_t>(std::stoul(length)));
        if (request.path == "/echo") {
            return gw::HttpResponse{200,
                                    R"({"method": ")" + request.method + R"(", "body": ")" + body +
                                        R"(", "type": ")" +
                                        request.header("Content-Type").value_or("") + "\"}",
                                    true};
        }
        return gw::HttpResponse{404, R"({"error": "경로 없음"})", true};
    });
    std::thread loop([&server] { server.serve_forever(); });
    const int port = server.port();
    const std::string posted =
        exchange(port,
                 "POST /echo HTTP/1.1\r\nHost: x\r\nContent-Type: application/json\r\n"
                 "Content-Length: 7\r\n\r\n{\"a\":1}");
    check(posted.rfind("HTTP/1.0 200 OK\r\n", 0) == 0, "200 응답 줄");
    check(posted.find("Content-Type: application/json; charset=utf-8\r\n") != std::string::npos &&
              posted.find("Content-Length: ") != std::string::npos &&
              posted.find("Cache-Control: no-store\r\n") != std::string::npos,
          "응답 머리");
    check(body_of(posted) == R"({"method": "POST", "body": "{"a":1}", "type": "application/json"})",
          "본문 읽기와 머리 이름 대소문자 무시");
    const std::string missing = exchange(port, "GET /none HTTP/1.0\r\n\r\n");
    check(missing.rfind("HTTP/1.0 404 Not Found\r\n", 0) == 0, "404");
    const std::string unsupported = exchange(port, "PUT /echo HTTP/1.0\r\n\r\n");
    check(unsupported.rfind("HTTP/1.0 501", 0) == 0, "지원하지 않는 메서드");
    const std::string broken = exchange(port, "garbage\r\n\r\n");
    check(broken.rfind("HTTP/1.0 400", 0) == 0, "요청 줄 오류");
    check(exchange(port, "GET /boom HTTP/1.0\r\n\r\n").empty(), "처리기 예외는 응답 없이 닫기");
    server.shutdown();
    loop.join();
    server.close();
}

void processes() {
    // 듣는 소켓을 열고 이 프로세스가 그 포트를 가졌는지 본다
    const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener < 0) {
        throw std::runtime_error("socket");
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    // POSIX 소켓 API는 sockaddr*를 받는다
    auto* generic = reinterpret_cast<sockaddr*>(&address);  // NOLINT(*-reinterpret-cast)
    if (::bind(listener, generic, sizeof(address)) != 0 || ::listen(listener, 1) != 0) {
        ::close(listener);
        throw std::runtime_error("bind");
    }
    socklen_t length = sizeof(address);
    ::getsockname(listener, generic, &length);
    const int port = ntohs(address.sin_port);
    check(gw::tcp_listening(port), "TCP 듣기 확인");
    check(gw::owns_listeners(::getpid(), {port}), "소켓 소유 확인");
    check(!gw::owns_listeners(::getpid(), {port, 1}), "가지지 않은 포트");
    ::close(listener);
    check(!gw::tcp_listening(port), "닫힌 포트");
    // 실행 파일로 자기 프로세스를 찾는다
    const auto self = std::filesystem::read_symlink("/proc/self/exe");
    check(gw::matching_process(self).has_value(), "실행 파일로 프로세스 찾기");
    // 로그를 붙여 실행하고 끝난 코드를 받는다
    const auto folder =
        std::filesystem::temp_directory_path() / ("gateway-stage6-" + std::to_string(::getpid()));
    std::filesystem::create_directories(folder);
    gw::ChildProcess child =
        gw::start_process(folder, folder, "echo", {"sh", "-c", "echo hello; exit 3"});
    check(child.wait(5) == 3, "실행 종료 코드");
    std::ifstream log(folder / "echo.log");
    std::string line;
    std::getline(log, line);
    check(line == "hello", "로그에 출력 붙이기");
    std::ifstream pid_file(folder / "echo.pid");
    int pid = 0;
    pid_file >> pid;
    check(pid == child.pid(), "PID 파일");
    bool missing = false;
    try {
        gw::start_process(folder, folder, "none", {"no-such-binary-argus"});
    } catch (const gw::OsError& error) {
        missing = std::string(error.what()).rfind("[Errno 2] ", 0) == 0;
    }
    check(missing, "없는 실행 파일은 원본 OSError 형식");
    // 게이트웨이처럼 종료 신호를 막고 SIGPIPE를 무시한 부모에서도 자식은 막힘·무시 없이
    // 시작해야 한다. 그래야 SIGTERM으로 정상 종료된다 (2026-11-06 실기).
    sigset_t blocked;
    sigset_t previous;
    sigemptyset(&blocked);
    sigaddset(&blocked, SIGTERM);
    sigaddset(&blocked, SIGINT);
    ::pthread_sigmask(SIG_BLOCK, &blocked, &previous);
    const auto old_pipe = ::signal(SIGPIPE, SIG_IGN);
    gw::ChildProcess masks = gw::start_process(
        folder, folder, "masks", {"sh", "-c", "grep -E '^Sig(Blk|Ign):' /proc/self/status"});
    check(masks.wait(5) == 0, "신호 상태 자식 종료");
    std::ifstream masks_log(folder / "masks.log");
    const std::string masks_text((std::istreambuf_iterator<char>(masks_log)),
                                 std::istreambuf_iterator<char>());
    check(masks_text.find("SigBlk:\t0000000000000000") != std::string::npos,
          "자식은 신호를 막지 않음: " + masks_text);
    // sh는 SIGPIPE를 스스로 무시하지 않으므로 SigIgn의 SIGPIPE(13번, 0x1000) 비트가 꺼져야 한다
    const auto ign_at = masks_text.find("SigIgn:\t");
    check(ign_at != std::string::npos &&
              (std::stoull(masks_text.substr(ign_at + 8, 16), nullptr, 16) & 0x1000ULL) == 0,
          "자식은 SIGPIPE를 무시하지 않음: " + masks_text);
    gw::ChildProcess sleeper = gw::start_process(folder, folder, "sleeper", {"sleep", "30"});
    ::kill(sleeper.pid(), SIGTERM);
    const auto stopped = sleeper.wait(5);
    check(stopped == -SIGTERM, "막힌 부모가 띄운 자식도 SIGTERM으로 종료");
    if (!stopped) {
        ::kill(sleeper.pid(), SIGKILL);
        static_cast<void>(sleeper.wait(5));
    }
    ::signal(SIGPIPE, old_pipe);
    ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
    std::filesystem::remove_all(folder);
}

}  // namespace

int main() {
    try {
        http();
        processes();
    } catch (const std::exception& error) {
        std::cerr << "예외: " << error.what() << '\n';
        return 1;
    }
    if (failures() != 0) {
        std::cerr << failures() << "개 실패\n";
        return 1;
    }
    std::cout << "GatewayStage6Test 통과\n";
    return 0;
}
