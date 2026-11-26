// 6단계 follow_service 비교 시험용 가짜 추적기·추종기. 마지막 --port N에서 TCP로 듣기만 한다.
// 시험 폴더의 bin/ 아래에 argus_follow_tracker·argus_follow_controller 이름으로 복사해 쓴다.
// SIGTERM을 받으면 끝난다 (기본 동작). 장치·FC·추종 명령은 다루지 않는다.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, std::next(argv, argc));
    int port = -1;
    for (std::size_t i = 1; i + 1 < args.size(); ++i) {
        if (args.at(i) == "--port") {
            port = std::stoi(args.at(i + 1));
        }
    }
    if (port <= 0) {
        std::cerr << "--port 필요\n";
        return 2;
    }
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "socket 실패\n";
        return 1;
    }
    const int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    // POSIX 소켓 API는 sockaddr*를 받는다
    if (::bind(fd, reinterpret_cast<const sockaddr*>(&address),  // NOLINT(*-reinterpret-cast)
               sizeof(address)) != 0 ||
        ::listen(fd, 16) != 0) {
        std::cerr << "listen 실패 " << port << '\n';
        return 1;
    }
    std::cout << "listening " << port << '\n' << std::flush;
    for (;;) {
        const int client = ::accept(fd, nullptr, nullptr);
        if (client >= 0) {
            ::close(client);
        }
    }
}
