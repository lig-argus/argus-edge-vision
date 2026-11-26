// 추적기 입력의 최신 프레임만 꺼내기 시험 (2026-11-09).
// 쌓인 메시지 중 마지막 정상 메시지만 나오는지, 깨진 메시지를 건너뛰는지 inproc로 확인한다.
#include "argus/runtime/follow/include/latest_pixel_receive.hpp"

#include <zmq.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {
int& failures() {
    static int count = 0;
    return count;
}
void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures();
        std::cerr << "실패: " << what << '\n';
    }
}

const std::string kTopic = "perception/observations";

void send_parts(void* socket, const std::vector<std::string>& parts) {
    for (std::size_t index = 0; index < parts.size(); ++index) {
        const int flags = index + 1 < parts.size() ? ZMQ_SNDMORE : 0;
        zmq_send(socket, parts.at(index).data(), parts.at(index).size(), flags);
    }
}

// 보낸 메시지가 모두 SUB에 도착할 때까지 기다린다 (inproc라도 비동기다).
void settle() { std::this_thread::sleep_for(std::chrono::milliseconds{50}); }
}  // namespace

int main() {
    void* context = zmq_ctx_new();
    void* pub = zmq_socket(context, ZMQ_PUB);
    void* sub = zmq_socket(context, ZMQ_SUB);
    int high_water = 100;
    zmq_setsockopt(pub, ZMQ_SNDHWM, &high_water, sizeof(high_water));
    zmq_setsockopt(sub, ZMQ_RCVHWM, &high_water, sizeof(high_water));
    zmq_bind(pub, "inproc://latest-pixel");
    zmq_connect(sub, "inproc://latest-pixel");
    zmq_setsockopt(sub, ZMQ_SUBSCRIBE, kTopic.data(), kTopic.size());
    settle();

    // 1. 다섯 개가 쌓이면 마지막 하나만 나오고 네 개를 건너뛴다
    for (int frame = 1; frame <= 5; ++frame) {
        send_parts(pub, {kTopic, "frame" + std::to_string(frame)});
    }
    settle();
    auto latest = follow::receive_latest_pixel(sub, kTopic, 1024);
    check(latest.valid && latest.payload == "frame5", "최신 프레임: " + latest.payload);
    check(latest.skipped == 4 && latest.invalid == 0, "건너뛴 수");

    // 2. 비었으면 정상 메시지 없음
    latest = follow::receive_latest_pixel(sub, kTopic, 1024);
    check(!latest.valid && !latest.error.empty(), "빈 대기열");

    // 3. 가장 새 메시지가 깨졌으면(부분 3개) 그 앞의 정상 메시지를 쓴다
    send_parts(pub, {kTopic, "frame6"});
    send_parts(pub, {kTopic, "frame7", "extra"});
    settle();
    latest = follow::receive_latest_pixel(sub, kTopic, 1024);
    check(latest.valid && latest.payload == "frame6",
          "깨진 최신 메시지 뒤 정상: " + latest.payload);
    check(latest.skipped == 0 && latest.invalid == 1, "깨진 메시지 수");

    // 4. 깨진 메시지 뒤 정상 메시지가 오면 정상 메시지를 쓴다 (크기 초과 → 정상)
    send_parts(pub, {kTopic, std::string(2048, 'x')});
    send_parts(pub, {kTopic, "frame8"});
    settle();
    latest = follow::receive_latest_pixel(sub, kTopic, 1024);
    check(latest.valid && latest.payload == "frame8", "크기 초과 뒤 정상: " + latest.payload);
    check(latest.invalid == 1, "크기 초과 수");

    // 5. 깨진 메시지만 있으면 기존과 같은 오류 문구
    send_parts(pub, {kTopic});
    settle();
    latest = follow::receive_latest_pixel(sub, kTopic, 1024);
    check(!latest.valid && latest.error == "pixel multipart topic invalid",
          "부분 1개 오류: " + latest.error);
    send_parts(pub, {kTopic, "frame9", "extra"});
    settle();
    latest = follow::receive_latest_pixel(sub, kTopic, 1024);
    check(!latest.valid && latest.error == "pixel multipart size invalid",
          "부분 3개 오류: " + latest.error);

    // 6. 한 번에 받는 수의 상한
    for (int frame = 0; frame < 10; ++frame) {
        send_parts(pub, {kTopic, "burst" + std::to_string(frame)});
    }
    settle();
    latest = follow::receive_latest_pixel(sub, kTopic, 1024, 4);
    check(latest.valid && latest.payload == "burst3" && latest.skipped == 3, "상한 4개");
    latest = follow::receive_latest_pixel(sub, kTopic, 1024);
    check(latest.valid && latest.payload == "burst9" && latest.skipped == 5, "상한 뒤 나머지");

    zmq_close(sub);
    zmq_close(pub);
    zmq_ctx_term(context);
    if (failures() == 0) {
        std::cout << "latest pixel receive: all checks passed\n";
    }
    return failures() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
