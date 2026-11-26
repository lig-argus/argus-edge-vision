// 50ms 정렬 실패 시 새 FC 표본을 기다리는 재시도 동작 검사 (가짜 투영기·가짜 시계).
#include "argus/perception/geometry/include/receipt_alignment_retry.hpp"

#include <cassert>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>

using namespace follow;

namespace {
// fc["seq"]가 ready_seq 이상이면 성공, 그 전에는 reason으로 실패한다.
struct FakeProjector {
    std::int64_t ready_seq;
    std::string reason;
    mutable int calls = 0;
    int project(const NativeJson&, const NativeJson& fc, std::int64_t) const {
        ++calls;
        if (fc.at("seq").get<std::int64_t>() >= ready_seq) {
            return 7;
        }
        throw std::runtime_error(reason);
    }
};

struct Harness {
    std::int64_t clock_ns = 1000000000;
    std::int64_t seq = 0;
    std::string session = "s";
    int epoch = 3;
    int reads = 0;
    NativeJson read() {
        ++reads;
        return NativeJson{{"epoch", epoch}, {"onboard_session", session}, {"seq", seq++}};
    }
};

std::string run_error(FakeProjector& projector, Harness& h, int wait_ms) {
    try {
        project_with_receipt_retry(
            projector, NativeJson::object(), [&] { return h.read(); }, [&] { return h.clock_ns; },
            [&] { h.clock_ns += 2000000; }, std::chrono::milliseconds{wait_ms});
    } catch (const std::exception& error) {
        return error.what();
    }
    return {};
}
}  // namespace

int main() {
    const std::string sdk = "SDK/FC clock alignment exceeds 50ms";
    const std::string receipt = "FC receipt alignment exceeds 50ms";

    // SDK 시각 방식: 새 FC 표본이 오면 기다려서 성공한다.
    {
        FakeProjector p{3, sdk};
        Harness h;
        const int result = project_with_receipt_retry(
            p, NativeJson::object(), [&] { return h.read(); }, [&] { return h.clock_ns; },
            [&] { h.clock_ns += 2000000; }, std::chrono::milliseconds{50});
        assert(result == 7);
        assert(p.calls == 4);
    }
    // 수신 시각 방식도 기존처럼 기다린다.
    {
        FakeProjector p{2, receipt};
        Harness h;
        assert(run_error(p, h, 50).empty());
    }
    // 기다림 0ms면 기존처럼 바로 거부하고 원래 사유를 유지한다.
    {
        FakeProjector p{1, sdk};
        Harness h;
        assert(run_error(p, h, 0) == sdk);
        assert(p.calls == 1);
    }
    // 50ms 안에 표본이 오지 않으면 원래 사유로 거부한다 (수신 시각 문구로 바꾸지 않는다).
    {
        FakeProjector p{1000, sdk};
        Harness h;
        assert(run_error(p, h, 50) == sdk);
        assert(h.clock_ns - 1000000000 <= 52000000);
    }
    // 정렬 이외의 실패는 기다리지 않는다.
    {
        FakeProjector p{1000, "fresh optical flow required"};
        Harness h;
        assert(run_error(p, h, 50) == "fresh optical flow required");
        assert(p.calls == 1 && h.reads == 1);
    }
    // 기다리는 중 FC 문맥(epoch)이 바뀌면 중단한다.
    {
        FakeProjector p{1000, sdk};
        Harness h;
        bool changed = false;
        std::string error;
        try {
            project_with_receipt_retry(
                p, NativeJson::object(),
                [&] {
                    auto fc = h.read();
                    if (changed) {
                        fc["epoch"] = 4;
                    }
                    changed = true;
                    return fc;
                },
                [&] { return h.clock_ns; }, [&] { h.clock_ns += 2000000; },
                std::chrono::milliseconds{50});
        } catch (const std::exception& e) {
            error = e.what();
        }
        assert(error == "FC context changed during receipt retry");
    }
    // 기다림을 쓰려면 onboard 세션이 있어야 한다.
    {
        FakeProjector p{0, sdk};
        Harness h;
        h.session.clear();
        assert(run_error(p, h, 50) == "FC receipt retry context missing");
    }
    return 0;
}
