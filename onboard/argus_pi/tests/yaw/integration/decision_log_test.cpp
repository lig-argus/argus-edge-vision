#include "argus/runtime/follow/include/decision_log.hpp"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::filesystem::path directory =
        std::filesystem::path(argv[1]) /
        ("decision-log-test-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    assert(!std::filesystem::exists(directory));
    assert(std::filesystem::create_directories(directory));
    const auto path = directory / "trace.jsonl";
    const auto retry_path = directory / "retry.jsonl";
    {
        follow::DecisionLog retry(retry_path);
        std::promise<void> unblock;
        auto gate = unblock.get_future().share();
        std::atomic<bool> started{false};
        assert(retry.try_record([gate, &started] {
            started = true;
            gate.wait();
            return follow::NativeJson{{"schema", "blocking"}};
        }));
        for (int i = 0; i < 200 && !started; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(started);
        for (int i = 0; i < 32; ++i) {
            assert(retry.try_record(
                [i] { return follow::NativeJson{{"schema", "filler"}, {"sequence", i}}; }, true));
        }
        std::function<follow::NativeJson()> saved_fault = [] {
            return follow::NativeJson{{"schema", "fault"},
                                      {"stop_trigger_reason", "original_reason"},
                                      {"tick_monotonic_ns", 12345}};
        };
        assert(!retry.try_record(saved_fault, true, false));
        unblock.set_value();
        bool accepted = false;
        for (int i = 0; i < 400 && !accepted; ++i) {
            accepted = retry.try_record(saved_fault, true, false);
            if (!accepted) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        assert(accepted);
    }
    {
        std::ifstream in(retry_path);
        std::string line;
        bool found = false;
        while (std::getline(in, line)) {
            const auto record = follow::NativeJson::parse(line);
            if (record.value("schema", std::string{}) == "fault") {
                found = record.value("stop_trigger_reason", std::string{}) == "original_reason" &&
                        record.value("tick_monotonic_ns", 0) == 12345;
            }
            assert(record.value("dropped_since_previous_record", 0u) == 0u);
        }
        assert(found);
    }
    assert(std::filesystem::remove(retry_path));
    // Force a bounded rollover and verify that existing evidence survives in backup slots.
    {
        std::ofstream old1(path.string() + ".1");
        old1 << "previous-backup";
        std::ofstream full(path, std::ios::binary);
        full << std::string(8 * 1024 * 1024 - 256, 'x');
    }
    constexpr int kAttempts = 10000;
    {
        follow::DecisionLog log(path);
        const auto started = std::chrono::steady_clock::now();
        for (int i = 0; i < kAttempts; ++i) {
            log.try_record([i] {
                // Simulates slow disk / serialization without blocking the producer.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                return follow::NativeJson{{"schema", "test"}, {"sequence", i}};
            });
        }
        const auto elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
                .count();
        std::cout << "enqueue_10000_ms=" << elapsed_ms << "\n";
        // A deliberately 1ms/record worker must not make a producer wait for its backlog.
        assert(elapsed_ms < 2000);
    }
    assert(std::filesystem::exists(path.string() + ".1"));
    assert(std::filesystem::file_size(path.string() + ".1") == 8 * 1024 * 1024 - 256);
    std::string backup;
    std::ifstream(path.string() + ".2") >> backup;
    assert(backup == "previous-backup");
    std::ifstream actual(path);
    std::string line;
    int previous = -1, written = 0;
    std::uint64_t lost = 0;
    while (std::getline(actual, line)) {
        const auto value = follow::NativeJson::parse(line);
        lost += value.value("dropped_since_previous_record", std::uint64_t{0});
        if (value.value("schema", std::string{}) == "test") {
            const int sequence = value.at("sequence");
            assert(sequence > previous);
            previous = sequence;
            ++written;
        }
    }
    assert(written > 0);
    assert(static_cast<std::uint64_t>(written) + lost == kAttempts);
    assert(std::filesystem::file_size(path) <= 8 * 1024 * 1024);
    actual.close();
    // A critical event is visible before the worker is destroyed, including when
    // it arrives immediately after the regular periodic flush.
    const auto critical_path = directory / "critical.jsonl";
    {
        follow::DecisionLog log(critical_path);
        log.try_record([] { return follow::NativeJson{{"sequence", 1}}; });
        for (int i = 0; i < 200 && (!std::filesystem::exists(critical_path) ||
                                    std::filesystem::file_size(critical_path) == 0);
             ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(std::filesystem::exists(critical_path));
        assert(std::filesystem::file_size(critical_path) > 0);
        const auto initial_bytes = std::filesystem::file_size(critical_path);
        log.try_record([] { return follow::NativeJson{{"sequence", 2}}; }, true);
        for (int i = 0; i < 200 && std::filesystem::file_size(critical_path) == initial_bytes;
             ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(std::filesystem::file_size(critical_path) > initial_bytes);
        std::ifstream active(critical_path);
        std::string first, second;
        assert(static_cast<bool>(std::getline(active, first)));
        assert(static_cast<bool>(std::getline(active, second)));
        assert(follow::NativeJson::parse(second).at("sequence") == 2);
    }
    assert(std::filesystem::remove(critical_path));
    // Normal backlog cannot consume the dedicated critical-event slots.
    const auto priority_path = directory / "priority.jsonl";
    {
        follow::DecisionLog log(priority_path);
        std::promise<void> unblock;
        auto released = unblock.get_future().share();
        std::atomic<bool> worker_started{false};
        log.try_record([&] {
            worker_started.store(true);
            released.wait();
            return follow::NativeJson{{"sequence", -1}};
        });
        for (int i = 0; i < 200 && !worker_started.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(worker_started.load());
        for (int i = 0; i < 1000; ++i) {
            log.try_record([i] { return follow::NativeJson{{"sequence", i}}; });
        }
        log.try_record([] { return follow::NativeJson{{"sequence", 2000}}; }, true);
        unblock.set_value();
        bool found = false;
        for (int i = 0; i < 400 && !found; ++i) {
            if (std::filesystem::exists(priority_path)) {
                std::ifstream active(priority_path);
                std::string row;
                while (std::getline(active, row)) {
                    auto event = follow::NativeJson::parse(row, nullptr, false);
                    if (!event.is_discarded() && event.value("sequence", 0) == 2000) {
                        found = true;
                    }
                }
            }
            if (!found) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        assert(found);
    }
    assert(std::filesystem::remove(priority_path));
    // Failure to open is contained within the diagnostics worker, not propagated to caller.
    {
        follow::DecisionLog unavailable(directory / "missing-parent" / "trace.jsonl");
        unavailable.try_record([] { return follow::NativeJson{{"sequence", 1}}; });
        for (int i = 0; i < 100 && unavailable.healthy(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(!unavailable.healthy());
    }
    // Linux /dev/full emulates disk full after open; the worker must report failure.
    if (std::filesystem::exists("/dev/full")) {
        follow::DecisionLog full("/dev/full");
        full.try_record([] { return follow::NativeJson{{"sequence", 1}}; });
        for (int i = 0; i < 100 && full.healthy(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        assert(!full.healthy());
    }
    // Remove only files generated by this uniquely named test instance.
    assert(std::filesystem::remove(path));
    assert(std::filesystem::remove(path.string() + ".1"));
    assert(std::filesystem::remove(path.string() + ".2"));
    assert(std::filesystem::remove(directory));
    return 0;
}
