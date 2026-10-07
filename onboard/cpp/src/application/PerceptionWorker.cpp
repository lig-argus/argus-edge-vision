#include "argus/application/PerceptionWorker.hpp"
#include "argus/application/BoxPostprocessor.hpp"
#include "argus/application/FrameQueue.hpp"
#include "argus/application/Recorder.hpp"
#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>
namespace argus {
void PerceptionWorker::run(const volatile std::sig_atomic_t &interrupted) {
    const auto &options = root_.options();
    // Load the Python image worker before opening an SDK producer that has bounded pipe deadlines.
    auto detector = root_.create_detector(context_, session_);
    auto source = root_.create_frame_source();
    FrameQueue queue(options.queue_capacity);
    std::unique_ptr<Recorder> recorder;
    if (!options.latency_csv.empty())
        recorder = std::make_unique<Recorder>(options.latency_csv);
    BoxPostprocessor postprocessor;
    std::atomic<bool> stopping{};
    std::exception_ptr capture_error;
    std::mutex error_mutex;
    bus_.diagnostic(root_.source_name(), "perception_only",
                    "C++ orchestrator; Python image worker. Tracker/CMC/control/VehiclePort not started.",
                    clock_.now());
    std::thread capture([&] {
        try {
            unsigned failures{};
            while (!stopping) {
                Frame frame;
                auto result = source->read(frame);
                if (result == FrameRead::EndOfStream)
                    break;
                if (result == FrameRead::Retry) {
                    if (++failures >= 10)
                        throw std::runtime_error("camera returned 10 consecutive empty frames");
                    continue;
                }
                failures = 0;
                frame.validate();
                queue.put(std::move(frame));
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(error_mutex);
            capture_error = std::current_exception();
        }
        queue.close();
    });
    auto check_capture = [&] {
        std::lock_guard<std::mutex> lock(error_mutex);
        if (capture_error)
            std::rethrow_exception(capture_error);
    };
    auto cleanup = [&] {
        stopping = true;
        source->request_stop();
        queue.close();
        if (capture.joinable())
            capture.join();
    };
    try {
        while (!interrupted) {
            check_capture();
            Frame frame;
            if (!queue.take(frame, std::chrono::milliseconds(100))) {
                check_capture();
                if (queue.drained())
                    break;
                continue;
            }
            auto before = clock_.now().monotonic_ns;
            auto result = detector->infer(frame);
            postprocessor.validate(result, frame, session_);
            check_capture();
            auto stamp = clock_.now();
            if (result.infer_done_monotonic_ns > stamp.monotonic_ns)
                throw std::runtime_error("worker inference timestamp is in the future");
            Observation o;
            o.source = root_.source_name();
            o.session_id = session_;
            o.model = result.model;
            o.frame_id = frame.frame_id;
            o.image_width = frame.width;
            o.image_height = frame.height;
            o.timing = frame.timing;
            o.published_monotonic_ns = stamp.monotonic_ns;
            o.infer_done_monotonic_ns = result.infer_done_monotonic_ns;
            o.preprocess_ms = result.preprocess_ms;
            o.inference_ms = result.inference_ms;
            o.ipc_ms = result.ipc_ms;
            o.queue_dwell_ms = (before - frame.timing.received_monotonic_ns) / 1000000.0;
            o.full_boxes = std::move(result.full_boxes);
            bus_.publish_observation(o);
            if (recorder)
                recorder->record(o, queue.dropped());
            if (options.log_every && o.frame_id % options.log_every == 0)
                std::cout << "frame=" << o.frame_id << " objects=" << o.full_boxes.size()
                          << " infer=" << o.inference_ms << "ms dropped=" << queue.dropped() << std::endl;
        }
        check_capture();
        cleanup();
        if (recorder)
            recorder->close();
    } catch (...) {
        cleanup();
        throw;
    }
}
} // namespace argus
