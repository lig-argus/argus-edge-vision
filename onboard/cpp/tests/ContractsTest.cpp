#include "argus/adapters/MessageBus.hpp"
#include "argus/application/BoxPostprocessor.hpp"
#include "argus/application/EnvironmentProfile.hpp"
#include "argus/application/FrameQueue.hpp"
#include "argus/contracts/Track.hpp"
#include <iostream>
#include <limits>
static void require(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
template <class F> static void rejects(F &&function) {
    bool rejected = false;
    try {
        function();
    } catch (const std::exception &) {
        rejected = true;
    }
    require(rejected, "invalid contract accepted");
}
int main() {
    try {
        argus::FrameTiming receipt{1000, 100, std::nullopt, std::nullopt, "host_monotonic"};
        rejects([&] { receipt.require_exposure_time(); });
        receipt.exposure_monotonic_ns = 50;
        require(receipt.require_exposure_time() == 50, "exposure lost");
        receipt.exposure_monotonic_ns = 101;
        rejects([&] { receipt.validate(); });
        receipt.exposure_monotonic_ns.reset();
        argus::Frame frame{7, 4, 4, 12, "BGR8", std::make_shared<std::vector<std::uint8_t>>(48), receipt};
        frame.validate();
        auto invalid = frame;
        invalid.stride = 8;
        rejects([&] { invalid.validate(); });
        argus::FrameQueue queue(2);
        queue.put(frame);
        frame.frame_id = 8;
        queue.put(frame);
        frame.frame_id = 9;
        queue.put(frame);
        queue.close();
        require(queue.dropped() == 1, "queue did not drop oldest");
        argus::Frame out;
        require(queue.take(out, std::chrono::milliseconds(1)) && out.frame_id == 8,
                "FIFO/drop-oldest changed");
        require(queue.take(out, std::chrono::milliseconds(1)) && out.frame_id == 9 && queue.drained(),
                "queue did not drain");
        rejects([] { argus::FrameQueue invalid(0); });
        argus::FullBox box{0, "target", 0.9, 0, 0, 3, 3};
        argus::Track track{42, box, argus::TrackState::Confirmed, 100, 50};
        auto predicted = track.with_prediction(box);
        require(predicted.last_real_detection_monotonic_ns == 100 && predicted.ttl_ns == 50,
                "prediction refreshed TTL");
        require(!predicted.stale(149) && predicted.stale(150) && predicted.stale(99),
                "TTL boundary/clock policy");
        argus::TrackBatch batch{1, 100, 1000, {predicted}};
        require(batch.valid_track(42, 149) && !batch.valid_track(42, 151),
                "fresh batch refreshed stale track");
        require(!track.with_real_detection(box, 140).stale(180), "real detection did not refresh");
        rejects([&] { track.with_real_detection(box, 99); });
        auto missing = track;
        missing.last_real_detection_monotonic_ns.reset();
        require(missing.stale(100), "missing timestamp accepted");
        auto zero = track;
        zero.ttl_ns = 0;
        require(zero.stale(100), "zero TTL accepted");
        argus::EnvironmentProfile profile{"REAL_IR",       "ir_sdk",        "hailo_yolox",
                                          "bytetrack_cmc", "mavsdk_serial", "camera_pixel"};
        for (const auto &name :
             {"bytetrack_cmc", "ocsort_cmc", "hybridsort_hmiou_off_cmc", "hybridsort_hmiou_on_cmc"}) {
            profile.tracker = name;
            argus::validate_profile(profile);
        }
        profile.coordinate_frame = "gazebo_world_enu";
        rejects([&] { argus::validate_profile(profile); });
        frame.frame_id = 7;
        argus::DetectionResult result{7, 120, "session", "model", 1, 2, 3, {box}};
        argus::BoxPostprocessor post;
        post.validate(result, frame, "session");
        result.frame_id = 8;
        rejects([&] { post.validate(result, frame, "session"); });
        result.frame_id = 7;
        result.full_boxes[0].x2 = 4;
        rejects([&] { post.validate(result, frame, "session"); });
        result.full_boxes[0] = box;
        result.full_boxes[0].confidence = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { post.validate(result, frame, "session"); });
        argus::Observation observation;
        observation.frame_id = 7;
        observation.session_id = "session";
        observation.timing = receipt;
        observation.full_boxes = {box};
        auto bytes = argus::observation_message(observation);
        auto decoded = msgpack::unpack(bytes.data(), bytes.size());
        auto map = decoded.get().as<std::map<std::string, msgpack::object>>();
        require(map.size() == 23 && map.at("schema").as<std::string>() == "argus.perception.observations.v2",
                "observation schema/framing");
        require(map.at("seq").as<unsigned>() == 7 && !map.at("exposure_time_valid").as<bool>() &&
                    map.at("exposure_monotonic_ns").is_nil(),
                "frame identity/time falsified");
        auto detections = map.at("detections").as<std::vector<msgpack::object>>();
        auto b = detections[0].as<std::map<std::string, msgpack::object>>();
        require(b.size() == 8 && b.at("track_id").is_nil() && !b.count("hull_x1"),
                "full box contract changed");
        std::cout << "Frame/queue/exposure/TTL/profile/full-box/MessagePack contracts passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
