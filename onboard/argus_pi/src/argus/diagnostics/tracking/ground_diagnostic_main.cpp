// Pure stdin/stdout shadow evaluator. No flight backend, owner, UART or mode commands.
#include <iostream>
#include <memory>

#include "argus/common/serialization/include/native_json_io.hpp"
#include "argus/runtime/config/include/bus_following_config.hpp"
#include "argus/tracking/contracts/include/tracks_v1_codec.hpp"
using namespace follow;
NativeJson track_json(const TrackRecordV1& t) {
    NativeJson v{{"track_id", t.track_id},
                 {"class_id", t.class_id},
                 {"label", t.label},
                 {"bbox", {t.x1, t.y1, t.x2, t.y2}},
                 {"confidence", t.confidence},
                 {"real_detection", t.real_detection},
                 {"last_real_ns", t.last_real_detection_monotonic_ns}};
    v["ground"] = t.ground ? NativeJson(*t.ground) : NativeJson(nullptr);
    return v;
}
int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw std::runtime_error("profile path required");
        }
        const auto config = load_bus_following_config(argv[1]);
        ReacquisitionTrace trace;
        std::unique_ptr<TargetSelector> selector;
        NativeJson identity;
        std::uint64_t sequence = 0;
        std::string line;
        while (std::getline(std::cin, line)) {
            NativeJson out{{"schema", "argus.target.ground_diagnostic.v1"},
                           {"shadow_only", true},
                           {"flight_command_count", 0},
                           {"evaluations", NativeJson::array()}};
            try {
                const auto input = NativeJson::parse(line);
                const auto safe = input.at("safety");
                if (safe.value("fresh", false) != true ||
                    safe.value("identity_ok", false) != true ||
                    safe.value("expected_uid", "") != config.expected_fc_uid ||
                    !safe.contains("armed") || safe.at("armed") != false ||
                    !safe.contains("ground") || safe.at("ground") != true ||
                    safe.value("follow_active", true) || safe.value("busy", true)) {
                    selector.reset();
                    identity = nullptr;
                    throw std::runtime_error("paused_not_verified_disarmed_ground_idle");
                }
                const auto raw = input.at("tracks");
                const auto bytes = NativeJson::to_msgpack(raw);
                const auto decoded = TracksV1Codec::decode(
                    std::string_view(reinterpret_cast<const char*>(bytes.data()), bytes.size()));
                if (!decoded.valid) {
                    throw std::runtime_error(decoded.error);
                }
                const auto& frame = decoded.message;
                const auto now = Clock::time_point{
                    std::chrono::nanoseconds{input.at("received_ns").get<std::int64_t>()}};
                out["source"] = frame.source;
                out["frame_id"] = frame.frame_id;
                out["sample_ns"] = frame.sample_monotonic_ns;
                out["generation"] = frame.generation;
                out["metric_comparable"] =
                    ground_tracks_schema(frame.schema) && !camera_only_tracks(frame);
                if (!out["metric_comparable"].get<bool>()) {
                    selector.reset();
                    identity = nullptr;
                    throw std::runtime_error("camera_only_no_metric_scores");
                }
                const auto selected = input.at("selection");
                if (!selected.is_object()) {
                    selector.reset();
                    identity = nullptr;
                    throw std::runtime_error("waiting_for_gui_selection");
                }
                trace.clear();
                if (!selector || selected != identity) {
                    selector = std::make_unique<TargetSelector>(config.selector, &trace);
                    identity = selected;
                    if (!selector->update(frame, now)) {
                        throw std::runtime_error("initial_frame_rejected");
                    }
                    auto select = selector->execute(
                        TargetCommand{"select", TargetCommandAction::Select,
                                      selected.at("generation"), selected.at("track_id"),
                                      config.expected_fc_uid},
                        now);
                    if (!select.accepted) {
                        selector.reset();
                        throw std::runtime_error(select.reason);
                    }
                    // Initializes ONLY this isolated memory replica. There is no flight runtime.
                    auto start = selector->execute(
                        TargetCommand{"shadow-memory", TargetCommandAction::Start,
                                      selected.at("generation"), selected.at("track_id"),
                                      config.expected_fc_uid},
                        now);
                    if (!start.accepted) {
                        selector.reset();
                        throw std::runtime_error(start.reason);
                    }
                    out["initialized"] = true;
                } else {
                    out["accepted_frame"] = selector->update(frame, now);
                }
                selector->advance(now);
                const auto snapshot = selector->snapshot(now);
                out["phase"] = target_phase_name(snapshot.target_phase);
                out["reason"] = snapshot.recovery_reason;
                out["mission_target_id"] = snapshot.mission_target_id;
                // 이 배포 selector는 prediction deadline 필드를 제공하지 않는다.
                out["prediction_valid_until_ns"] = nullptr;
                for (const auto& call : trace) {
                    NativeJson row{{"previous", track_json(call.previous)},
                                   {"dt_s", call.dt_s},
                                   {"ground_dt_s", call.ground_dt_s},
                                   {"velocity_ne", {call.velocity_north, call.velocity_east}},
                                   {"kind", static_cast<int>(call.result)},
                                   {"candidates", NativeJson::array()}};
                    row["predicted_ground_ne"] =
                        call.previous.ground
                            ? NativeJson::array({(*call.previous.ground)[0] +
                                                     call.velocity_north * call.ground_dt_s,
                                                 (*call.previous.ground)[1] +
                                                     call.velocity_east * call.ground_dt_s})
                            : NativeJson(nullptr);
                    row["best_score"] =
                        call.best_score ? NativeJson(*call.best_score) : NativeJson(nullptr);
                    row["second_score"] =
                        call.second_score ? NativeJson(*call.second_score) : NativeJson(nullptr);
                    row["score_margin"] = call.best_score && call.second_score
                                              ? NativeJson(*call.second_score - *call.best_score)
                                              : NativeJson(nullptr);
                    for (const auto& c : call.candidates) {
                        auto v = track_json(c.track);
                        v["reason"] = c.reason;
                        v["distance_m"] =
                            c.distance ? NativeJson(*c.distance) : NativeJson(nullptr);
                        v["area_ratio"] =
                            c.area_ratio ? NativeJson(*c.area_ratio) : NativeJson(nullptr);
                        v["score"] = c.score ? NativeJson(*c.score) : NativeJson(nullptr);
                        row["candidates"].push_back(v);
                    }
                    out["evaluations"].push_back(row);
                }
                out["thresholds"] = {
                    {"ground_gate_m", config.selector.recovery.ground_gate_m},
                    {"score_margin_min", 4.0},
                    {"max_gap_ms", config.selector.recovery.max_gap.count()},
                    {"confirmation_frames", config.selector.recovery.confirmation_frames}};
            } catch (const std::exception& e) {
                out["error"] = e.what();
            }
            out["sequence"] = ++sequence;
            std::cout << out.dump() << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
