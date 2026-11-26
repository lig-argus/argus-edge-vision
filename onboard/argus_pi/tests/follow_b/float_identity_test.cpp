#include <cassert>
#include <fstream>
#include <limits>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "argus/tracking/identity/include/persistent_generation.hpp"

using namespace follow;
template <class F>
bool rejected(F action) {
    try {
        action();
    } catch (const TrackingIdentityError&) {
        return true;
    }
    return false;
}
int main() {
    for (std::uint32_t id = 1; id <= kFloatIdentityMax; ++id) {
        volatile float wire = static_cast<float>(id);
        assert(identity_from_float(wire) == id);
    }
    for (float value : {0.f, -1.f, 1.5f, 16777216.f, std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::quiet_NaN()}) {
        assert(rejected([&] { identity_from_float(value); }));
    }
    FloatTrackIdSequence edge{kFloatIdentityMax};
    assert(edge.allocate() == kFloatIdentityMax);
    assert(rejected([&] { edge.allocate(); }));
    assert(rejected([&] { edge.allocate(); }));
    FloatTrackIdSequence shared;
    std::mutex mutex;
    std::set<std::int64_t> ids;
    std::vector<std::thread> threads;
    for (int i = 0; i < 4; ++i) {
        threads.emplace_back([&] {
            for (int j = 0; j < 1000; ++j) {
                const auto id = shared.allocate();
                std::lock_guard<std::mutex> lock(mutex);
                assert(ids.insert(id).second);
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }
    assert(ids.size() == 4000 && *ids.begin() == 1 && *ids.rbegin() == 4000);
    const auto root = std::filesystem::temp_directory_path() /
                      ("argus-float-identities-" + std::to_string(::getpid()));
    std::filesystem::create_directories(root);
    const auto file = (root / "generation").string();
    assert(rejected([&] { PersistentGeneration missing{file}; }));
    {
        PersistentGeneration first{file, true};
        assert(first.allocate() == 1);
        assert(first.allocate() == 2);
        assert(rejected([&] { PersistentGeneration duplicate{file}; }));
    }
    {
        PersistentGeneration restarted{file};
        assert(restarted.allocate() == 3);
    }
    assert(rejected([&] { PersistentGeneration reset{file, true}; }));
    {
        std::ofstream stream(file);
        stream << "broken\n";
    }
    assert(rejected([&] { PersistentGeneration corrupt{file}; }));
    {
        std::ofstream stream(file);
        stream << "ARGUS_FLOAT_IDS_V1\n16777214\n";
    }
    {
        PersistentGeneration last{file};
        assert(last.allocate() == kFloatIdentityMax);
        assert(rejected([&] { last.allocate(); }));
    }
    {
        PersistentGeneration exhausted{file};
        assert(rejected([&] { exhausted.allocate(); }));
    }
    std::filesystem::remove(file);
    assert(rejected([&] { PersistentGeneration lost{file}; }));
    std::filesystem::remove_all(root);
}
