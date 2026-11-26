// 2026-11-12 실외 장착각 모드 선택 시험: 정비 선택값 검사, 실행 중 모드 판별, 설정·카메라 경로.
// 인자로 ARGUS 배포 폴더를 주면 실제 파일로 경로 선택까지 본다 (FC·프로세스는 열지 않음).
// 2026-11-18 데모 모드: 실내 제어기 선택(indoor_controller), 실행 중 demo 판별, demo 경로.
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include "argus/gateway/bridge/include/follow_preferences.hpp"
#include "argus/gateway/settings/include/deployment_settings.hpp"

namespace gw = argus::gateway;

namespace {
void require(bool value, const char* message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}

gw::Wire preference(const char* environment, const char* controller) {
    gw::Wire value = gw::Wire::object();
    gw::wire_set(value, "environment", environment);
    gw::wire_set(value, "outdoor_controller", controller);
    return value;
}

bool rejected(const gw::Wire& value) {
    try {
        (void)gw::FollowPreferences::validate(value);
    } catch (const gw::ValueError&) {
        return true;
    }
    return false;
}

void preference_test() {
    for (const char* name :
         {"predictive", "predictive_low", "predictive_m30", "predictive_low_m30"}) {
        require(!rejected(preference("outdoor", name)), "outdoor mode accepted");
        gw::Wire status = gw::Wire::object();
        gw::wire_set(status, "configured_follow_environment", "outdoor");
        gw::wire_set(status, "configured_follow_controller", name);
        const gw::Wire active = gw::active_profile(status);
        require(!active.is_null() && active.at("controller") == gw::Wire(name),
                "running mode recognised");
    }
    require(rejected(preference("outdoor", "pid")), "pid rejected");
    require(rejected(preference("outdoor", "predictive_30")), "unknown mode rejected");
    std::cout << "PASS outdoor mode preferences\n";
}

gw::Wire preference3(const char* environment, const char* outdoor, const char* indoor) {
    gw::Wire value = preference(environment, outdoor);
    gw::wire_set(value, "indoor_controller", indoor);
    return value;
}

void write_text(const std::filesystem::path& path, const std::string& text) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    stream << text;
    require(static_cast<bool>(stream), "write preference file");
}

gw::Wire running(const char* environment, const char* controller) {
    gw::Wire status = gw::Wire::object();
    gw::wire_set(status, "configured_follow_environment", environment);
    gw::wire_set(status, "configured_follow_controller", controller);
    return status;
}

bool runtime_rejected(const char* fc_environment, const gw::Wire& status) {
    gw::Wire state = gw::Wire::object();
    gw::wire_set(state, "flight_environment", fc_environment);
    try {
        gw::verify_runtime_environment(state, status);
    } catch (const gw::ValueError&) {
        return true;
    }
    return false;
}

bool save_rejected(gw::FollowPreferences& preferences, const char* key, const char* value) {
    try {
        gw::Wire change = gw::Wire::object();
        gw::wire_set(change, key, value);
        preferences.save(change);
    } catch (const gw::ValueError&) {
        return true;
    }
    return false;
}

void save_one(gw::FollowPreferences& preferences, const char* key, const char* value) {
    gw::Wire change = gw::Wire::object();
    gw::wire_set(change, key, value);
    preferences.save(change);
}

void demo_preference_test() {
    // 예전 두 키 파일은 indoor_controller=predictive, 결과는 세 키
    const gw::Wire legacy = gw::FollowPreferences::validate(preference("indoor", "predictive_low"));
    require(legacy.size() == 3 && legacy.at("indoor_controller") == gw::Wire("predictive"),
            "legacy 2-key means indoor predictive");
    require(!rejected(preference3("indoor", "predictive", "demo")), "3-key demo accepted");
    require(!rejected(preference3("outdoor", "predictive_m30", "predictive")), "3-key predictive");
    require(rejected(preference3("indoor", "predictive", "predictive_low")),
            "indoor outdoor-only controller rejected");
    require(rejected(preference3("indoor", "demo", "demo")), "outdoor demo rejected");
    gw::Wire extra = preference3("indoor", "predictive", "demo");
    gw::wire_set(extra, "other", 1);
    require(rejected(extra), "4 keys rejected");
    gw::Wire wrong = gw::Wire::object();
    gw::wire_set(wrong, "environment", "indoor");
    gw::wire_set(wrong, "indoor_controller", "demo");
    require(rejected(wrong), "missing outdoor_controller rejected");

    const auto directory = std::filesystem::temp_directory_path() / "argus_demo_pref_test";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    const auto path = directory / "follow-preferences.json";
    gw::FollowPreferences preferences(path);
    // 파일 없음: 환경 변수 기본값 (단일 스레드 시험)
    ::setenv("ARGUS_FOLLOW_ENVIRONMENT", "indoor", 1);         // NOLINT(concurrency-mt-unsafe)
    ::setenv("ARGUS_FOLLOW_CONTROLLER", "predictive_low", 1);  // NOLINT(concurrency-mt-unsafe)
    ::setenv("ARGUS_FOLLOW_INDOOR_CONTROLLER", "demo", 1);     // NOLINT(concurrency-mt-unsafe)
    require(preferences.read() == preference3("indoor", "predictive_low", "demo"),
            "env default with indoor controller");
    require(gw::demo_profile(preferences.selected()), "env demo selected");
    ::unsetenv("ARGUS_FOLLOW_INDOOR_CONTROLLER");  // NOLINT(concurrency-mt-unsafe)
    require(preferences.read().at("indoor_controller") == gw::Wire("predictive"),
            "env default indoor predictive");
    ::unsetenv("ARGUS_FOLLOW_ENVIRONMENT");  // NOLINT(concurrency-mt-unsafe)
    ::unsetenv("ARGUS_FOLLOW_CONTROLLER");   // NOLINT(concurrency-mt-unsafe)
    // 예전 두 키 파일
    write_text(path, R"({"environment": "indoor", "outdoor_controller": "predictive_low"})");
    gw::Wire selected = preferences.selected();
    require(selected.at("environment") == gw::Wire("indoor") &&
                selected.at("controller") == gw::Wire("predictive"),
            "legacy file selects indoor predictive");
    // 일부만 저장해도 세 키를 모두 쓴다
    save_one(preferences, "indoor_controller", "demo");
    gw::Wire written;
    {
        std::ifstream stream(path);
        written = gw::Wire::parse(stream);
    }
    require(written == preference3("indoor", "predictive_low", "demo"), "save writes 3 keys");
    selected = preferences.selected();
    require(selected.at("controller") == gw::Wire("demo") && gw::demo_profile(selected),
            "indoor demo selected");
    save_one(preferences, "environment", "outdoor");
    selected = preferences.selected();
    require(selected.at("environment") == gw::Wire("outdoor") &&
                selected.at("controller") == gw::Wire("predictive_low") &&
                !gw::demo_profile(selected),
            "outdoor uses outdoor controller");
    require(preferences.read().at("indoor_controller") == gw::Wire("demo"),
            "indoor choice kept while outdoor");
    {
        gw::Wire change = gw::Wire::object();
        gw::wire_set(change, "environment", "indoor");
        gw::wire_set(change, "outdoor_controller", "predictive");
        gw::wire_set(change, "indoor_controller", "predictive");
        preferences.save(change);
    }
    require(preferences.selected().at("controller") == gw::Wire("predictive"), "3-key save");
    require(save_rejected(preferences, "controller", "demo"), "unknown save key rejected");
    require(save_rejected(preferences, "indoor_controller", "predictive_low"),
            "bad indoor controller save rejected");
    require(save_rejected(preferences, "outdoor_controller", "demo"), "outdoor demo save rejected");
    // 실행 중 프로필: demo는 실내만
    const gw::Wire active = gw::active_profile(running("indoor", "demo"));
    require(!active.is_null() && active.at("controller") == gw::Wire("demo") &&
                gw::demo_profile(active),
            "running indoor demo recognised");
    require(gw::active_profile(running("outdoor", "demo")).is_null(), "outdoor demo unknown");
    require(!runtime_rejected("indoor", running("indoor", "demo")), "indoor demo starts");
    require(!runtime_rejected("indoor", running("indoor", "predictive")), "indoor predictive");
    require(runtime_rejected("indoor", running("indoor", "predictive_low")),
            "indoor outdoor-only controller rejected at start");
    require(runtime_rejected("outdoor", running("outdoor", "demo")), "outdoor demo rejected");
    require(runtime_rejected("outdoor", running("indoor", "demo")), "environment mismatch");
    require(!runtime_rejected("outdoor", running("outdoor", "predictive")), "outdoor predictive");
    // 고도 범위 판단 (저장 선택값 또는 실행 중 상태)
    require(!gw::demo_selected_or_active(preferences, std::nullopt), "predictive no band");
    require(gw::demo_selected_or_active(preferences, running("indoor", "demo")),
            "active demo band");
    require(!gw::demo_selected_or_active(preferences, running("indoor", "predictive")),
            "active predictive no band");
    save_one(preferences, "indoor_controller", "demo");
    require(gw::demo_selected_or_active(preferences, std::nullopt), "selected demo band");
    write_text(path, "{bad json");
    require(!gw::demo_selected_or_active(preferences, std::nullopt), "unreadable file no band");
    require(gw::demo_selected_or_active(preferences, running("indoor", "demo")),
            "unreadable file, active demo band");
    std::filesystem::remove_all(directory);
    std::cout << "PASS indoor demo preferences\n";
}

// 2026-11-19 실내 데모2(demo2): 저장, 실행 중 판별, 시작 환경 검사. 데모(회전만) 고도 대역은
// 아니다.
void demo2_preference_test() {
    require(!rejected(preference3("indoor", "predictive", "demo2")), "3-key demo2 accepted");
    require(rejected(preference3("indoor", "demo2", "predictive")), "outdoor demo2 rejected");
    require(rejected(preference3("indoor", "predictive", "demo3")), "unknown indoor rejected");
    const gw::Wire active = gw::active_profile(running("indoor", "demo2"));
    require(!active.is_null() && active.at("controller") == gw::Wire("demo2"),
            "running indoor demo2 recognised");
    require(!gw::demo_profile(active), "demo2 is not the rotation-only demo");
    require(gw::active_profile(running("outdoor", "demo2")).is_null(), "outdoor demo2 unknown");
    require(!runtime_rejected("indoor", running("indoor", "demo2")), "indoor demo2 starts");
    require(runtime_rejected("outdoor", running("outdoor", "demo2")), "outdoor demo2 rejected");
    require(runtime_rejected("outdoor", running("indoor", "demo2")), "demo2 environment mismatch");

    const auto directory = std::filesystem::temp_directory_path() / "argus_demo2_pref_test";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    gw::FollowPreferences preferences(directory / "follow-preferences.json");
    save_one(preferences, "environment", "indoor");
    save_one(preferences, "indoor_controller", "demo2");
    const gw::Wire selected = preferences.selected();
    require(selected.at("controller") == gw::Wire("demo2") && !gw::demo_profile(selected),
            "indoor demo2 selected");
    require(!gw::demo_selected_or_active(preferences, running("indoor", "demo2")),
            "demo2 has no rotation-only height band");
    std::filesystem::remove_all(directory);
    std::cout << "PASS indoor demo2 preferences\n";
}

void settings_test(const std::filesystem::path& root) {
    struct Case {
        const char* environment;
        const char* controller;
        const char* moving;
        const char* measurement;
        const char* camera;
    };
    for (const Case& item :
         {Case{"indoor", "predictive", "moving.config", "measurement.json", "camera.json"},
          Case{"outdoor", "predictive", "outdoor-predictive-moving.config",
               "measurement-outdoor.json", "camera.json"},
          Case{"outdoor", "predictive_low", "outdoor-low-predictive-moving.config",
               "measurement-outdoor.json", "camera.json"},
          Case{"outdoor", "predictive_m30", "outdoor-m30-predictive-moving.config",
               "measurement-outdoor-m30.json", "camera-mount30.json"},
          Case{"outdoor", "predictive_low_m30", "outdoor-low-m30-predictive-moving.config",
               "measurement-outdoor-m30.json", "camera-mount30.json"}}) {
        const gw::EnvironmentMap environment{{"ARGUS_FOLLOW_ENVIRONMENT", item.environment},
                                             {"ARGUS_FOLLOW_CONTROLLER", item.controller}};
        const auto settings = gw::load_deployment_settings(root, environment);
        const auto& paths = settings.at("paths");
        const auto name = [&paths](const char* key) {
            return std::filesystem::path(paths.at(key).get<std::string>()).filename().string();
        };
        require(name("moving") == item.moving, "moving profile path");
        require(name("measurement") == item.measurement, "measurement path");
        require(name("camera") == item.camera, "camera calibration path");
    }
    bool indoor_rejected = false;
    try {
        (void)gw::load_deployment_settings(root, {{"ARGUS_FOLLOW_ENVIRONMENT", "indoor"},
                                                  {"ARGUS_FOLLOW_CONTROLLER", "predictive_m30"}});
    } catch (const std::exception&) {
        indoor_rejected = true;
    }
    require(indoor_rejected, "indoor 30 degree mode rejected");
    // 2026-11-18 데모 모드: 실내 demo는 두 자리 모두 demo-yaw.config, 측정/카메라는 실내 기본
    {
        const auto settings = gw::load_deployment_settings(
            root, {{"ARGUS_FOLLOW_ENVIRONMENT", "indoor"}, {"ARGUS_FOLLOW_CONTROLLER", "demo"}});
        const auto& paths = settings.at("paths");
        const auto name = [&paths](const char* key) {
            return std::filesystem::path(paths.at(key).get<std::string>()).filename().string();
        };
        require(name("moving") == "demo-yaw.config", "demo moving slot");
        require(name("yaw") == "demo-yaw.config", "demo yaw slot");
        require(name("measurement") == "measurement.json", "demo measurement");
        require(name("camera") == "camera.json", "demo camera");
        require(settings.at("follow_controller") == "demo", "demo controller name");
        const auto indoor = gw::load_deployment_settings(
            root,
            {{"ARGUS_FOLLOW_ENVIRONMENT", "indoor"}, {"ARGUS_FOLLOW_CONTROLLER", "predictive"}});
        const auto& indoor_paths = indoor.at("paths");
        require(std::filesystem::path(indoor_paths.at("yaw").get<std::string>()).filename() ==
                    "yaw.config",
                "indoor predictive yaw unchanged");
    }
    // 2026-11-19 데모2: 이동 칸 demo2-moving.config, 회전 칸 demo2-yaw.config, 측정/카메라는 실내
    // 기본
    {
        const auto settings = gw::load_deployment_settings(
            root, {{"ARGUS_FOLLOW_ENVIRONMENT", "indoor"}, {"ARGUS_FOLLOW_CONTROLLER", "demo2"}});
        const auto& paths = settings.at("paths");
        const auto name = [&paths](const char* key) {
            return std::filesystem::path(paths.at(key).get<std::string>()).filename().string();
        };
        require(name("moving") == "demo2-moving.config", "demo2 moving slot");
        require(name("yaw") == "demo2-yaw.config", "demo2 yaw slot");
        require(name("measurement") == "measurement.json", "demo2 measurement");
        require(name("camera") == "camera.json", "demo2 camera");
        require(settings.at("follow_controller") == "demo2", "demo2 controller name");
        bool outdoor_demo2_rejected = false;
        try {
            (void)gw::load_deployment_settings(root, {{"ARGUS_FOLLOW_ENVIRONMENT", "outdoor"},
                                                      {"ARGUS_FOLLOW_CONTROLLER", "demo2"}});
        } catch (const std::exception&) {
            outdoor_demo2_rejected = true;
        }
        require(outdoor_demo2_rejected, "outdoor demo2 rejected");
    }
    bool outdoor_demo_rejected = false;
    try {
        (void)gw::load_deployment_settings(
            root, {{"ARGUS_FOLLOW_ENVIRONMENT", "outdoor"}, {"ARGUS_FOLLOW_CONTROLLER", "demo"}});
    } catch (const std::exception&) {
        outdoor_demo_rejected = true;
    }
    require(outdoor_demo_rejected, "outdoor demo rejected");
    std::cout << "PASS outdoor mode paths\n";
}
}  // namespace

int main(int argc, char** argv) {
    try {
        preference_test();
        demo_preference_test();
        demo2_preference_test();
        if (argc > 1) {
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic) C++17 main 인자
            settings_test(argv[1]);
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
