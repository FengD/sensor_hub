// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera main

#include <gflags/gflags.h>
#include "common/common.h"
#include "module_diagnose/module_diagnose.h"
#include "camera_drivers/output/cyber_output.h"
#include "camera_drivers/camera.h"
#include "common/util.h"

#define MODULE "CameraDriver"
DEFINE_string(config_file, "params/drivers/camera/default/camera_config.prototxt",
              "path of config file");
DEFINE_string(tiovx_config, "TIOVX_CONFIG", "The tiovx config file path");
DEFINE_bool(use_product_name, true, "use product_name to splicing config_file");

namespace sensor {
namespace hub {
namespace camera {

int main(int argc, char* argv[]) {
    // gflags command setting
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (!std::getenv("MAIN_WS")) {
        LOG(FATAL) << "[CAMERA_MAIN] MAIN_WS not setting!";
    } else {
        LOG(INFO) << "[CAMERA_MAIN] Current MAIN_WS: " << std::string(std::getenv("MAIN_WS"));
    }

    apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
    apollo::cyber::Init(MODULE);
    common::Singleton<CameraCyberOutput>::get()->init(MODULE);

    CameraComponent camera_component;
#ifdef WITH_TDA4
    std::string config;
    auto product_name = get_product_name();
    LOG(INFO) << "[" << MODULE << "] Product name is " << product_name;
    if (FLAGS_use_product_name) {
        config = std::string(std::getenv("MAIN_WS")) + "/params/drivers/camera/"+
                product_name + "/camera_config.prototxt";
        std::string tiovx_config = std::string(std::getenv("MAIN_WS"))
                + "/params/drivers/camera/" + product_name + "/app_multi_cam.cfg";
        set_env(FLAGS_tiovx_config, tiovx_config);
    } else {
        config = std::string(std::getenv("MAIN_WS")) + '/' + FLAGS_config_file;
    }
#else
    std::string config = std::string(std::getenv("MAIN_WS")) + '/' + FLAGS_config_file;
#endif
    LOG(INFO) << "[CAMERA_MAIN] Use proto config: " << config;

    if (!sensor::hub::util::is_path_exists(config)) {
        LOG(FATAL) << "[CAMERA_MAIN] MAIN not exists, please setup environment first.";
        return 1;
    }

    if (!sensor::hub::util::get_proto_from_file(config, &camera_component)) {
        LOG(FATAL) << "[CAMERA_MAIN] failed to read camera config proto.";
        return 1;
    }

    LOG(INFO) << camera_component.DebugString();

    std::vector<std::shared_ptr<sensor::hub::Camera>> cameras;
    for (const auto& config : camera_component.component_config()) {
        std::shared_ptr<sensor::hub::Camera> camera = std::make_shared<sensor::hub::Camera>(config);
        cameras.emplace_back(camera);
    }

    for (const auto& camera : cameras) {
        camera->start();
    }

    LOG(INFO) << "[CAMERA_MAIN] camera_driver started.";

    common::Singleton<ModuleDiagnose>::get()->init(MODULE, "CAMERA_DIAGNOSE");
    common::Singleton<ModuleDiagnose>::get()->set_period_usec(100000);
    common::Singleton<ModuleDiagnose>::get()->set_ready();
    common::Singleton<ModuleDiagnose>::get()->start();
    LOG(INFO) << "[CAMERA_MAIN] camera module_diagnose started.";

    apollo::cyber::WaitForShutdown();
    common::Singleton<ModuleDiagnose>::get()->stop();
    for (const auto& camera : cameras) {
        camera->stop();
    }

    for (const auto& camera : cameras) {
        if (camera->is_alive()) {
            camera->join();
        }
        LOG(INFO) << "[CAMERA_MAIN] camera[" << camera->get_name() << "] joined successfully.";
    }

    LOG(WARNING) << "[CAMERA_MAIN] camera_driver terminated.";
    return 0;
}

}  // namespace camera
}  // namespace hub
}  // namespace sensor

int main(int argc, char* argv[]) { return sensor::hub::camera::main(argc, argv); }
