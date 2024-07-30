// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera main

#include <gflags/gflags.h>
#include "common/common.h"
#include "module_diagnose/module_diagnose.h"
#ifdef WITH_ROS2
#include "camera_drivers/output/ros_output.h"
#else
#include "camera_drivers/output/cyber_output.h"
#endif
#include "camera_drivers/camera.h"
#include "util/util.h"

#define MODULE "CameraDriver"
DEFINE_string(config_file, "params/drivers/camera/default/camera_config.prototxt",
              "path of config file");
DEFINE_string(tiovx_config, "TIOVX_CONFIG", "The tiovx config file path");
DEFINE_bool(use_product_name, true, "use product_name to splicing config_file");

namespace crdc {
namespace airi {
namespace camera {

int main(int argc, char* argv[]) {
    // gflags command setting
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (!std::getenv("CRDC_WS")) {
        LOG(FATAL) << "[CAMERA_MAIN] CRDC_WS not setting!";
    } else {
        LOG(INFO) << "[CAMERA_MAIN] Current CRDC_WS: " << std::string(std::getenv("CRDC_WS"));
    }

    #ifdef WITH_ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(MODULE);
    common::Singleton<CameraROSOutput>::get()->init(MODULE);
    #else
    apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
    apollo::cyber::Init(MODULE);
    common::Singleton<CameraCyberOutput>::get()->init(MODULE);
    #endif

    CameraComponent camera_component;
    std::string config = std::string(std::getenv("CRDC_WS")) + '/' + FLAGS_config_file;

    LOG(INFO) << "[CAMERA_MAIN] Use proto config: " << config;

    if (!crdc::airi::util::is_path_exists(config)) {
        LOG(FATAL) << "[CAMERA_MAIN] CRDC not exists, please setup environment first.";
        return 1;
    }

    if (!crdc::airi::util::get_proto_from_file(config, &camera_component)) {
        LOG(FATAL) << "[CAMERA_MAIN] failed to read camera config proto.";
        return 1;
    }

    LOG(INFO) << camera_component.DebugString();

    std::vector<std::shared_ptr<crdc::airi::Camera>> cameras;
    for (const auto& config : camera_component.component_config()) {
        std::shared_ptr<crdc::airi::Camera> camera = std::make_shared<crdc::airi::Camera>(config);
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

    #ifdef WITH_ROS2
    rclcpp::spin(node);
    rclcpp::shutdown();
    #else
    apollo::cyber::WaitForShutdown();
    #endif
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
}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) { return crdc::airi::camera::main(argc, argv); }
