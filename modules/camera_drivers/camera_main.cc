// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera main

#define MODULE "CameraDriver"

#include "common/common.h"
#include "camera_drivers/output/cyber_output.h"
#include "camera_drivers/camera.h"

namespace crdc {
namespace airi {
namespace camera {

int main(int argc, char* argv[]) {
    // log dir setting
    google::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = 0;
    FLAGS_v = 0;
    FLAGS_stderrthreshold = 3;
    FLAGS_alsologtostderr = true;

    apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
    apollo::cyber::Init(MODULE);
    common::Singleton<CameraCyberOutput>::get()->init(MODULE);

    CameraComponent camera_component;

    if (argc < 2) {
        LOG(FATAL) << "[CAMERA_MAIN] No camera proto file given." << std::endl;
        return 1;
    }

    if (!crdc::airi::util::is_path_exists(argv[1])) {
        LOG(FATAL) << "[CAMERA_MAIN] CRDC not exists, please setup environment first." << std::endl;
        return 1;
    }

    if (!crdc::airi::util::get_proto_from_file(argv[1], &camera_component)) {
        LOG(FATAL) << "[CAMERA_MAIN] failed to read camera config proto." << std::endl;
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
    apollo::cyber::WaitForShutdown();
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

int main(int argc, char* argv[]) {return crdc::airi::camera::main(argc, argv); }