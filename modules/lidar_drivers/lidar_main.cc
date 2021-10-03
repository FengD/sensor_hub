// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar main

#define MODULE "LidarDriver"

#include "common/common.h"
#include "lidar_drivers/output/cyber_output.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#include "lidar_drivers/lidar.h"

namespace crdc {
namespace airi {
namespace lidar {

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
    common::Singleton<LidarCyberOutput>::get()->init(MODULE);

    LidarComponentConfig lidar_component_config;

    if (argc < 2) {
        LOG(FATAL) << "[LIDAR_MAIN] No proto file given." << std::endl;
        return 1;
    }

    if (!crdc::airi::util::is_path_exists(argv[1])) {
        LOG(FATAL) << "[LIDAR_MAIN] CRDC not exists, please setup environment first." << std::endl;
        return 1;
    }

    if (!crdc::airi::util::get_proto_from_file(argv[1], &lidar_component_config)) {
        LOG(FATAL) << "[LIDAR_MAIN] failed to read lidar config proto." << std::endl;
        return 1;
    }

    LOG(INFO) << lidar_component_config.DebugString();

    std::vector<std::shared_ptr<crdc::airi::Lidar>> lidars;
    for (const auto& config : lidar_component_config.component_config()) {
        std::shared_ptr<crdc::airi::Lidar> lidar = std::make_shared<crdc::airi::Lidar>(config);
        lidars.emplace_back(lidar);
    }
    std::cout << lidars.size() << std::endl;
    for (const auto& lidar : lidars) {
        lidar->start();
    }

    LOG(INFO) << "[LIDAR_MAIN] lidar_driver started.";
    apollo::cyber::WaitForShutdown();
    for (const auto& lidar : lidars) {
        lidar->stop();
    }

    for (const auto& lidar : lidars) {
        if (lidar->is_alive()) {
            lidar->join();
        }
        LOG(INFO) << "[LIDAR_MAIN] lidar[" << lidar->get_name() << "] joined successfully.";
    }

    LOG(WARNING) << "[LIDAR_MAIN] lidar_driver terminated.";
    return 0;
}

}  // namespace lidar
}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {return crdc::airi::lidar::main(argc, argv); }