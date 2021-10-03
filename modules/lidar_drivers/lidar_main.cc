// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar main

#define MODULE "LidarDriver"

#include "common/common.h"
#include "cyber/cyber.h"
#include "lidar_drivers/output/cyber_output.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"
#include "lidar_drivers/lidar.h"

namespace crdc {
namespace airi {
namespace sensor_hub {
namespace lidar {

int main(int argc, char* argv[]) {
    // log dir string
    google::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = 0;
    FLAGS_v = 0;
    FLAGS_stderrthreshold = 3;

    apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
    apollo::cyber::Init(MODULE);
    common::Singleton<LidarCyberOutput>::get()->init(MODULE);

    LidarComponentConfig lidar_component_config;
    std::vector<std::shared_ptr<crdc::airi::sensor_hub::Lidar>> lidars;

    if (crdc::airi::util::is_path_exists(argv[1])) {
        LOG(FATAL) << "[LIDAR_MAIN] CRDC not exists, please setup environment first.";
    }

    if (crdc::airi::util::get_proto_from_file(argv[1], &lidar_component_config)) {
        LOG(FATAL) << "[LIDAR_MAIN] failed to read lidar config proto.";
    }
    
    std::cout << "[LIDAR_MAIN] lidar_driver terminated" << std::endl << std::flush;
    return 0;
}

}  // namespace lidar
}  // namespace sensor_hub
}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {return crdc::airi::sensor_hub::lidar::main(argc, argv); }