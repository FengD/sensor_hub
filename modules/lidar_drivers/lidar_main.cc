// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar main

#define MODULE "LidarDriver"

#include "common/common.h"
#include "cyber/cyber.h"
#include "lidar_drivers/cyber_message.h"
#include "lidar_drivers/proto/lidar_component_config.pb.h"

namespace crdc {
namespace airi {

int main(int argc, char* argv[]) {
    // log dir string
    google::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_colorlogtostderr = true;
    FLAGS_v = 0;
    FLAGS_stderrthreshold = 3;

    apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
    apollo::cyber::Init(MODULE);
    common::Singleton<LidarCyberMessage>::get()->init(MODULE);

    LidarComponentConfig lidar_component_config;
    
    std::cout << "[LIDAR_MAIN] lidar_driver terminated" << std::endl << std::flush;
    return 0;
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {return crdc::airi::main(argc, argv); }