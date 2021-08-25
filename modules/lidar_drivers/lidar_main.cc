// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar main

#define MODULE "LidarDriver"

#include "common/common.h"

namespace crdc {
namespace airi {

int main(int argc, char* argv[]) {
    // log dir string
    // google::ParseCommandLineFlags(&argc, &argv, true);

    // FLAGS_colorlogtostderr = true;
    // FLAGS_miniloglevel = 0;
    // FLAGS_v = 0;
    // FLAGS_stderrthreshold = 3;

    // apollo::cyber::airi::GlobalData::Instance()->SetProcessGroup(MODULE);
    // apollo::cyber::Init(MODULE);

    // Singleton<CyberMessage>::get()->init(MODULE);

    std::cout << "[LIDAR_MAIN] lidar_driver terminated" << std::endl << std::flush;
    return 0;
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) {return crdc::airi::main(argc, argv); }