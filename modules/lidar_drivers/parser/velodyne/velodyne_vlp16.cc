// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser vlp16

#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"

namespace crdc {
namespace airi {

bool LidarParserVlp16::init(const ParserConfig& config) {

}

bool LidarParserVlp16::parse_lidar_packet(const Packet* raw_packet, 
                                          std::shared_ptr<PointCloud>* cloud) {

}

}  // namespace airi
}  // namespace crdc