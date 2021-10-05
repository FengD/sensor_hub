// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar factory

#include "lidar_drivers/lidar.h"
#include "lidar_drivers/input/socket/socket.h"
#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"

namespace crdc {
namespace airi {

REGISTER_LIDAR_INPUT(SocketInput);

REGISTER_LIDAR_PARSER(LidarParserVlp16);

}  // namespace airi
}  // namespace crdc

