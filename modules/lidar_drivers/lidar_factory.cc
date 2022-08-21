// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar factory

#include "lidar_drivers/lidar.h"
#include "lidar_drivers/input/socket/socket.h"
#include "lidar_drivers/input/cyber/cyber_input.h"
#include "lidar_drivers/parser/PandarXT32/pandar_xt32.h"

namespace crdc {
namespace airi {

REGISTER_LIDAR_INPUT(SocketInput);
REGISTER_LIDAR_INPUT(CyberInput);

REGISTER_LIDAR_PARSER(LidarParserPxt32);

}  // namespace airi
}  // namespace crdc

