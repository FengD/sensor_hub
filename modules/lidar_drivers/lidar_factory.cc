// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar factory

#include "lidar_drivers/lidar.h"
#include "lidar_drivers/input/input.h"
#include "lidar_drivers/input/socket/socket.h"

namespace crdc {
namespace airi {

REGISTER_LIDAR_INPUT(SocketInput);

}  // namespace airi
}  // namespace crdc

