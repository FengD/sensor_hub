// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar factory

#include "lidar_drivers/lidar.h"
#include "lidar_drivers/input/socket/socket.h"
#include "lidar_drivers/input/pcap/pcap_input.h"
#ifndef WITH_ROS2
#include "lidar_drivers/input/cyber/cyber_input.h"
#else
#include "lidar_drivers/input/ros2/ros2_input.h"
#endif
#include "lidar_drivers/parser/pandar/pandar_xt32.h"
#include "lidar_drivers/parser/pandar/pandar_p40.h"
#include "lidar_drivers/parser/innoviz/innoviz_one.h"
#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"
#include "lidar_drivers/parser/robosense/robosense_ruby.h"
#include "lidar_drivers/parser/single/lanhai_lds50cs.h"
#include "lidar_drivers/parser/arbe/Phoenix_A0.h"
#include "lidar_drivers/parser/falcon/falcon.h"

namespace sensor {
namespace hub {
#ifndef CALIBRATE
REGISTER_LIDAR_INPUT(SocketInput);
#endif
REGISTER_LIDAR_INPUT(PcapInput);
#ifndef WITH_ROS2
REGISTER_LIDAR_INPUT(CyberInput);
#else
REGISTER_LIDAR_INPUT(ROS2Input);
#endif

REGISTER_LIDAR_PARSER(LidarParserPxt32);
REGISTER_LIDAR_PARSER(LidarParserP40);
REGISTER_LIDAR_PARSER(LidarParserInnovizone);
REGISTER_LIDAR_PARSER(LidarParserVlp16);
REGISTER_LIDAR_PARSER(LidarParserRSRuby);
REGISTER_LIDAR_PARSER(LidarParserLds50cs);
REGISTER_LIDAR_PARSER(RadarParserPhoenixA0);
REGISTER_LIDAR_PARSER(LidarParserFalcon);

}  // namespace hub
}  // namespace sensor
