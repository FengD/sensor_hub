// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING / JianFei JIANG
// Description: databag_message_type_convertor factory

#include "ins_drivers/ins.h"
#include "ins_drivers/parser/asensing_ins570d/asensing_ins570d.h"
#include "lidar_drivers/lidar.h"
#ifndef WITH_ROS2
#include "lidar_drivers/input/cyber/cyber_input.h"
#endif
#include "lidar_drivers/parser/pandar/pandar_xt32.h"
#include "lidar_drivers/parser/pandar/pandar_p40.h"
#include "lidar_drivers/parser/innoviz/innoviz_one.h"
#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"
#include "lidar_drivers/parser/robosense/robosense_ruby.h"
#include "lidar_drivers/parser/single/lanhai_lds50cs.h"
#include "lidar_drivers/parser/arbe/Phoenix_A0.h"

namespace crdc {
namespace airi {

REGISTER_LIDAR_PARSER(LidarParserPxt32);
REGISTER_LIDAR_PARSER(LidarParserP40);
REGISTER_LIDAR_PARSER(LidarParserInnovizone);
REGISTER_LIDAR_PARSER(LidarParserVlp16);
REGISTER_LIDAR_PARSER(LidarParserRSRuby);
REGISTER_LIDAR_PARSER(LidarParserLds50cs);
REGISTER_LIDAR_PARSER(RadarParserPhoenixA0);

REGISTER_INS_PARSER(InsParser570d);

}  // namespace airi
}  // namespace crdc

