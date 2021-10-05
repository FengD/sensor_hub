// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser

#pragma once

#include <memory>
#include "common/common.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "cyber/sensor_proto/lidar.pb.h"
#include "cyber/sensor_proto/eth_packet.pb.h"

namespace crdc {
namespace airi {

struct LidarPointCloud {
  std::shared_ptr<PointCloud> proto_cloud_;
  LidarPointCloud() {}
};

struct LidarPoint {
  LidarPoint() {}
};

class LidarParser {
 public:
  LidarParser() = default;
  virtual ~LidarParser() = default;
  /**
   * @brief Init the parser. It needs to be redefined for each subclass.
   * @param The lidar input config.
   * @return status
   */
  virtual bool init(const ParserConfig& config);

  /**
   * @brief Parser the lidar packet to cloud points
   * @param The tinpu raw ethernet packet 
   * @param The output cloud
   * @return status
   */
  virtual bool parse_lidar_packet(const Packet* raw_packet, std::shared_ptr<PointCloud>* cloud);
 
 protected:
  virtual bool init_pool();
  virtual bool init_lidar_parser();
  virtual bool init_calib();
  virtual bool is_frame_end(const uint64_t& azimuth);
  virtual bool is_lidar_packet_valid(const Packet* packet);
  virtual uint64_t get_timestamp(const Packet* packet);
  virtual void calculate_points_data_xyz(const Packet* packet, LidarPoint& pt);

  ParserConfig config_;
  int64_t time_zone_microseconds_;
  std::shared_ptr<common::CCObjectPool<LidarPointCloud>> cloud_pool_ = nullptr;
  uint64_t device_timestamp_;
};

REGISTER_COMPONENT(LidarParser);
#define REGISTER_LIDAR_PARSER(name) REGISTER_CLASS(LidarParser, name)

}  // namespace airi
}  // namespace crdc
