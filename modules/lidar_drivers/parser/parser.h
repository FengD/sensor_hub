// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar parser

#pragma once

#include <memory>
#include <vector>
#include "common/common.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
#include "cyber/sensor_proto/lidar.pb.h"
#include "cyber/sensor_proto/eth_packet.pb.h"

namespace crdc {
namespace airi {

#define ADD_FIELD(cloud, name, datatype, offset, count)   \
  f = cloud->add_fields();                                \
  f->set_name(name);                                      \
  f->set_datatype(datatype);                              \
  f->set_offset(offset);                                  \
  f->set_count(count);

struct LidarPointCloud {
  uint64_t first_packet_utime_;
  uint64_t last_packet_utime_;
  std::shared_ptr<PointCloud2> proto_cloud_;
  LidarPointCloud() {}
};

struct LidarPoint {
  float x_ = 0;
  float y_ = 0;
  float z_ = 0;
  uint8_t intensity_ = 0;
  float distance = 0;
  uint64_t timestamp_ = 0;
  uint16_t ring_ = 0;
  float azimuth_ = 0;
  float elevation_ = 0;
  uint8_t semantic_flag_ = 0;
  LidarPoint() {}
};

struct LidarParserInfo {
  uint64_t packet_timestamp_ = 0;
  uint16_t pixel_azimuth_diff_ = 0;
  uint16_t start_of_block_ = 0;
  uint16_t pixel_azimuth_ = 0;
  uint16_t pixel_elevation_ = 0;
  uint16_t pixel_distance_ = 0;
  uint16_t pixel_intensity_ = 0;
  int32_t block_ = 0;
  int32_t firing_ = 0;
  int32_t laser_ = 0;
  float distance_ = 0;
  float azimuth_ = 0;
  float elevation_ = 0;
  uint64_t timestamp_ = 0;
  uint8_t intensity_ = 0;
  LidarParserInfo() {}
};

struct LidarCalibInfo {
  float calib_azimuth_ = 0;
  int32_t offset_x_ = 0;
  float elevation_cos_ = 0;
  float elevation_sin_ = 0;
  LidarCalibInfo() {}
};

static const float ROTATION_RESOLUTION = 0.01f;
static const uint16_t ROTATION_MAX_UNITS = 36000;
static const uint32_t AZIMUTH_SCALE = 100;
static const uint32_t AZIMUTH_OFFSET = 36000;
static const double RAD_PER_DEGREE = 0.0174532922;
static const float MAX_DISTANCE = 200.0f;
static const float MIN_DISTANCE = 0.2f;

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
  virtual bool parse_lidar_packet(const Packet* packet, std::shared_ptr<LidarPointCloud>* cloud);

 protected:
  /**
   * @brief Parser the lidar packet timestamp
   * @param The tinpu raw ethernet packet
   * @return microseconds timestamp
   */
  uint64_t get_timestamp(const Packet* packet);

  /**
   * @brief Init lidar parser
   * @return status
   */
  virtual bool init_lidar_parser() { return false; }

  /**
   * @brief Check if lidar packet valid
   * @param The input raw ethernet packet
   * @return status
   */
  virtual bool is_lidar_packet_valid(const Packet* packet);

  /**
   * @brief If there is some other info in the packet
   * @param The input raw ethernet packet
   */
  virtual void parse_lidar_other_info(const Packet* packet) {}

  /**
   * @brief parse the block info, need to be redefined
   * @param The input raw ethernet packet data
   * @param lidar parser info
   */
  virtual void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) = 0;

  /**
   * @brief parse the point info, need to be redefined
   * @param The input raw ethernet packet
   * @param lidar parser info
   */
  virtual void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) = 0;

  /**
   * @brief parse the timestamp in the packets, need to be redefined
   * @param The input raw ethernet packet
   * @param timestamp
   */
  virtual uint64_t get_packet_timestamp(const Packet* packet) = 0;

  /**
   * @brief calibration the point info, need to be redefined
   * @param lidar parser info
   */
  virtual void calibrate_point(LidarParserInfo& parser_info) = 0;

  /**
   * @brief Init cloud pool
   * @return status
   */
  virtual bool init_pool();

  /**
   * @brief Init the calibration lists
   * @return status
   */
  virtual bool init_calib();

  /**
   * @brief check if the frame is end
   * @param azimuth in pixel
   * @return status
   */
  virtual bool is_frame_end(const uint64_t& azimuth);

  /**
   * @brief calculate the points data xyz
   * @param lidar parser info
   * @param temp point
   */
  virtual void calculate_points_data_xyz(const LidarParserInfo& parser_info, LidarPoint& pt);

  /**
   * @brief init the frame in each loop
   * @return status
   */
  inline bool frame_init() {
    cloud_ = cloud_pool_->GetObject();
    if (cloud_ == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to get cloud data.";
      return false;
    }
    lidar_point_count_ = 0;
    lidar_point_ = (struct LidarPoint*)cloud_->proto_cloud_->mutable_data()->data();
    cloud_->proto_cloud_->mutable_data()->resize(config_.max_points() * sizeof(LidarPoint));
    frame_valid_pts_ = valid_pts_;
    frame_invalid_pts_ = invalid_pts_;
    invalid_pts_ = 0;
    valid_pts_ = 0;
    max_azimuth_gap_ = 0;
    return true;
  }

  /**
   * @brief check if the point is invalid
   * @param lidar parser info
   * @return status
   */
  inline bool is_invalid_point(const LidarParserInfo& parser_info) {
    if (parser_info.distance_ > MAX_DISTANCE || parser_info.distance_ < MIN_DISTANCE) {
      invalid_pts_++;
      return true;
    } else {
      valid_pts_++;
      return false;
    }
  }

  /**
   * @brief calculate the points position in the data
   */
  inline void calculate_lidar_point_pos() {
    auto& packet_config = config_.lidar_packet_config();
    for (auto laser = 0; laser < packet_config.lasers(); ++laser) {
      auto ring = packet_config.ring_map(laser);
      for (auto x = 0; x < calib_info_[laser].offset_x_; x += packet_config.lasers()) {
        int pos = x + ring;
        lidar_point_[pos] = lidar_point_[pos + lidar_point_count_];
      }
    }
    lidar_point_[0].timestamp_ = frame_start_utime_;
    lidar_point_[lidar_point_count_ - 1].timestamp_ = frame_end_utime_;
  }

  /**
   * @brief update the frame
   * @param packet timestamp
   * @param lidar parser info
   * @return status
   */
  inline bool update_frame(const uint64_t& packet_time, LidarParserInfo& parser_info) {
    if (!frame_init()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to init frame.";
      return false;
    }

    parser_info.firing_ = 0;
    parser_info.laser_ = 0;
    calibrate_point(parser_info);
    frame_start_utime_ = parser_info.timestamp_;
    cloud_->first_packet_utime_ = packet_time;
    return true;
  }

  ParserConfig config_;
  uint16_t last_pixel_azimuth_;
  uint16_t max_azimuth_gap_;
  int64_t time_zone_microseconds_;
  int32_t packet_points_;

  float distance_resolution_ = 0.005f;
  bool is_init_distance_resolution_ = false;
  bool is_init_calib_elevation_ = false;
  bool is_init_calib_azimuth_ = false;

  struct LidarPoint* lidar_point_;
  std::shared_ptr<common::CCObjectPool<LidarPointCloud>> cloud_pool_ = nullptr;
  std::shared_ptr<LidarPointCloud> cloud_;
  uint64_t device_timestamp_;
  uint64_t frame_start_utime_;
  uint64_t frame_end_utime_;
  int32_t valid_pts_;
  int32_t invalid_pts_;
  int32_t frame_valid_pts_;
  int32_t frame_invalid_pts_;
  uint32_t frame_seq_;
  int32_t lidar_point_count_;
  std::vector<LidarCalibInfo> calib_info_;
};

REGISTER_COMPONENT(LidarParser);
#define REGISTER_LIDAR_PARSER(name) REGISTER_CLASS(LidarParser, name)

}  // namespace airi
}  // namespace crdc
