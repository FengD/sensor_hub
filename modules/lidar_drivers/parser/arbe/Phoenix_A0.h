// Copyright (C) 2021 FengD Inc.
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ins parser for arbe_Phoenix_A0 device

#pragma once
#include <memory>
#include <string>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

#define BIN_DATA_WITH_PHASE_SIZE_BYTES             12
#define BIN_DATA_NO_PHASE_SIZE_BYTES               10
#define BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES   7
#define BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES     6

#define UTILS_PC_IF_CFG_BITS_PER_TARGET_56        0x0
#define UTILS_PC_IF_CFG_BITS_PER_TARGET_48        0x1
#define UTILS_PC_IF_CFG_BITS_PER_TARGET_96        0x2
#define UTILS_PC_IF_CFG_BITS_PER_TARGET_80        0x3

#define EQUALIZATION_UNIT 12.04   // 40 log10(2)
#define POW_TO_DB_RATIO   0.1881  // (float)(3 / 16.0f)
#define MIN_DOPPLER       -100
#define MAX_DOPPLER       100

typedef struct PcRes {
  float range_resolution = 0.0;
  float doppler_resolution = 0.0;
  float azimuth_coefficient = 0.0;
  float elevation_coefficient = 0.0;
  PcRes() {}
}PcRes;

typedef struct PcMetadata {
  PcRes pc_resolution;
  float range_offset = 0.0;
  float range_res_in_db = 0.0;
  unsigned int azimuth_fft_size = 0;
  unsigned int elevation_fft_size = 0;
  unsigned int azimuth_fft_padding = 0;
  unsigned int elevation_fft_padding = 0;
  PcMetadata() {}
}PcMetadata;

typedef struct PcFrameHeader {
  uint64_t time = 0;
  uint16_t frame_counter = 0;
  uint8_t  is_4d = 0;
  uint8_t  frame_type = 0;
  uint16_t udp_lost_packages = 0;
  uint32_t number_of_points = 0;
  uint16_t crd_count = 0;
  PcFrameHeader() {}
}PcFrameHeader;

typedef struct PcFloatBins {
  float signed_doppler_bin;
  float signed_azimuth_bin;
  float signed_elevation_bin;
  float unsigned_phase_bin;
  float unsigned_range_bin;
  float unsigned_power_bin;
  float unsigned_doppler_bin;
  float unsigned_azimuth_bin;
  float unsigned_elevation_bin;
  PcFloatBins() {}
}PcFloatBins;

typedef struct TRangeFields {
  uint32_t fraction : 16;
  uint32_t integral : 10;
  uint32_t range_zoom : 6;
  TRangeFields() {}
}TRangeFields;

typedef struct TRangeCoef {
  union{
    TRangeFields tFields;
    uint32_t value;
  };
  TRangeCoef() {}
}TRangeCoef;

typedef struct TDopplerFields {
  uint32_t fraction : 16;
  uint32_t integral : 14;
  uint32_t isDynamic: 1;
  uint32_t isFine : 1;
  TDopplerFields() {}
}TDopplerFields;

typedef struct TDopplerCoef {
  union {
    TDopplerFields tFields;
    uint32_t value;
  };
  TDopplerCoef() {}
}TDopplerCoef;

typedef struct TAzimuthFields {
  uint32_t fraction : 16;
  uint32_t padding : 6;
  uint32_t fftRange : 10;
  TAzimuthFields() {}
}TAzimuthFields;

typedef struct TAzimuthCoef {
  union {
    TAzimuthFields tFields;
    uint32_t value;
  };
  TAzimuthCoef() {}
}TAzimuthCoef;

typedef struct TElevationFields {
  uint32_t fraction : 16;
  uint32_t padding : 6;
  uint32_t fftRange : 10;
  TElevationFields() {}
}TElevationFields;

typedef struct TElevationCoef {
  union {
    TElevationFields tFields;
    uint32_t value;
  };
  TElevationCoef() {}
}TElevationCoef;

typedef struct TPcMetadata {
  TRangeCoef range;
  TDopplerCoef doppler;
  TAzimuthCoef azimuth;
  TElevationCoef elevation;
  TPcMetadata() {}
}TPcMetadata;

typedef struct TPointCloud {
  uint16_t us_prefix;
  uint16_t us_type;
  uint32_t us_length;
  uint32_t us_timelsb;
  uint32_t us_timemsb;
  uint16_t us_frame_counter;
  uint16_t us_message_number;
  uint8_t us_last_packet;
  uint8_t us_target_fmtfrmtype;
  uint16_t us_crdcount;
  TPcMetadata us_metadata;
  uint16_t us_header_crc;
  uint16_t us_payload_crc;
  TPointCloud() {}
}TPointCloud;

typedef struct {
  union {
    uint8_t data[BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES];
    struct {
      uint64_t power : 10;
      uint64_t elevation : 6;
      uint64_t azimuth : 8;
      uint64_t doppler : 12;
      uint64_t range : 9;
      uint64_t dummy : 3;
    }Fields;
  };
}Target48bit;

typedef struct {
  union {
    uint8_t data[BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES];
    struct {
      uint64_t phase : 11;
      uint64_t power : 10;
      uint64_t elevation : 6;
      uint64_t azimuth : 8;
      uint64_t doppler : 12;
      uint64_t range : 9;
    }Fields;
  };
}Target56bit;

typedef struct {
  float signed_elevation_bin;
  float signed_azimuth_bin;
  float signed_doppler_bin;
  float unsigned_phase_bin;
  float unsigned_power_bin;
  float unsigned_range_bin;
  float unsigned_elevation_bin;
  float unsigned_azimuth_bin;
  float unsigned_doppler_bin;
}TargetGeneric;

class RadarParserPhoenixA0 : public LidarParser {
 public:
  RadarParserPhoenixA0();
  virtual ~RadarParserPhoenixA0();

  bool init_lidar_parser() override;

  bool init_calib() override;

  bool init_pool() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  void calibrate_point(LidarParserInfo& parser_info) override;

  bool parse_lidar_packet(const Packet* packet,
                                std::shared_ptr<LidarPointCloud>* cloud) override;

  bool is_frame_end(const TPointCloud* t_pointcloud);

  bool is_radar_packet_valid(const Packet* packet);

  bool set_point_cloud(std::shared_ptr<LidarPointCloud>* &cloud, bool &ret,
                                          double height, const Packet* packet);

  std::string get_name() const override {
    return "RadarParserPhoenixA0";
  }

 protected:
  inline bool update_frame(const uint64_t& packet_time) {
    if (!frame_init()) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to init frame.";
      return false;
    }
    cloud_->first_packet_utime_ = packet_time;
    return true;
  }

  inline bool frame_init() {
    cloud_ = cloud_pool_->GetObject();
    if (cloud_ == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] Failed to get cloud data.";
      return false;
    }
#ifdef WITH_ROS2
    const RadarPoint* temp = reinterpret_cast<const RadarPoint*>(cloud_->cloud_msg_->data.data());
    radar_point_ = const_cast<RadarPoint*>(temp);
    cloud_->cloud_msg_->data.resize(config_.max_points() * sizeof(RadarPoint));
#else
    const RadarPoint* temp = reinterpret_cast<const RadarPoint*>(
                                              cloud_->proto_cloud_->mutable_data()->data());
    radar_point_ = const_cast<RadarPoint*>(temp);
    cloud_->proto_cloud_->mutable_data()->resize(config_.max_points() * sizeof(RadarPoint));
#endif

    radar_point_count_ = 0;
    message_counter_ = 0;
    frame_valid_pts_ = valid_pts_;
    frame_invalid_pts_ = invalid_pts_;
    invalid_pts_ = 0;
    valid_pts_ = 0;
    return true;
  }

  inline bool is_invalid_point(RadarPoint& parser_info) {
    if (parser_info.doppler_ > max_doppler_ || parser_info.doppler_ < min_doppler_) {
      invalid_pts_++;
      return true;
    } else {
      valid_pts_++;
      return false;
    }
  }

  const uint8_t  RangeBinToDb[128] = {
    0x0,  0xC,  0x13, 0x18, 0x1C, 0x1F, 0x22, 0x24, 0x26, 0x28, 0x2A, 0x2B, 0x2D, 0x2E, 0x2F, 0x30,
    0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x36, 0x37, 0x38, 0x39, 0x39, 0x3A, 0x3A, 0x3B, 0x3C, 0x3C,
    0x3D, 0x3D, 0x3E, 0x3E, 0x3F, 0x3F, 0x40, 0x40, 0x41, 0x41, 0x41, 0x42, 0x42, 0x43, 0x43, 0x43,
    0x44, 0x44, 0x44, 0x45, 0x45, 0x45, 0x46, 0x46, 0x46, 0x47, 0x47, 0x47, 0x47, 0x48, 0x48, 0x48,
    0x49, 0x49, 0x49, 0x49, 0x4A, 0x4A, 0x4A, 0x4A, 0x4B, 0x4B, 0x4B, 0x4B, 0x4B, 0x4C, 0x4C, 0x4C,
    0x4C, 0x4D, 0x4D, 0x4D, 0x4D, 0x4D, 0x4E, 0x4E, 0x4E, 0x4E, 0x4E, 0x4F, 0x4F, 0x4F, 0x4F, 0x4F,
    0x4F, 0x50, 0x50, 0x50, 0x50, 0x50, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x51, 0x52, 0x52, 0x52,
    0x52, 0x52, 0x52, 0x53, 0x53, 0x53, 0x53, 0x53, 0x53, 0x53, 0x54, 0x54, 0x54, 0x54, 0x54, 0x54,
  };

 private:
  void get_pc_float_bins(PcFloatBins &pc_float_bins, TargetGeneric &target_generic, int point_size);
  void get_header_info(const TPointCloud* t_pointcloud, PcFrameHeader& pc_header);
  void get_metadata_info(const TPointCloud* t_pointcloud, PcMetadata& pc_metadata);
  uint8_t is_4d(const TDopplerCoef& doppler);
  uint8_t get_frame_number(const uint8_t& frametype);
  uint32_t calc_physical_rangezoom(const TRangeCoef& range);
  uint32_t get_point_size_of_target(const uint32_t& untarget_format);
  float calc_power(int power);
  float calc_range_coefficient(const TRangeCoef& range);
  float calc_doppler_coefficient(const TDopplerCoef& doppler);
  float calc_azimuth_coefficient(const TAzimuthCoef& azimuth);
  float calc_elevation_coefficient(const TElevationCoef& elevation);
  float range_offset(const TRangeCoef& range);
  float calc_range_res_in_db(const TPcMetadata& meta);
  float calc_doppler_signbin(int bin, const TDopplerCoef& doppler);
  float calc_azimuth_signbin(int bin, const TAzimuthCoef& azimuth);
  float calc_elevation_signbin(int bin, const TElevationCoef& elevation);
  float calc_power_equlizer(float power, int rangebin,
                               float range_res_in_db, float range_offset);
  TargetGeneric get_physical_bin(const uint8_t* data,
                            uint32_t number_of_bytes_target, const TPcMetadata* meta);
  RadarPoint set_radar_point(PcFloatBins &pc_float_bins, PcMetadata &pc_metadata,
                                                    PcFrameHeader &pc_header);

  bool is_frame_end_;
  uint8_t last_frame_flag_;
  uint16_t last_udp_counter_;
  uint32_t message_counter_;
  int32_t radar_point_count_;
  float min_doppler_;
  float max_doppler_;
  RadarPoint* radar_point_;
};

}  // namespace airi
}  // namespace crdc
