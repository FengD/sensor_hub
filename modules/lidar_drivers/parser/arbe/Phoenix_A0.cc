// Copyright (C) 2021 FengD Inc.
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: radar parser for arbe_Phoenix_A0 device

#include <vector>
#include "lidar_drivers/parser/arbe/Phoenix_A0.h"
#include "lidar_drivers/proto/lidar_config.pb.h"

namespace crdc {
namespace airi {
RadarParserPhoenixA0::RadarParserPhoenixA0() {
  message_counter_ = 0;
  min_doppler_ = MIN_DOPPLER;
  max_doppler_ = MAX_DOPPLER;
  last_udp_counter_ = 0;
  last_frame_flag_ = 0;
  is_frame_end_ = 0;
}

RadarParserPhoenixA0::~RadarParserPhoenixA0() { }

bool RadarParserPhoenixA0::init_lidar_parser() { return true; }

bool RadarParserPhoenixA0::init_calib() { return true; }

bool RadarParserPhoenixA0::init_pool() {
  cloud_pool_.reset(new common::CCObjectPool<LidarPointCloud>(config_.pool_size()));
  cloud_pool_->ConstructAll();
  std::vector<std::shared_ptr<LidarPointCloud>> temp_clouds;
#ifdef WITH_ROS2
  sensor_msgs::msg::PointField f;
#endif
  for (auto i = 0; i < config_.pool_size(); ++i) {
    auto cloud = cloud_pool_->GetObject();
    if (cloud == nullptr) {
      LOG(ERROR) << "[" << config_.frame_id() << "] failed to getobject: " << i;
      return false;
    }

#ifdef WITH_ROS2
    cloud->cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->cloud_msg_->fields.clear();
    add_field(f, "sec", sensor_msgs::msg::PointField::UINT32, 0, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "usec", sensor_msgs::msg::PointField::UINT32, 4, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "x", sensor_msgs::msg::PointField::FLOAT32, 8, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "y", sensor_msgs::msg::PointField::FLOAT32, 12, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "z", sensor_msgs::msg::PointField::FLOAT32, 16, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "doppler", sensor_msgs::msg::PointField::FLOAT32, 20, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "range", sensor_msgs::msg::PointField::FLOAT32, 24, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "snr", sensor_msgs::msg::PointField::FLOAT32, 28, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "power", sensor_msgs::msg::PointField::FLOAT32, 32, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "azimuth", sensor_msgs::msg::PointField::FLOAT32, 36, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "elevation", sensor_msg::msg::PointField::FLOAT32, 40, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "elevation_bin", sensor_msg::msg::PointField::FLOAT32, 44, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "azimuth_bin", sensor_msg::msg::PointField::FLOAT32, 48, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "doppler_bin", sensor_msg::msg::PointField::FLOAT32, 52, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "range_bin", sensor_msg::msg::PointField::FLOAT32, 56, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    add_field(f, "power_bin", sensor_msg::msg::PointField::FLOAT32, 60, 1);
    cloud->cloud_msg_->fields.emplace_back(f);
    cloud->cloud_msg_->data.resize(config_.max_points() * sizeof(RadarPoint));
    cloud->cloud_msg_->point_step = sizeof(RadarPoint);
    cloud->cloud_msg_->height = config_.lidar_packet_config().lasers();
    cloud->cloud_msg_->is_dense = true;
    cloud->cloud_msg_->header.frame_id = config_.frame_id();
    cloud->cloud_msg_->is_bigendian = false;
#else
    cloud->proto_cloud_ = std::make_shared<PointCloud2>();
    cloud->proto_cloud_->clear_fields();
    PointField* f;
    ADD_FIELD(cloud->proto_cloud_, "sec", PointField::UINT32, 0, 1);
    ADD_FIELD(cloud->proto_cloud_, "usec", PointField::UINT32, 4, 1);
    ADD_FIELD(cloud->proto_cloud_, "x", PointField::FLOAT32, 8, 1);
    ADD_FIELD(cloud->proto_cloud_, "y", PointField::FLOAT32, 12, 1);
    ADD_FIELD(cloud->proto_cloud_, "z", PointField::FLOAT32, 16, 1);
    ADD_FIELD(cloud->proto_cloud_, "doppler", PointField::FLOAT32, 20, 1);
    ADD_FIELD(cloud->proto_cloud_, "range", PointField::FLOAT32, 24, 1);
    ADD_FIELD(cloud->proto_cloud_, "snr", PointField::FLOAT32, 28, 1);
    ADD_FIELD(cloud->proto_cloud_, "power", PointField::FLOAT32, 32, 1);
    ADD_FIELD(cloud->proto_cloud_, "azimuth", PointField::FLOAT32, 36, 1);
    ADD_FIELD(cloud->proto_cloud_, "elevation", PointField::FLOAT32, 40, 1);
    ADD_FIELD(cloud->proto_cloud_, "elevation_bin", PointField::FLOAT32, 44, 1);
    ADD_FIELD(cloud->proto_cloud_, "azimuth_bin", PointField::FLOAT32, 48, 1);
    ADD_FIELD(cloud->proto_cloud_, "doppler_bin", PointField::FLOAT32, 52, 1);
    ADD_FIELD(cloud->proto_cloud_, "range_bin", PointField::FLOAT32, 56, 1);
    ADD_FIELD(cloud->proto_cloud_, "power_bin", PointField::FLOAT32, 60, 1);
    cloud->proto_cloud_->mutable_data()->resize(config_.max_points() * sizeof(RadarPoint));
    cloud->proto_cloud_->set_point_step(sizeof(RadarPoint));
    cloud->proto_cloud_->set_height(config_.lidar_packet_config().lasers());
    cloud->proto_cloud_->set_is_dense(true);
    cloud->proto_cloud_->mutable_header()->set_frame_id(config_.frame_id());
    cloud->proto_cloud_->set_is_bigendian(false);
#endif
    temp_clouds.emplace_back(cloud);
  }
  temp_clouds.clear();
  return true;
}

void RadarParserPhoenixA0::get_block_info(const uint8_t *data, LidarParserInfo &parser_info) {
  (void)data;
  (void)parser_info;
}

void RadarParserPhoenixA0::get_point_raw_info(const uint8_t *data, LidarParserInfo &parser_info) {
  (void)data;
  (void)parser_info;
}

void RadarParserPhoenixA0::calibrate_point(LidarParserInfo &parser_info) {
  (void)parser_info;
}

uint64_t RadarParserPhoenixA0::get_packet_timestamp(const Packet *packet) {
#ifdef WITH_ROS2
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data.data());
#else
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data().data());
#endif
  const TPointCloud* t_pointcloud = reinterpret_cast<const TPointCloud*>(data);

  uint64_t timestamp = (((static_cast<uint64_t>(t_pointcloud->us_timemsb) << 32)) |
                    static_cast<uint64_t>(t_pointcloud->us_timelsb)) >> 16;

  return timestamp;
}

bool RadarParserPhoenixA0::is_radar_packet_valid(const Packet* packet) {
#ifdef WITH_ROS2
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data.data());
#else
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data().data());
#endif
  const TPointCloud* t_pointcloud = reinterpret_cast<const TPointCloud*>(data);
  uint16_t header_checksum = (uint16_t)t_pointcloud->us_prefix;
  if (header_checksum != config_.lidar_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.lidar_packet_config().check_sum();
    return false;
  }

  uint16_t us_type = (uint16_t)t_pointcloud->us_type;
  if (us_type != config_.lidar_packet_config().data_type()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] data_type " << us_type
               << " wrong " << config_.lidar_packet_config().data_type();
    return false;
  }

  uint32_t point_size = get_point_size_of_target(t_pointcloud->us_target_fmtfrmtype);
  if (point_size != config_.lidar_packet_config().point_size()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] point_size " << point_size
               << " wrong " << config_.lidar_packet_config().point_size();
    return false;
  }

  if (sizeof(TPointCloud) != config_.lidar_packet_config().block_offset()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] block_offset " << sizeof(TPointCloud)
               << " wrong " << config_.lidar_packet_config().block_offset();
    return false;
  }

  return true;
}

uint8_t RadarParserPhoenixA0::is_4d(const TDopplerCoef& doppler) {
  return doppler.tFields.isFine;
}

uint8_t RadarParserPhoenixA0::get_frame_number(const uint8_t& frametype) {
  return frametype & 0x3f;
}

float RadarParserPhoenixA0::calc_range_coefficient(const TRangeCoef& range) {
  uint16_t complete = range.tFields.integral;
  uint16_t fraction = range.tFields.fraction;
  return complete + fraction / static_cast<float>(1 << 16);
}

float RadarParserPhoenixA0::calc_doppler_coefficient(const TDopplerCoef& doppler) {
  return doppler.tFields.integral + (doppler.tFields.fraction / static_cast<float>(1 << 16));
}

float RadarParserPhoenixA0::calc_azimuth_coefficient(const TAzimuthCoef& azimuth) {
  return azimuth.tFields.fraction / static_cast<float>(1 << 16);
}

float RadarParserPhoenixA0::calc_elevation_coefficient(const TElevationCoef& elevation) {
  return elevation.tFields.fraction / static_cast<float>(1 << 16);
}

uint32_t RadarParserPhoenixA0::calc_physical_rangezoom(const TRangeCoef& range) {
  switch (range.tFields.range_zoom) {
    case 0x0: return 1;
    case 0x1: return 2;
    case 0x3: return 4;
    case 0x7: return 8;
    case 0xF: return 16;
    default: return 0;
  }
}

float RadarParserPhoenixA0::range_offset(const TRangeCoef& range) {
  uint32_t zoom_factor = calc_physical_rangezoom(range);
  return ((static_cast<float>(zoom_factor) - 1) / 2);
}

float RadarParserPhoenixA0::calc_range_res_in_db(const TPcMetadata& meta) {
  uint16_t complete = meta.range.tFields.integral;
  uint16_t fraction = meta.range.tFields.fraction;
  float coef = complete + fraction / static_cast<float>(1 << 16);
  return 40 * log10(coef);
}

uint32_t RadarParserPhoenixA0::get_point_size_of_target(const uint32_t& untarget_format) {
  uint32_t bytesPerTarget = 0;
  switch (((untarget_format >> 6) & 0x3)) {
    case UTILS_PC_IF_CFG_BITS_PER_TARGET_56: {
      bytesPerTarget = BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES; break;
    }
    case UTILS_PC_IF_CFG_BITS_PER_TARGET_48: {
      bytesPerTarget = BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES; break;
    }
    case UTILS_PC_IF_CFG_BITS_PER_TARGET_96: {
      bytesPerTarget = BIN_DATA_WITH_PHASE_SIZE_BYTES; break;
    }
    case UTILS_PC_IF_CFG_BITS_PER_TARGET_80: {
      bytesPerTarget = BIN_DATA_NO_PHASE_SIZE_BYTES; break;
    }
    default: { break; }
  }
  return bytesPerTarget;
}

float RadarParserPhoenixA0::calc_azimuth_signbin(int bin, const TAzimuthCoef& azimuth) {
  return (-1) * (bin - (azimuth.tFields.fftRange / 2) + azimuth.tFields.padding);
}

float RadarParserPhoenixA0::calc_doppler_signbin(int bin, const TDopplerCoef& doppler) {
  float dopplerbin;
  if (doppler.tFields.isFine) {
    dopplerbin = static_cast<float>((bin + 0x800) % 0x1000);
    dopplerbin -= 0x800;
  } else {
    dopplerbin = static_cast<float>((bin + 0x200) % 0x400);
    dopplerbin -= 0x200;
  }
  return (-1) * dopplerbin;
}

float RadarParserPhoenixA0::calc_elevation_signbin(int bin, const TElevationCoef& elevation) {
  return (-1) * (bin - (elevation.tFields.fftRange) / 2 + elevation.tFields.padding);
}

TargetGeneric RadarParserPhoenixA0::get_physical_bin(const uint8_t* data,
                                    uint32_t number_of_bytes_target, const TPcMetadata* meta) {
  TargetGeneric target_generic = {0, 0, 0, 0, 0, 0, 0, 0, 0 };
  if (number_of_bytes_target == BIN_DATA_COMPRESSED_NO_PHASE_SIZE_BYTES) {
    const Target48bit* target48bit = reinterpret_cast<const Target48bit*>(data);

    target_generic.signed_azimuth_bin =
           calc_azimuth_signbin(static_cast<float>(target48bit->Fields.azimuth), meta->azimuth);
    target_generic.signed_doppler_bin =
           calc_doppler_signbin(static_cast<float>(target48bit->Fields.doppler), meta->doppler);
    target_generic.signed_elevation_bin =
           calc_elevation_signbin(static_cast<float>(target48bit->Fields.elevation),
                                  meta->elevation);
    target_generic.unsigned_phase_bin = 0;
    target_generic.unsigned_power_bin = static_cast<float>(target48bit->Fields.power);
    target_generic.unsigned_range_bin = static_cast<float>(target48bit->Fields.range);
    target_generic.unsigned_azimuth_bin = static_cast<float>(target48bit->Fields.azimuth);
    target_generic.unsigned_elevation_bin = static_cast<float>(target48bit->Fields.elevation);
    target_generic.unsigned_doppler_bin = static_cast<float>(target48bit->Fields.doppler);
  } else if (number_of_bytes_target == BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES) {
    const Target56bit* target56bit = reinterpret_cast<const Target56bit*>(data);
    target_generic.signed_azimuth_bin =
          calc_azimuth_signbin(static_cast<float>(target56bit->Fields.azimuth), meta->azimuth);
    target_generic.signed_doppler_bin =
          calc_doppler_signbin(static_cast<float>(target56bit->Fields.doppler), meta->doppler);
    target_generic.signed_elevation_bin =
          calc_elevation_signbin(static_cast<float>(target56bit->Fields.elevation),
                                 meta->elevation);
    target_generic.unsigned_phase_bin = static_cast<float>(target56bit->Fields.phase);
    target_generic.unsigned_power_bin = static_cast<float>(target56bit->Fields.power);
    target_generic.unsigned_range_bin = static_cast<float>(target56bit->Fields.range);
    target_generic.unsigned_azimuth_bin = static_cast<float>(target56bit->Fields.azimuth);
    target_generic.unsigned_elevation_bin = static_cast<float>(target56bit->Fields.elevation);
    target_generic.unsigned_doppler_bin = static_cast<float>(target56bit->Fields.doppler);
  } else {
    LOG(ERROR) << "rawdata is not 48bit or 56bit.";
  }
  return target_generic;
}

float RadarParserPhoenixA0::calc_power(int power) {
  return static_cast<float>(power * POW_TO_DB_RATIO);
}

float RadarParserPhoenixA0::calc_power_equlizer(float power, int rangebin,
                                                   float range_res_in_db, float range_offset) {
  int zoom = static_cast<int>(2 * range_offset + 1);
  // This should not happen but causes a crash if a bad frame is received
  if ((rangebin / zoom) > 128) return 0;
  float eq_p = power + range_res_in_db + RangeBinToDb[static_cast<int>(rangebin / zoom)];
  switch (zoom) {
    case 2:
      eq_p += EQUALIZATION_UNIT;
      break;
    case 4:
      eq_p += 2 * EQUALIZATION_UNIT;
      break;
    case 8:
      eq_p += 3 * EQUALIZATION_UNIT;
      break;
    default:  // 3d mode or no zoom
      break;
  }
  return eq_p;
}

void RadarParserPhoenixA0::get_header_info(const TPointCloud* t_pointcloud,
                                                    PcFrameHeader& pc_header) {
  pc_header.time = (((static_cast<uint64_t>(t_pointcloud->us_timemsb) << 32) |
                    static_cast<uint64_t>(t_pointcloud->us_timelsb)) >> 16) * 1000;
  pc_header.frame_counter = t_pointcloud->us_frame_counter;
  pc_header.crd_count = t_pointcloud->us_crdcount;
  pc_header.is_4d = is_4d(t_pointcloud->us_metadata.doppler);
  pc_header.frame_type = get_frame_number(t_pointcloud->us_target_fmtfrmtype);
  pc_header.number_of_points = (t_pointcloud->us_length - sizeof(TPointCloud)) /
                              get_point_size_of_target(t_pointcloud->us_target_fmtfrmtype);
  if (message_counter_ != t_pointcloud->us_message_number) {
    pc_header.udp_lost_packages += (t_pointcloud->us_message_number - message_counter_);
    message_counter_ = t_pointcloud->us_message_number;
  }
  message_counter_++;
}

void RadarParserPhoenixA0::get_metadata_info(const TPointCloud* t_pointcloud,
                                                    PcMetadata& pcmetadata) {
  pcmetadata.pc_resolution.range_resolution =
                          calc_range_coefficient(t_pointcloud->us_metadata.range);
  pcmetadata.pc_resolution.doppler_resolution =
                          calc_doppler_coefficient(t_pointcloud->us_metadata.doppler);
  pcmetadata.pc_resolution.azimuth_coefficient =
                          calc_azimuth_coefficient(t_pointcloud->us_metadata.azimuth);
  pcmetadata.pc_resolution.elevation_coefficient =
                          calc_elevation_coefficient(t_pointcloud->us_metadata.elevation);
  pcmetadata.range_offset = range_offset(t_pointcloud->us_metadata.range);
  pcmetadata.range_res_in_db = calc_range_res_in_db(t_pointcloud->us_metadata);
  pcmetadata.azimuth_fft_size = t_pointcloud->us_metadata.azimuth.tFields.fftRange;
  pcmetadata.elevation_fft_size = t_pointcloud->us_metadata.elevation.tFields.fftRange;
  pcmetadata.azimuth_fft_padding = t_pointcloud->us_metadata.azimuth.tFields.padding;
  pcmetadata.elevation_fft_padding = t_pointcloud->us_metadata.elevation.tFields.padding;
}

bool RadarParserPhoenixA0::is_frame_end(const TPointCloud* t_pointcloud) {
  if (t_pointcloud->us_last_packet == 1 ||
     (last_udp_counter_ > t_pointcloud->us_message_number && last_frame_flag_ == 0)) {
    return true;
  }
  return false;
}

void RadarParserPhoenixA0::get_pc_float_bins(PcFloatBins &pc_float_bins,
                                          TargetGeneric &target_generic, int point_size) {
  pc_float_bins.signed_azimuth_bin = target_generic.signed_azimuth_bin;
  pc_float_bins.signed_doppler_bin = target_generic.signed_doppler_bin;
  pc_float_bins.signed_elevation_bin = target_generic.signed_elevation_bin;
  pc_float_bins.unsigned_range_bin = target_generic.unsigned_range_bin;
  pc_float_bins.unsigned_power_bin = target_generic.unsigned_power_bin;
  pc_float_bins.unsigned_doppler_bin = target_generic.unsigned_doppler_bin;
  pc_float_bins.unsigned_azimuth_bin = target_generic.unsigned_azimuth_bin;
  pc_float_bins.unsigned_elevation_bin = target_generic.unsigned_elevation_bin;

  if (point_size == BIN_DATA_WITH_PHASE_SIZE_BYTES ||
      point_size == BIN_DATA_COMPRESSED_WITH_PHASE_SIZE_BYTES) {
    pc_float_bins.unsigned_phase_bin = target_generic.unsigned_phase_bin;
  } else {
    pc_float_bins.unsigned_phase_bin = 0;
  }
}

RadarPoint RadarParserPhoenixA0::set_radar_point(PcFloatBins &pc_float_bins, PcMetadata &pcmetadata,
                                                                    PcFrameHeader &pc_header) {
  RadarPoint pt;
  pt.range_ = (pc_float_bins.unsigned_range_bin - pcmetadata.range_offset) *
                                        pcmetadata.pc_resolution.range_resolution;
  pt.doppler_ = pc_float_bins.signed_doppler_bin * pcmetadata.pc_resolution.doppler_resolution;
  pt.elevation_ = pc_float_bins.signed_elevation_bin *
                                    pcmetadata.pc_resolution.elevation_coefficient;
  pt.azimuth_ = pc_float_bins.signed_azimuth_bin * pcmetadata.pc_resolution.azimuth_coefficient;
  pt.snr_ = calc_power(pc_float_bins.unsigned_power_bin);
  pt.power_ = calc_power_equlizer(pt.snr_, static_cast<int>(pc_float_bins.unsigned_range_bin),
                              40 * log(pcmetadata.pc_resolution.range_resolution) / log(10),
                              pcmetadata.range_offset);
  pt.x_ = pt.range_ * pt.azimuth_;
  pt.z_ = pt.range_ * pt.elevation_;
  pt.y_ = sqrt(pt.range_ * pt.range_ - pt.x_ * pt.x_ - pt.z_ * pt.z_);
  pt.timestamp_ = pc_header.time;
  pt.range_bin_ = pc_float_bins.unsigned_range_bin;
  pt.power_bin_ = pc_float_bins.unsigned_power_bin;
  pt.doppler_bin_ = pc_float_bins.unsigned_doppler_bin;
  pt.elevation_bin_ = pc_float_bins.unsigned_elevation_bin;
  pt.azimuth_bin_ = pc_float_bins.unsigned_azimuth_bin;
  return pt;
}

bool RadarParserPhoenixA0::set_point_cloud(std::shared_ptr<LidarPointCloud>* &cloud, bool &ret,
                                          double height, const Packet* packet) {
#ifdef WITH_ROS2
  cloud_->cloud_msg_->header.stamp.sec = frame_end_utime_ / 1000000;
  cloud_->cloud_msg_->header.stamp.nanosec = frame_end_utime_ % 1000000 * 1000;
  cloud_->cloud_msg_->data.resize(radar_point_count_ * sizeof(RadarPoint));
  cloud_->cloud_msg_->width = radar_point_count_;
  cloud_->cloud_msg_->height = height;
  cloud_->cloud_msg_->row_step = sizeof(RadarPoint) * cloud_->cloud_msg_->width;
  cloud_->last_packet_utime_ = packet->time_system;
  *cloud = cloud_;
  ret = true;
  if (!update_frame(packet->time_system)) {
    return false;
  }
#else
  auto header = cloud_->proto_cloud_->mutable_header();
  header->set_sequence_num(frame_seq_++);
  header->set_radar_timestamp(frame_end_utime_);
  cloud_->proto_cloud_->mutable_data()->resize(radar_point_count_ * sizeof(RadarPoint));
  cloud_->proto_cloud_->set_width(radar_point_count_);
  cloud_->proto_cloud_->set_height(height);
  cloud_->proto_cloud_->set_row_step(sizeof(RadarPoint) * cloud_->proto_cloud_->width());
  cloud_->last_packet_utime_ = packet->time_system();
  *cloud = cloud_;
  ret = true;
  if (!update_frame(packet->time_system())) {
    return false;
  }
#endif
  return true;
}

bool RadarParserPhoenixA0::parse_lidar_packet(const Packet* packet,
                                 std::shared_ptr<LidarPointCloud>* cloud) {
  bool ret = false;
  if (!is_radar_packet_valid(packet)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet data is invalid.";
    return ret;
  }

  if (radar_point_count_ > static_cast<int32_t>(config_.max_points())) {
    LOG(ERROR) << "[" << config_.frame_id() << "] radar point is close to max ["
               << radar_point_count_ << ", " << config_.max_points() << "]";
  }

  auto& packet_config = config_.lidar_packet_config();
  PcFloatBins pc_float_bins;
  PcFrameHeader pc_header;
  PcMetadata pcmetadata;
#ifdef WITH_ROS2
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data.data());
#else
  const uint8_t* data = reinterpret_cast<const uint8_t*>(packet->data().data());
#endif
  const TPointCloud* t_pointcloud = reinterpret_cast<const TPointCloud*>(data);

  get_header_info(t_pointcloud, pc_header);
  get_metadata_info(t_pointcloud, pcmetadata);

  if (pc_header.udp_lost_packages > 0) {
    LOG(WARNING) << "packages lost udp: " << pc_header.udp_lost_packages;
  }

  if (unlikely(cloud_ == nullptr)) {
    if (!is_frame_end(t_pointcloud)) {
      return false;
    }

    if (!update_frame(pc_header.time)) {
      return false;
    }
  } else {
    if (is_frame_end_) {
      // use system time if sensor time is invalid
      if (config_.use_local_time()) {
        auto now = get_now_microsecond();
        if (fabs(now - frame_end_utime_) > 10000000) frame_end_utime_ = now;
      }
      if (!set_point_cloud(cloud, ret, packet_config.lasers(), packet)) return false;
    }
  }

  for (uint32_t i = packet_config.block_offset() ; i < t_pointcloud->us_length ;
                                                  i += packet_config.point_size()) {
    TargetGeneric target_generic =
              get_physical_bin(&data[i], packet_config.point_size(), &t_pointcloud->us_metadata);
    get_pc_float_bins(pc_float_bins, target_generic, packet_config.point_size());

    RadarPoint &pt = radar_point_[radar_point_count_];
    pt = set_radar_point(pc_float_bins, pcmetadata, pc_header);
    if (is_invalid_point(pt)) {
      continue;
    }
    radar_point_count_++;
  }
  is_frame_end_ = is_frame_end(t_pointcloud);
  frame_end_utime_ = pc_header.time;
  last_frame_flag_ = t_pointcloud->us_last_packet;
  last_udp_counter_ = t_pointcloud->us_message_number;
  return ret;
}

}  // namespace airi
}  // namespace crdc
