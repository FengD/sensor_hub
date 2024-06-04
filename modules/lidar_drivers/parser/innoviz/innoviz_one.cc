// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: JianFei JIANG
// Description: lidar parser Innovizone

#include "lidar_drivers/parser/innoviz/innoviz_one.h"
#include <vector>

namespace crdc {
namespace airi {

LidarParserInnovizone::LidarParserInnovizone() {
  frame_end_ = false, udp_matrix_calculate_ = true;
  frame_num_ = 0, lidar_point_sum_ = 0, ox16_temp_index_ = 0, ox16_start_index_ = 0;
  r_mat_ = std::vector<Eigen::Matrix<float, 3, 3>>(4, Eigen::Matrix<float, 3, 3>::Zero());
  d_mat_ = std::vector<Eigen::Matrix<float, 3, 1>>(4, Eigen::Matrix<float, 3, 1>::Zero());
  v_mat_ = std::vector<Eigen::Matrix<float, 8, 3>>(4, Eigen::Matrix<float, 8, 3>::Zero());
}

bool LidarParserInnovizone::init_lidar_parser() {
  auto packet_config = config_.mutable_lidar_packet_config();
  if (packet_config->ring_map_size() != packet_config->lasers()) {
    packet_config->clear_ring_map();
    for (auto i = 0; i < packet_config->lasers(); ++i) {
      packet_config->add_ring_map(i);
    }
  }

  if (packet_config->calib_elevation_size() != packet_config->lasers()) {
    packet_config->clear_calib_elevation();
    for (auto i = 0; i < packet_config->lasers(); ++i) {
      packet_config->add_calib_elevation(i);
    }
  }

  if (packet_config->calib_azimuth_size() != packet_config->lasers()) {
    packet_config->clear_calib_azimuth();
    for (auto i = 0; i < packet_config->lasers(); ++i) {
      packet_config->add_calib_azimuth(0.0);
    }
  }
  return true;
}

void LidarParserInnovizone::get_block_info(const uint8_t *data,
                                        LidarParserInfo &parser_info) {
  (void)data;
  (void)parser_info;
}

void LidarParserInnovizone::get_point_raw_info(const uint8_t *data,
                                            LidarParserInfo &parser_info) {
  (void)data;
  (void)parser_info;
}

uint64_t LidarParserInnovizone::get_packet_timestamp(const Packet *packet) {
#ifdef WITH_ROS2
  const uint8_t *data = (const uint8_t *)packet->data.data();
#else
  const uint8_t *data = (const uint8_t *)packet->data().data();
#endif
  uint32_t world_time_s = get_uint32(data, 148);
  uint32_t world_time_us = get_uint32(data, 152);
  uint64_t timestamp = (world_time_s * 100000 + world_time_us / 100000) * 100000 +
                        world_time_us % 100000;
  return timestamp;
}

void LidarParserInnovizone::calibrate_point(LidarParserInfo &parser_info) {
  (void)parser_info;
}

bool LidarParserInnovizone::is_lidar_packet_valid(const Packet *packet) {
#ifdef WITH_ROS2
  const uint8_t *data = (const uint8_t *)packet->data.data();
#else
  const uint8_t *data = (const uint8_t *)packet->data().data();
#endif
  uint32_t header_checksum = (data[1] << 8) | data[0];
  if (header_checksum != config_.lidar_packet_config().check_sum()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] checksum " << header_checksum
               << " wrong " << config_.lidar_packet_config().check_sum();
    return false;
  }
  return true;
}

uint32_t LidarParserInnovizone::get_uint32(const uint8_t *data, const int& start_index) {
  return (data[3 + start_index] << 24) | (data[2 + start_index] << 16) |
         (data[1 + start_index] << 8) | (data[start_index]);
}

uint16_t LidarParserInnovizone::get_uint16(const uint8_t *data, const int& start_index) {
  return (data[1 + start_index] << 8) | (data[start_index]);
}

float LidarParserInnovizone::get_float32(const uint8_t *data, const int& start_index) {
  uint32_t byt = (data[3 + start_index] << 24) | (data[2 + start_index] << 16) |
                 (data[1 + start_index] << 8) | (data[start_index]);
  float res;
  memcpy(&res, &byt, sizeof(res));
  return res;
}

template<typename T>
void LidarParserInnovizone::get_matrix(T& input_mat, const int& start_index,
                                      const uint8_t* data) {
  for (auto i = 0; i < input_mat.rows(); ++i) {
    for (auto j = 0; j < input_mat.cols(); ++j) {
      int index = start_index + 4 * (i * input_mat.cols() + j);
      input_mat(i, j) = get_float32(data, index);
    }
  }
}

Eigen::Vector3f LidarParserInnovizone::calculate_points_xyz(
    const float& r, const int& i, const int& k,
    const Eigen::Matrix<float, 3, 3> &q_mat) {
  Eigen::Matrix<float, 8, 3> v_i_mat = v_mat_.at(i);
  Eigen::Matrix<float, 3, 1> v_i_k(v_i_mat(k, 0), v_i_mat(k, 1), v_i_mat(k, 2));
  Eigen::Matrix<float, 3, 1> xyz_vector = (r * q_mat * (-v_i_k) + d_mat_.at(i)) * 0.01;
  return xyz_vector;
}

void LidarParserInnovizone::macro_pixel_cloud(const int& lrf_num, const int& ref_num,
                                           const int& pixel_num,
                                           const uint8_t* data, const int& start_index,
                                           LidarParserInfo &parser_info,
                                           const Eigen::Matrix<float, 3, 3>& q_mat) {
  uint16_t pixel_distance = 0;
  int intensity = 0, confidence = 0,  pixel_start_index = start_index;
  for (auto j = 0; j < ref_num; j++) {
    pixel_distance = get_uint16(data, pixel_start_index);
    intensity = data[pixel_start_index + 2];
    confidence = data[pixel_start_index + 3] & 63;
    parser_info.distance_ = pixel_distance;
    parser_info.intensity_ = intensity;

    pixel_start_index += 4;
    lidar_point_sum_++;
    if (confidence < 62) {
      lidar_point_sum_--;
      continue;
    }

    Eigen::Vector3f dis_mat = calculate_points_xyz(pixel_distance, lrf_num, pixel_num, q_mat);
    LidarPoint &pt = lidar_point_[lidar_point_sum_];
    pt.ring_ = 0;
    pt.timestamp_ = parser_info.timestamp_;
    pt.intensity_ = parser_info.intensity_;
    pt.distance = parser_info.distance_;
    pt.x_ = dis_mat(0);
    pt.y_ = dis_mat(1);
    pt.z_ = dis_mat(2);
  }
}

Eigen::Matrix<float, 3, 3> LidarParserInnovizone::macro_pixel_angle_matrix(
    const int& lrf_num, const uint8_t *data, const int& start_index) {
  int16_t theta_val = (data[start_index + 647] << 8) | data[start_index + 646];
  int16_t phi_val = (data[start_index + 649] << 8) | data[start_index + 648];

  float x = theta_val * POW_2_18;
  float y = phi_val * POW_2_18;
  float x_sq = x * x, y_sq = y * y;
  float sin_theta = 2 * x / (1 + x_sq), sin_phi = 2 * y / (1 + y_sq),
        cos_theta = (1 - x_sq) / (1 + y_sq), cos_phi = (1 - x_sq) / (1 + y_sq);

  Eigen::Matrix<float, 3, 1> u_mat;
  u_mat(0) = cos_theta * cos_phi;
  u_mat(1) = -sin_theta * cos_phi;
  u_mat(2) = sin_phi;
  Eigen::Matrix<float, 3, 3> unit_mat;
  unit_mat.setIdentity(3, 3);
  Eigen::Matrix<float, 3, 3> q_mat;
  q_mat = r_mat_.at(lrf_num) * (unit_mat - 2 * (u_mat * u_mat.transpose()));
  return q_mat;
}

int LidarParserInnovizone::calculate_macro_pixel(const int& lrf_num,
                                              const uint8_t *data,
                                              const int& start_index,
                                              LidarParserInfo &parser_info) {
  Eigen::Matrix<float, 3, 3> q_mat = macro_pixel_angle_matrix(lrf_num, data, start_index);
  int reflection_sum = 0;
  int start_reflection = start_index + 652;
  int pixel_index = start_index + 668;
  int pixel_num = 0;

  for (auto i = start_reflection; i < start_reflection + 16; i += 2) {
    int reflection = data[i] & 3;
    macro_pixel_cloud(lrf_num, reflection, pixel_num, data, pixel_index,
                      parser_info, q_mat);
    pixel_index = 4 * reflection + pixel_index;
    reflection_sum += reflection;
    pixel_num++;
  }
  return reflection_sum;
}

void LidarParserInnovizone::get_lrf_pixel(const int& start_index, const uint8_t *data,
                                      LidarParserInfo &parser_info) {
  int length_start_index = start_index + 4;
  uint32_t macro_pixel_length[4] = {0};
  macro_pixel_length[0] = start_index;
  macro_pixel_length[1] = get_uint16(data, length_start_index) + 8;
  macro_pixel_length[2] =
      macro_pixel_length[1] + get_uint16(data, (macro_pixel_length[1] + 4)) + 8;

  int length = macro_pixel_length[0] + 12;
  int lrf_num = data[length];
  int reflection_sum = 0;
  for (auto m = 0; m < 3; m++) {
    int pixel_start_index[101] = {0};
    int pixel_sum = 0;
    int macro_num = 1;
    for (auto k = 1; k < 101; k++) {
      reflection_sum = calculate_macro_pixel(lrf_num, data,
                           macro_pixel_length[m] + pixel_start_index[k - 1], parser_info);
      pixel_sum += reflection_sum;
      if ((macro_num % 2) == 0) {
        pixel_start_index[k] = pixel_start_index[k - 1] + 4 * reflection_sum + 48;
      } else {
        pixel_start_index[k] = pixel_start_index[k - 1] + 4 * reflection_sum + 24;
      }
      macro_num++;
    }
  }
}

void LidarParserInnovizone::udp_matrix_calculate(const uint8_t *data) {
  if (udp_matrix_calculate_) {
    for (size_t i = 0; i < 1000; i++) {
      uint32_t data_type = get_uint32(data, i);
      if (data_type == OX17_TYPE) {
        for (size_t j = 0; j < 4; j++) {
          get_matrix(r_mat_.at(j), i + 84 + j * 36, data);
          get_matrix(d_mat_.at(j), i + 228 + j * 12, data);
          get_matrix(v_mat_.at(j), i + 276 + j * 96, data);
        }
      }
      if (data_type == OX16_TYPE) {
        ox16_temp_index_ = i;
        break;
      }
    }
    udp_matrix_calculate_ = false;
  }
}

bool LidarParserInnovizone::parse_lidar_packet(
    const Packet *packet, std::shared_ptr<LidarPointCloud> *cloud) {
  bool ret = false;
  if (!is_lidar_packet_valid(packet)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] packet is invalid.";
    return ret;
  }

  if (lidar_point_sum_ > static_cast<int32_t>(config_.max_points())) {
    LOG(ERROR) << "[" << config_.frame_id() << "] lidar point is close to max ["
               << lidar_point_sum_ << ", " << config_.max_points() << "]";
  }

  // parse_lidar_other_info(packet);
  auto &packet_config = config_.lidar_packet_config();
  LidarParserInfo parser_info;
#ifdef WITH_ROS2
  const uint8_t *data = reinterpret_cast<const uint8_t *>(packet->data.data()) +
                        packet_config.block_offset();
#else
  const uint8_t *data = reinterpret_cast<const uint8_t *>(packet->data().data()) +
                        packet_config.block_offset();
#endif

  uint32_t udp_type = get_uint32(data, 0);
  if (udp_type == OX41_TYPE) {
    // is_frame_end
    frame_num_++;
    if (frame_num_ == 2) {
      frame_end_ = true;
      frame_num_--;
    }
    udp_matrix_calculate(data);
    parser_info.packet_timestamp_ = get_timestamp(packet);
    frame_end_utime_ = parser_info.packet_timestamp_;
    ox16_start_index_ = ox16_temp_index_;
  } else if (udp_type == OX16_TYPE) {
    ox16_start_index_ = 0;
  }

  if (unlikely(cloud_ == nullptr)) {
    if (!frame_end_) {
      return false;
    }
    if (!update_frame(parser_info.packet_timestamp_, parser_info)) {
      return false;
    }
  } else {
    if (frame_end_) {
#ifdef WITH_ROS2
      cloud_->cloud_msg_->header.stamp.sec = frame_end_utime_ / 1000000;
      cloud_->cloud_msg_->header.stamp.nanosec = frame_end_utime_ % 1000000 * 1000;
      cloud_->cloud_msg_->data.resize(lidar_point_sum_ * sizeof(LidarPoint));
      cloud_->cloud_msg_->width = lidar_point_sum_ / packet_config.lasers();
      cloud_->cloud_msg_->height = packet_config.lasers();
      cloud_->cloud_msg_->row_step = sizeof(LidarPoint) * cloud_->cloud_msg_->width;
      cloud_->last_packet_utime_ = packet->time_system;
      *cloud = cloud_;
      ret = true;
      lidar_point_sum_ = 0;
      frame_end_ = false;
      if (!update_frame(packet->time_system, parser_info)) {
        return false;
      }
#else
      auto header = cloud_->proto_cloud_->mutable_header();
      header->set_sequence_num(frame_seq_++);
      header->set_lidar_timestamp(frame_end_utime_);
      cloud_->proto_cloud_->mutable_data()->resize(lidar_point_sum_ * sizeof(LidarPoint));
      cloud_->proto_cloud_->set_width(lidar_point_sum_ / packet_config.lasers());
      cloud_->proto_cloud_->set_height(packet_config.lasers());
      cloud_->proto_cloud_->set_row_step(sizeof(LidarPoint) * cloud_->proto_cloud_->width());
      cloud_->last_packet_utime_ = packet->time_system();
      *cloud = cloud_;
      ret = true;
      lidar_point_sum_ = 0;
      frame_end_ = false;
      if (!update_frame(packet->time_system(), parser_info)) {
        return false;
      }
#endif
    }
  }
  get_lrf_pixel(ox16_start_index_, data, parser_info);
  return ret;
}
}  // namespace airi
}  // namespace crdc
