// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: JianFei JIANG
// Description: lidar parser Innovizone

#pragma once

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>
#include "common/common.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

class LidarParserInnovizone : public LidarParser {
 public:
  LidarParserInnovizone();
  virtual ~LidarParserInnovizone() = default;

  bool init_lidar_parser() override;

  void get_block_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  void get_point_raw_info(const uint8_t* data, LidarParserInfo& parser_info) override;

  uint64_t get_packet_timestamp(const Packet* packet) override;

  void calibrate_point(LidarParserInfo& parser_info) override;

  bool is_lidar_packet_valid(const Packet* packet) override;

  bool parse_lidar_packet(const Packet* packet,
                          std::shared_ptr<LidarPointCloud>* cloud) override;

  std::string get_name() const override {
    return "LidarParserInnovizone";
  }

 private:
  uint16_t get_uint16(const uint8_t *data, const int& start_index);

  uint32_t get_uint32(const uint8_t *data, const int& start_index);

  float get_float32(const uint8_t *data, const int& start_index);

  template<typename T> void get_matrix(T& input_mat, const int& start_index,
                                      const uint8_t* data);

  Eigen::Vector3f calculate_points_xyz(const float& r, const int& i,
          const int& k, const Eigen::Matrix<float, 3, 3>& q_mat);

  void macro_pixel_cloud(const int& lrf_num, const int& ref_num,
                         const int& pixel_num, const uint8_t* data,
                         const int& start_index, LidarParserInfo& parser_info,
                         const Eigen::Matrix<float, 3, 3>& q_mat);

  Eigen::Matrix<float, 3, 3> macro_pixel_angle_matrix(
                const int& lrf_num, const uint8_t *data, const int& start_index);

  int calculate_macro_pixel(const int& lrf_num, const uint8_t* data,
                           const int& start_index, LidarParserInfo& parser_info);

  void udp_matrix_calculate(const uint8_t *data);

  void get_lrf_pixel(const int& start_index, const uint8_t* data,
                             LidarParserInfo& parser_info);

  bool frame_end_, udp_matrix_calculate_;
  int frame_num_, lidar_point_sum_, ox16_temp_index_, ox16_start_index_;
  std::vector<Eigen::Matrix<float, 3, 3>> r_mat_;
  std::vector<Eigen::Matrix<float, 3, 1>> d_mat_;
  std::vector<Eigen::Matrix<float, 8, 3>> v_mat_;

  static constexpr int OX16_TYPE = 327702, OX17_TYPE = 327703, OX41_TYPE = 327745;
  static constexpr float POW_2_18 = 0.000003814697265625f;

#ifdef WITH_TEST
  FRIEND_TEST(LidarTest, innovizone_parser_test);
#endif
};

}  // namespace airi
}  // namespace crdc
