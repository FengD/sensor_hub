// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING

#include <gtest/gtest.h>

#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/parser/pandar/pandar_xt32.h"
#include "lidar_drivers/parser/pandar/pandar_p40.h"
#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
namespace sensor {
namespace hub {
  REGISTER_LIDAR_PARSER(LidarParserPxt32);
  REGISTER_LIDAR_PARSER(LidarParserP40);
  REGISTER_LIDAR_PARSER(LidarParserVlp16);

  class LidarParserTest : public ::testing::Test {};

  TEST_F(LidarParserTest, pandarxt32_name_test) {
    LidarParserPxt32 s;
    EXPECT_EQ("LidarParserPxt32", s.get_name());
  }

  TEST_F(LidarParserTest, pandarp40_name_test) {
    LidarParserP40 s;
    EXPECT_EQ("LidarParserP40", s.get_name());
  }

  TEST_F(LidarParserTest, vlp16_name_test) {
    LidarParserVlp16 s;
    EXPECT_EQ("LidarParserVlp16", s.get_name());
  }

  TEST_F(LidarParserTest, vlp16_init_parser_test) {
    LidarParserVlp16 l;
    std::string config_file_path =
        "../../../../../modules/lidar_drivers/params/drivers/lidar/gtest/lidar_velodyne.prototxt";
    LidarConfig config;
    sensor::hub::util::get_proto_from_file(config_file_path, &config);
    l.config_ = config.parser_config();

    // test for func init_lidar_parser
    EXPECT_TRUE(l.init_lidar_parser());
    auto lidar_packet_config = l.config_.lidar_packet_config();
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.ring_map_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_elevation_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_azimuth_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), l.calib_info_.size());
  }

  TEST_F(LidarParserTest, vlp16_is_frame_end_test){
    LidarParserVlp16 l;
    std::string config_file_path =
      "../../../../../modules/lidar_drivers/params/drivers/lidar/gtest/lidar_velodyne.prototxt";
    LidarConfig config;
    sensor::hub::util::get_proto_from_file(config_file_path, &config);
    l.config_ = config.parser_config();

    // last = split - 1; split; azimuth = split + 1
    l.last_pixel_azimuth_ = l.config_.split_azimuth() - 1;
    uint16_t azimuth = l.config_.split_azimuth() + 1;
    EXPECT_TRUE(l.is_frame_end(azimuth));
    // last = split + 2; split; azimuth = split + 1
    l.last_pixel_azimuth_ = l.config_.split_azimuth() + 2;
    azimuth = l.config_.split_azimuth() + 1;
    EXPECT_TRUE(l.is_frame_end(azimuth));
    // last = max - 1; split = 0; azimuth = 1
    l.last_pixel_azimuth_ = ROTATION_MAX_UNITS - 1;
    l.config_.set_split_azimuth(0);
    azimuth = 1;
    EXPECT_TRUE(l.is_frame_end(azimuth));
    // last = max - 2; split = 0; azimuth = max - 1
    l.last_pixel_azimuth_ = ROTATION_MAX_UNITS - 2;
    l.config_.set_split_azimuth(0);
    azimuth = ROTATION_MAX_UNITS - 1;
    EXPECT_FALSE(l.is_frame_end(azimuth));
    // last = max - 1; split = azimuth = 0
    l.last_pixel_azimuth_ = ROTATION_MAX_UNITS - 1;
    l.config_.set_split_azimuth(0);
    azimuth = l.config_.split_azimuth();
    EXPECT_TRUE(l.is_frame_end(azimuth));
  }

  TEST_F(LidarParserTest, pandarxt32_init_parser_test) {
    LidarParserPxt32 l;
    std::string config_file_path = 
      "../../../../../modules/lidar_drivers/params/drivers/lidar/test/lidar_right.prototxt";
    LidarConfig config;
    sensor::hub::util::get_proto_from_file(config_file_path, &config);
    l.config_ = config.parser_config();

    // test for func init_lidar_parser
    EXPECT_TRUE(l.init_lidar_parser());
    auto lidar_packet_config = l.config_.lidar_packet_config();
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_elevation_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_azimuth_size());

    for (int i = 0; i < lidar_packet_config.lasers(); ++i) {
      EXPECT_EQ(lidar_packet_config.calib_elevation(i), l.LIDAR_LINE_ANGLE32[i] * RAD_PER_DEGREE);
      EXPECT_EQ(lidar_packet_config.calib_azimuth(i), 0.0);
    }
  }

  TEST_F(LidarParserTest, pardarxt32_is_frame_end_test) {
    LidarParserPxt32 l;
    std::string config_file_path = 
      "../../../../../modules/lidar_drivers/params/drivers/lidar/test/lidar_right.prototxt";
    LidarConfig config;
    sensor::hub::util::get_proto_from_file(config_file_path, &config);
    l.config_ = config.parser_config();

    // last = split - 1; split; azimuth = split + 1
    l.last_pixel_azimuth_ = l.config_.split_azimuth() - 1;
    uint16_t azimuth = l.config_.split_azimuth() + 1;
    EXPECT_TRUE(l.is_frame_end(azimuth));

    // last = split + 2; split; azimuth = split + 1
    l.last_pixel_azimuth_ = l.config_.split_azimuth() + 2;
    azimuth = l.config_.split_azimuth() + 1;
    EXPECT_TRUE(l.is_frame_end(azimuth));

    // last = max - 1; split = 0; azimuth = 1
    l.last_pixel_azimuth_ = ROTATION_MAX_UNITS - 1;
    l.config_.set_split_azimuth(0);
    azimuth = 1;
    EXPECT_TRUE(l.is_frame_end(azimuth));

    // last = max - 2; split = 0; azimuth = max - 1
    l.last_pixel_azimuth_ = ROTATION_MAX_UNITS - 2;
    l.config_.set_split_azimuth(0);
    azimuth = ROTATION_MAX_UNITS - 1;
    EXPECT_FALSE(l.is_frame_end(azimuth));

    // last = max - 1; split = azimuth = 0;
    l.last_pixel_azimuth_ = ROTATION_MAX_UNITS - 1;
    l.config_.set_split_azimuth(0);
    azimuth = l.config_.split_azimuth();
    EXPECT_TRUE(l.is_frame_end(azimuth)); 
  }

  } // namespace hub
  } // namespace sensor

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}