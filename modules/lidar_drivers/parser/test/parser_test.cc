// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING

#include <gtest/gtest.h>

#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/parser/pandar/pandar_xt32.h"
#include "lidar_drivers/parser/pandar/pandar_p40.h"
#include "lidar_drivers/parser/innoviz/innoviz_one.h"
#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"
#include "lidar_drivers/parser/robosense/robosense_ruby.h"
#include "lidar_drivers/parser/single/lanhai_lds50cs.h"
#include "lidar_drivers/parser/arbe/Phoenix_A0.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
namespace crdc {
namespace airi {
  REGISTER_LIDAR_PARSER(LidarParserPxt32);
  REGISTER_LIDAR_PARSER(LidarParserP40);
  REGISTER_LIDAR_PARSER(LidarParserInnovizone);
  REGISTER_LIDAR_PARSER(LidarParserVlp16);
  REGISTER_LIDAR_PARSER(LidarParserRSRuby);
  REGISTER_LIDAR_PARSER(LidarParserLds50cs);
  REGISTER_LIDAR_PARSER(RadarParserPhoenixA0);

  class LidarParserTest : public ::testing::Test {};

  TEST_F(LidarParserTest, pandarxt32_name_test) {
    LidarParserPxt32 s;
    EXPECT_EQ("LidarParserPxt32", s.get_name());
  }

  TEST_F(LidarParserTest, pandarp40_name_test) {
    LidarParserP40 s;
    EXPECT_EQ("LidarParserP40", s.get_name());
  }

  TEST_F(LidarParserTest, innovizone_name_test) {
    LidarParserInnovizone s;
    EXPECT_EQ("LidarParserInnovizone", s.get_name());
  }

  TEST_F(LidarParserTest, vlp16_name_test) {
    LidarParserVlp16 s;
    EXPECT_EQ("LidarParserVlp16", s.get_name());
  }

  TEST_F(LidarParserTest, rsrubylite_name_test) {
    LidarParserRSRuby s;
    EXPECT_EQ("LidarParserRSRuby", s.get_name());
  }

  TEST_F(LidarParserTest, lanhailds50cs_name_test) {
    LidarParserLds50cs s;
    EXPECT_EQ("LidarParserLds50cs", s.get_name());
  }

  TEST_F(LidarParserTest, phoenixA0_name_test) {
    RadarParserPhoenixA0 s;
    EXPECT_EQ("RadarParserPhoenixA0", s.get_name());
  }

  TEST_F(LidarParserTest, lanhailds50cs_init_lidar_parser_test) {
    LidarParserLds50cs l;
    std::string config_file_path = 
      "../../../../../modules/lidar_drivers/params/drivers/lidar/HE00-3/lidar_rear.prototxt";
    LidarConfig config;
    crdc::airi::util::get_proto_from_file(config_file_path, &config);
    l.config_ = config.parser_config();

    EXPECT_TRUE(l.init_lidar_parser());
    auto lidar_packet_config = l.config_.lidar_packet_config();
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.ring_map_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_elevation_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_azimuth_size());
    for (int i = 0; i < lidar_packet_config.lasers(); ++i) {
      EXPECT_EQ(lidar_packet_config.ring_map(i), i);
      EXPECT_EQ(lidar_packet_config.calib_elevation(i), i);
      EXPECT_EQ(lidar_packet_config.calib_azimuth(i), i);
    }
  }

  TEST_F(LidarParserTest, lanhailds50cs_is_frame_end_test) {
    LidarParserLds50cs l;
    std::string config_file_path = 
      "../../../../../modules/lidar_drivers/params/drivers/lidar/HE00-3/lidar_rear.prototxt";
    LidarConfig config;
    crdc::airi::util::get_proto_from_file(config_file_path, &config);
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

  TEST_F(LidarParserTest, lanhailds50cs_pixel_xyz_test) {
    LidarParserLds50cs l;
    // input NULL
    l.pixel_xyz(nullptr);
    EXPECT_EQ(0, l.lidar_point_sum_);
    // data.N_ = 0
    LidarParserLds50cs::RawData data = {0, 0, 0, {0}};
    l.pixel_xyz(reinterpret_cast<const uint8_t *>(&data));
    EXPECT_EQ(0, l.lidar_point_sum_);
    // data.N_ = 10
    data.N_ = 10;
    struct LidarPoint points[10];
    l.lidar_point_ = points;
    l.pixel_xyz(reinterpret_cast<const uint8_t *>(&data));
    EXPECT_EQ(10, l.lidar_point_sum_);
    EXPECT_FALSE(l.frame_end_);
  }

  TEST_F(LidarParserTest, vlp16_init_parser_test) {
    LidarParserVlp16 l;
    std::string config_file_path =
        "../../../../../modules/lidar_drivers/params/drivers/lidar/gtest/lidar_velodyne.prototxt";
    LidarConfig config;
    crdc::airi::util::get_proto_from_file(config_file_path, &config);
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
    crdc::airi::util::get_proto_from_file(config_file_path, &config);
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

  TEST_F(LidarParserTest, robosense_init_parser_test) {
    LidarParserRSRuby l;
    std::string config_file_path =
        "../../../../../modules/lidar_drivers/params/drivers/lidar/gtest/lidar_RSrubylite.prototxt";
    LidarConfig config;
    crdc::airi::util::get_proto_from_file(config_file_path, &config);
    l.config_ = config.parser_config();
    EXPECT_TRUE(l.init_lidar_parser());
    auto lidar_packet_config = l.config_.lidar_packet_config();
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.ring_map_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_elevation_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), lidar_packet_config.calib_azimuth_size());
    EXPECT_EQ(l.config_.lidar_packet_config().lasers(), l.calib_info_.size());
    for (int i = 0; i < lidar_packet_config.lasers(); ++i) {
      EXPECT_EQ(lidar_packet_config.ring_map(i), i);
      EXPECT_EQ(lidar_packet_config.calib_elevation(i),
                l.LIDAR_LINE_ANGLE80_V2R_Lite[i] * RAD_PER_DEGREE);
      EXPECT_EQ(lidar_packet_config.calib_azimuth(i), l.HORIZON_CORRECT_ANGLE80_V2R_Lite[i]);
    }
  }

  TEST_F(LidarParserTest, pandarxt32_init_parser_test) {
    LidarParserPxt32 l;
    std::string config_file_path = 
      "../../../../../modules/lidar_drivers/params/drivers/lidar/test/lidar_right.prototxt";
    LidarConfig config;
    crdc::airi::util::get_proto_from_file(config_file_path, &config);
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
    crdc::airi::util::get_proto_from_file(config_file_path, &config);
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

  } // namespace airi
  } // namespace crdc

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}