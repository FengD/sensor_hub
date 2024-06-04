// Copyright (C) 2022 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Ailiang Xing

#include <gtest/gtest.h>

#include "lidar_drivers/input/cyber/cyber_input.h"
#include "lidar_drivers/input/input.h"
#include "lidar_drivers/parser/parser.h"
#include "lidar_drivers/parser/innoviz/innoviz_one.h"
#include "lidar_drivers/parser/robosense/robosense_ruby.h"
#include "lidar_drivers/parser/velodyne/velodyne_vlp16.h"
#include "lidar_drivers/parser/arbe/Phoenix_A0.h"
#include "lidar_drivers/proto/lidar_config.pb.h"
namespace crdc {
namespace airi {

REGISTER_LIDAR_INPUT(CyberInput);
REGISTER_LIDAR_PARSER(LidarParserRSRuby);
REGISTER_LIDAR_PARSER(LidarParserInnovizone);
REGISTER_LIDAR_PARSER(LidarParserVlp16);
REGISTER_LIDAR_PARSER(RadarParserPhoenixA0);

class LidarTest : public ::testing::Test {};

TEST_F(LidarTest, robosense_packet_test) {
  std::string config_file_path =
      "../../../../modules/lidar_drivers/params/drivers/lidar/gtest/lidar_RSrubylite.prototxt";
  LidarConfig config;
  crdc::airi::util::get_proto_from_file(config_file_path, &config);
  auto s = LidarInputFactory::get(config.input_config().name());
  s->config_ = config.input_config();
  Packet *packet;
  s->init(config.input_config());
  s->get_lidar_data(&packet);
  LidarParserRSRuby a;
  const uint8_t *data = (const uint8_t *)packet->data().data();
  uint64_t timestamp = ((data[15]) + (data[14] << 8) + (data[13] << 16) + (data[12] << 24));
  uint64_t timestamp_ns = ((data[19]) + (data[18] << 8) + (data[17] << 16) + (data[16] << 24));
  EXPECT_EQ(a.get_packet_timestamp(packet), timestamp * 1000000 + timestamp_ns);
  a.config_ = config.parser_config();
  EXPECT_EQ(packet->size(), a.config_.lidar_packet_config().size());
  uint32_t header_checksum = (data[1] << 8) | data[0];
  EXPECT_EQ(header_checksum, a.config_.lidar_packet_config().check_sum());
  EXPECT_EQ(true, a.is_lidar_packet_valid(packet));
}

TEST_F(LidarTest, innovizone_parser_test) {
  std::string config_file_path =
      "../../../../modules/lidar_drivers/params/drivers/lidar/gtest/lidar_innovizone.prototxt";
  LidarConfig config;
  crdc::airi::util::get_proto_from_file(config_file_path, &config);
  auto s = LidarInputFactory::get(config.input_config().name());
  s->config_ = config.input_config();
  Packet *packet;
  s->init(config.input_config());
  s->get_lidar_data(&packet);
  LidarParserInnovizone l;
  const uint8_t *data = (const uint8_t *)packet->data().data();
  l.config_ = config.parser_config();
  uint32_t header_checksum = (data[1] << 8) | data[0];
  EXPECT_EQ(header_checksum, l.config_.lidar_packet_config().check_sum());
  EXPECT_EQ(true, l.is_lidar_packet_valid(packet));
  EXPECT_EQ(true, l.init_lidar_parser());

  int start_index = 148;
  uint32_t world_time_s = (data[3 + start_index] << 24) | (data[2 + start_index] << 16) |
         (data[1 + start_index] << 8) | (data[start_index]);
  EXPECT_EQ(l.get_uint32(data, start_index), world_time_s);

  start_index = 152;
  uint32_t world_time_us = (data[3 + start_index] << 24) | (data[2 + start_index] << 16) |
         (data[1 + start_index] << 8) | (data[start_index]);
  uint64_t timestamp = (world_time_s * 100000 + world_time_us / 100000) * 100000 +
                        world_time_us % 100000;
  EXPECT_EQ(l.get_packet_timestamp(packet), timestamp);

  uint16_t u16_t = (data[1 + start_index] << 8) | (data[start_index]);
  EXPECT_EQ(l.get_uint16(data, start_index), u16_t);

  uint32_t byt = (data[3 + start_index] << 24) | (data[2 + start_index] << 16) |
                 (data[1 + start_index] << 8) | (data[start_index]);
  float res;
  memcpy(&res, &byt, sizeof(res));
  EXPECT_EQ(l.get_float32(data, start_index), res);
}

TEST_F(LidarTest, vlp16_packet_test) {
  std::string config_file_path =
      "../../../../modules/lidar_drivers/params/drivers/lidar/gtest/lidar_velodyne.prototxt";
  LidarConfig config;
  crdc::airi::util::get_proto_from_file(config_file_path, &config);
  auto s = LidarInputFactory::get(config.input_config().name());
  Packet *packet;
  // test CyberInput::init
  EXPECT_TRUE(s->init(config.input_config()));
  EXPECT_NE(0, s->config_.cyber_config().file_path().size());
  EXPECT_TRUE(s->init_pool());
  //test func of is_lidar_packet_valid
  s->get_lidar_data(&packet);
  LidarParserVlp16 l;
  l.config_ = config.parser_config();
  EXPECT_EQ(packet->size(), l.config_.lidar_packet_config().size());
  const uint8_t *data = (const uint8_t *)packet->data().data();
  uint32_t header_checksum = (data[1] << 8) | data[0];
  EXPECT_EQ(header_checksum, l.config_.lidar_packet_config().check_sum());
  EXPECT_EQ(true, l.is_lidar_packet_valid(packet));
  // test timestamp
  uint64_t timestamp = ((data[1200]) + (data[1201] << 8) + (data[1202] << 16) + (data[1203] << 24));
  EXPECT_EQ(l.get_packet_timestamp(packet), timestamp);
}

TEST_F(LidarTest, phoenixA0_packet_test) {
  std::string config_file_path =
      "../../../../modules/lidar_drivers/params/drivers/lidar/gtest/radar_cyber.prototxt";
  LidarConfig config;
  crdc::airi::util::get_proto_from_file(config_file_path, &config);
  auto s = LidarInputFactory::get(config.input_config().name());
  Packet *packet;
  // test CyberInput::init
  EXPECT_TRUE(s->init(config.input_config()));
  EXPECT_NE(0, s->config_.cyber_config().file_path().size());
  EXPECT_TRUE(s->init_pool());
  // //test func of parser
  s->get_lidar_data(&packet);
  RadarParserPhoenixA0 l;
  l.config_ = config.parser_config();
  const uint8_t *data = (const uint8_t *)packet->data().data();
  const TPointCloud* t_pointcloud = reinterpret_cast<const TPointCloud*>(data);
  EXPECT_EQ((uint16_t)t_pointcloud->us_prefix, l.config_.lidar_packet_config().check_sum());
  EXPECT_EQ(sizeof(TPointCloud), l.config_.lidar_packet_config().block_offset());
}

} // namespace airi
} // namespace crdc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
