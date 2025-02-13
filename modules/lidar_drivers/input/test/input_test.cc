// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING

#include <gtest/gtest.h>

#include "lidar_drivers/input/input.h"
#include "lidar_drivers/input/socket/socket.h"
#include "lidar_drivers/input/cyber/cyber_input.h"
#include "lidar_drivers/proto/lidar_config.pb.h"

namespace sensor {
namespace hub {

REGISTER_LIDAR_INPUT(SocketInput);
REGISTER_LIDAR_INPUT(CyberInput);

class LidarInputTest : public ::testing::Test {};

TEST_F(LidarInputTest, socket_init_test) {
  SocketInput s;
  LidarInputConfig config;
  EXPECT_EQ(false, s.init(config));
}

TEST_F(LidarInputTest, socket_name_test) {
  SocketInput s;
  EXPECT_EQ("SocketInput", s.get_name());
}

TEST_F(LidarInputTest, cyber_name_test) {
  CyberInput s;
  EXPECT_EQ("CyberInput", s.get_name());
}

TEST_F(LidarInputTest, cyber_init_test) {
  CyberInput s;
  std::string config_file_path =
      "../../../../../modules/lidar_drivers/params/drivers/lidar/gtest/cyber_input_test.prototxt";
  LidarConfig config;
  sensor::hub::util::get_proto_from_file(config_file_path, &config);
  s.config_ = config.input_config();
  auto file_index_ = 0;
  auto reader_ = std::make_shared<RecordReader>(s.config_.cyber_config().file_path(file_index_));
  const auto &type = reader_->GetMessageType(s.config_.cyber_config().channel());
  EXPECT_EQ(true, s.config_.has_cyber_config());
  EXPECT_EQ(false, type.empty());
  EXPECT_EQ(true, s.init_pool());
  EXPECT_EQ(true, s.init(config.input_config()));
}

}  // namespace hub
}  // namespace sensor

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
