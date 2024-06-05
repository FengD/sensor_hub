// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING

#include "ins_drivers/input/cyber/cyber_input.h"
#include "ins_drivers/input/input.h"
#include "ins_drivers/input/socket/ins_socket.h"
#include "ins_drivers/proto/ins_config.pb.h"
#include <gtest/gtest.h>
namespace crdc {
namespace airi {

REGISTER_INS_INPUT(InsSocketInput);
REGISTER_INS_INPUT(CyberInput);

class InsInputTest : public ::testing::Test {};

TEST_F(InsInputTest, socket_init_test) {
  InsSocketInput s;
  InsInputConfig config;
  EXPECT_EQ(false, s.init(config));
}

TEST_F(InsInputTest, socket_name_test) {
  InsSocketInput s;
  EXPECT_EQ("InsSocketInput", s.get_name());
}

TEST_F(InsInputTest, cyber_name_test) {
  CyberInput s;
  EXPECT_EQ("CyberInput", s.get_name());
}

} // namespace airi
} // namespace crdc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
