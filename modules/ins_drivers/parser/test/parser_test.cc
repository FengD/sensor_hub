// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING

#include <gtest/gtest.h>

#include "ins_drivers/parser/asensing_ins570d/asensing_ins570d.h"
#include "ins_drivers/parser/parser.h"
#include "ins_drivers/proto/ins_config.pb.h"
namespace crdc {
namespace airi {

REGISTER_INS_PARSER(InsParser570d);

class InsParserTest : public ::testing::Test {};

TEST_F(InsParserTest, ins570d_name_test) {
  InsParser570d a;
  EXPECT_EQ("InsParser570d", a.get_name());
}

TEST_F(InsParserTest, ins570d_init_test) {
  InsParser570d a;
  EXPECT_EQ(true, a.init_ins_parser());
}

TEST_F(InsParserTest, ins570d_get_timestamp_test) {
  InsParser570d a;
  Packet *packet;
  EXPECT_EQ(1, a.get_packet_timestamp(packet));
}

} // namespace airi
} // namespace crdc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
