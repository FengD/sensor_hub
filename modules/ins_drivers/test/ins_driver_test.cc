// Copyright (C) 2022 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Ailiang Xing

#include <gtest/gtest.h>

#include "ins_drivers/input/cyber/cyber_input.h"
#include "ins_drivers/input/socket/ins_socket.h"
#include "ins_drivers/ins.h"
#include "ins_drivers/parser/asensing_ins570d/asensing_ins570d.h"
#include "ins_drivers/proto/ins_component_config.pb.h"
#include "ins_drivers/proto/ins_config.pb.h"
namespace crdc {
namespace airi {
REGISTER_INS_PARSER(InsParser570d);
REGISTER_INS_INPUT(InsSocketInput);
REGISTER_INS_INPUT(CyberInput);
class InsDriverTest : public ::testing::Test {};

TEST_F(InsDriverTest, ins_name_test) {
  setenv("CRDC_WS", ".", 1);
  InsComponent config;
  InsComponentConfig config2;
  std::string config_file_path =
      "../../../../modules/ins_drivers/params/drivers/ins/test/ins_config.prototxt";
  crdc::airi::util::get_proto_from_file(config_file_path, &config);
  config2 = config.component_config()[0];
  config2.set_config_file("../../../../modules/ins_drivers/params/drivers/ins/test/ins.prototxt");
  InsSensor s(config2);
  EXPECT_EQ("INS", s.get_name());
}

TEST_F(InsDriverTest, init_parser_test) {
  setenv("CRDC_WS", ".", 1);
  InsComponentConfig config;
  InsSensor s(config);
  EXPECT_EQ(true, s.init_parser());
}

TEST_F(InsDriverTest, init_input_test) {
  setenv("CRDC_WS", ".", 1);
  InsComponentConfig config;
  InsSensor s(config);
  EXPECT_EQ(true, s.init_input());
}

} // namespace airi
} // namespace crdc

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
