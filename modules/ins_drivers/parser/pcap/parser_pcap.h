// Copyright (C) 2023 Hirain Technologies Inc.
// License: Modified BSD Software License Agreement
// Author: Zilou Cao
// Description: ins parser for asensing ins570d device

#pragma once
#include <memory>
#include <string>
#include "common/common.h"
#include "ins_drivers/parser/parser.h"


namespace crdc {
namespace airi {

unsigned int get_bit(const unsigned char *buff, int pos, int len);
float hex_to_float(const unsigned char *buff);
double hex_to_double(const unsigned char* buff);

struct INSMessageDataStruct {
  unsigned char IMU570DStruct[32];
  unsigned char GNSSDataStruct[117];
  unsigned char INSStruct[102];
  unsigned char StatusINSStruct[10];
};

class InsParserPcap : public InsParser {
 public:
  InsParserPcap() = default;
  virtual ~InsParserPcap() = default;

  bool init_ins_parser() override { return true; };

  uint64_t get_packet_timestamp(const Packet* packet) override { (void)packet; return 1; };
  void parse_ins_can_frame(char* can_frame_) override;

  std::string get_name() const override {
    return "InsParserPcap";
  }

 private:
  INSMessageDataStruct ins_message_data_struct_;
};

}  // namespace airi
}  // namespace crdc
