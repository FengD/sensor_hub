// Copyright (C) 2023 Hirain Technologies Inc.
// License: Modified BSD Software License Agreement
// Author: Zilou Cao
// Description: ins parser for asensing ins570d device

#include "ins_drivers/parser/pcap/parser_pcap.h"

namespace crdc {
namespace airi {

unsigned int get_bit(const unsigned char *buff, int pos, int len) {
  unsigned int bits = 0;
  for (int i = pos; i < pos + len; ++i) {
    bits = (bits << 1) + ((buff[i/8] >> (7 - i % 8)) & 1u);
  }
  return bits;
}

float hex_to_float(const unsigned char *buff) {
  int i = 0; float value = 0.0;
  unsigned int flag_bit = get_bit(buff, i++, 1);  // get flag bit
  int exp_bit = get_bit(buff, i, 8) - 127;
  i += 8;
  std::once_flag s_flag;
  while (i < 32) {
    std::call_once(s_flag, [&]{
      value += 1 * pow(2, exp_bit--);
    });
    value += get_bit(buff , i++, 1) * pow(2, exp_bit--);
  }

  return flag_bit == 1 ? -value : value;
}

double hex_to_double(const unsigned char* buff) {
  double value = 0; unsigned int i = 0;
  unsigned int flag_bit = get_bit(buff, i++, 1);  // get flag bit
  int exp_bit = get_bit(buff, i, 11) - 1023;
  i += 11;
  std::once_flag s_flag;
  while (i < 64) {
    std::call_once(s_flag, [&]{
      value += 1 * pow(2, exp_bit--);
    });
    value += get_bit(buff, i++, 1) * pow(2, exp_bit--);
  }
  return flag_bit == 1 ? -value : value;
}

void InsParserPcap::parse_ins_can_frame(char* can_frame_) {
  std::memcpy(&ins_message_data_struct_, can_frame_, sizeof(ins_message_data_struct_));
  // Refer to the communication matrix for the offset
#ifdef WITH_ROS2
  ins_data_->proto_ins_data_->linear_acceleration.y =
        hex_to_float(ins_message_data_struct_.IMU570DStruct);
  ins_data_->proto_ins_data_->linear_acceleration.x =
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 4);
  ins_data_->proto_ins_data_->linear_acceleration.z =
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 8);

  ins_data_->proto_ins_data_->angular_velocity.y =
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 12);
  ins_data_->proto_ins_data_->angular_velocity.x =
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 16);
  ins_data_->proto_ins_data_->angular_velocity.z =
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 20);

  ins_data_->proto_ins_data_->position.lat =
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 14);
  ins_data_->proto_ins_data_->position.lon =
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 22);
  ins_data_->proto_ins_data_->position.height =
        hex_to_float(ins_message_data_struct_.GNSSDataStruct + 30);

  ins_data_->proto_ins_data_->euler_angles.y =
        hex_to_float(ins_message_data_struct_.INSStruct);
  ins_data_->proto_ins_data_->euler_angles.x =
        hex_to_float(ins_message_data_struct_.INSStruct + 4);
  ins_data_->proto_ins_data_->euler_angles.z =
        hex_to_float(ins_message_data_struct_.INSStruct + 8);

  ins_data_->proto_ins_data_->linear_velocity.y =
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 64);
  ins_data_->proto_ins_data_->linear_velocity.x =
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 72);
  ins_data_->proto_ins_data_->linear_velocity.z =
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 80);
#else
  ins_data_->proto_ins_data_->mutable_linear_acceleration()->set_y(
        hex_to_float(ins_message_data_struct_.IMU570DStruct));
  ins_data_->proto_ins_data_->mutable_linear_acceleration()->set_x(
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 4));
  ins_data_->proto_ins_data_->mutable_linear_acceleration()->set_z(
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 8));

      LOG(ERROR) << "parse_ins_can_frame";
  ins_data_->proto_ins_data_->mutable_angular_velocity()->set_y(
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 12));
  ins_data_->proto_ins_data_->mutable_angular_velocity()->set_x(
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 16));
  ins_data_->proto_ins_data_->mutable_angular_velocity()->set_z(
        hex_to_float(ins_message_data_struct_.IMU570DStruct + 20));

  ins_data_->proto_ins_data_->mutable_position()->set_lat(
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 14));
  ins_data_->proto_ins_data_->mutable_position()->set_lon(
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 22));
  ins_data_->proto_ins_data_->mutable_position()->set_height(
        hex_to_float(ins_message_data_struct_.GNSSDataStruct + 30));

  ins_data_->proto_ins_data_->mutable_euler_angles()->set_y(
        hex_to_float(ins_message_data_struct_.INSStruct));
  ins_data_->proto_ins_data_->mutable_euler_angles()->set_x(
        hex_to_float(ins_message_data_struct_.INSStruct + 4));
  ins_data_->proto_ins_data_->mutable_euler_angles()->set_z(
        hex_to_float(ins_message_data_struct_.INSStruct + 8));

  ins_data_->proto_ins_data_->mutable_linear_velocity()->set_y(
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 64));
  ins_data_->proto_ins_data_->mutable_linear_velocity()->set_x(
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 72));
  ins_data_->proto_ins_data_->mutable_linear_velocity()->set_z(
        hex_to_double(ins_message_data_struct_.GNSSDataStruct + 80));
#endif
}

}  // namespace airi
}  // namespace crdc
