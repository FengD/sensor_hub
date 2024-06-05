// Copyright (C) 2021 FengD Inc.
// License: Modified BSD Software License Agreement
// Author: shichong.wang
// Description: ins parser for asensing ins570d device

#pragma once
#include <memory>
#include <string>
#include "common/common.h"
#include "ins_drivers/parser/parser.h"


namespace crdc {
namespace airi {

#define SIGNAL_LENGTH 8

struct InsSignal {
  int startBit;
  int length;
  double factor;
  float offset;
  float maximum;
  float minimum;
  int dataType;
  int is_unsigned;
  InsSignal() {
    startBit = 0;
    length = 0;
    factor = 0;
    offset = 0;
    maximum = 0;
    minimum = 0;
    dataType = 0;
    is_unsigned = 0;
  }
};

struct InsFrame {
  uint32_t frame_id;
  uint32_t frame_flag;
  uint8_t frame_len;
  uint8_t frame_data[8];
  InsFrame() {}
};

struct CanSignalIns570d {
  // DR old info
  uint64_t time_stamp;  // timestamp from linux system
  // 0x500
  float ACC_X;
  float ACC_Y;
  float ACC_Z;
  // 0x501
  float GYR0_X;
  float GYR0_Y;
  float GYR0_Z;
  // 0x502
  float INS_PitchAngle;
  float INS_RollAngle;
  float INS_HeadingAngle;
  // 0x503 height and time
  double INS_LocatHeight;
  int64_t INS_Time;
  // 0x504 latitude Longitude
  double INS_Latitude;
  double INS_Longitude;
  // 0x505 INS_Speed
  float INS_NorthSpd;
  float INS_EastSpd;
  float INS_ToGroundSpd;
  // 0x506 INS_DataInfo
  int INS_GpsFlag_Pos;
  int INS_NumSV;
  int INS_GpsFlag_Heading;
  int INS_Gps_Age;
  int INS_Car_Status;
  int INS_Status;
  // 0x507
  float INS_Std_Lat;
  float INS_Std_Lon;
  float INS_Std_LocatHeight;
  float INS_Std_Heading;
  // integral for 0x505 INS_Speed, to get pose for origon
  double HR_NorthDis;
  double HR_EastDis;
  double HR_ToGroundDis;
  CanSignalIns570d() {}
};

class InsParser570d : public InsParser {
 public:
  InsParser570d();
  virtual ~InsParser570d() = default;

  bool init_ins_parser() override;

  uint64_t get_packet_timestamp(const Packet* packet) override;
  void parse_ins_can_frame(char* can_frame_) override;
  std::string get_name() const override {
    return "InsParser570d";
  }

 private:
  // 0x500
  InsSignal ACC_X = InsSignal();
  InsSignal ACC_Y = InsSignal();
  InsSignal ACC_Z = InsSignal();
  // 0x501
  InsSignal GYR0_X = InsSignal();
  InsSignal GYR0_Y = InsSignal();
  InsSignal GYR0_Z = InsSignal();
  // 0x502
  InsSignal INS_PitchAngle = InsSignal();
  InsSignal INS_RollAngle = InsSignal();
  InsSignal INS_HeadingAngle = InsSignal();
  // 0x503 height and time
  InsSignal INS_LocatHeight = InsSignal();
  InsSignal INS_Time = InsSignal();
  // 0x504 latitude Longitude
  InsSignal INS_Latitude = InsSignal();
  InsSignal INS_Longitude = InsSignal();
  // 0x505 INS_Speed
  InsSignal INS_NorthSpd = InsSignal();
  InsSignal INS_EastSpd = InsSignal();
  InsSignal INS_ToGroundSpd = InsSignal();
  // 0x506 INS_DataInfo
  InsSignal INS_GpsFlag_Pos = InsSignal();
  InsSignal INS_NumSV = InsSignal();
  InsSignal INS_GpsFlag_Heading = InsSignal();
  InsSignal INS_Gps_Age = InsSignal();
  InsSignal INS_Car_Status = InsSignal();
  InsSignal INS_Status = InsSignal();
  // 0x507
  InsSignal INS_Std_Lat = InsSignal();
  InsSignal INS_Std_Lon = InsSignal();
  InsSignal INS_Std_LocatHeight = InsSignal();
  InsSignal INS_Std_Heading = InsSignal();

  int init_ins_singal(InsSignal* s, const double singal_[SIGNAL_LENGTH]);
  double unpack_ins_signal(InsSignal s, uint8_t* data);
  void parse_ins_can_by_frame_id(CanSignalIns570d can_signal_ins570d);
  void parse_ins_can_frame_to_struct(InsFrame* frame,
                                     CanSignalIns570d* signal);
};

}  // namespace airi
}  // namespace crdc
