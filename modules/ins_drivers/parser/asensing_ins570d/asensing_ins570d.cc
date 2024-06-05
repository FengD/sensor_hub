// Copyright (C) 2021 FengD Inc.
// License: Modified BSD Software License Agreement
// Author: shichong.wang
// Description: ins parser for asensing ins570d device

#include <arpa/inet.h>
#include "ins_drivers/parser/asensing_ins570d/asensing_ins570d.h"

namespace crdc {
namespace airi {

#define DEBUG_INS_570D

template <typename T>
double unpack_by_bit(InsSignal s, uint8_t* data, int& startIndex, int& shift,
                     T unpacked_value) {
  T tempValue = (T)(0);
  if (s.dataType) {
    for (int i = 0; i < s.length; i++) {
      tempValue =
          tempValue |
          ((((data[startIndex]) & ((uint8_t)(1) << shift)) >> shift) << i);
      shift++;
      if (shift == 8) {
        shift = 0;
        startIndex++;
      }
    }
  } else {
    for (int i = 0; i < s.length; i++) {
      tempValue =
          tempValue |
          ((((data[startIndex]) & ((uint8_t)(1) << shift)) >> shift) << i);
      shift++;
      if (shift == 8) {
        shift = 0;
        startIndex--;
      }
    }
  }
  unpacked_value = tempValue;
  return static_cast<double>(unpacked_value);
}

InsParser570d::InsParser570d() {
  // for INS 570D CAN
  // startbit, length, factor,  offset, max, min, type, is_unsigned
  // 0x500
  const double ACC_X_signal_[8] = {7, 16, 0.0001220703125, -4, 4, -4, 0, 1};
  const double ACC_Y_signal_[8] = {23, 16, 0.0001220703125, -4, 4, -4, 0, 1};
  const double ACC_Z_signal_[8] = {39, 16, 0.0001220703125, -4, 4, -4, 0, 1};
  // 0x501
  const double GYR0_X_signal_[8] = {7, 16, 0.0076293, -250, 250, -250, 0, 1};
  const double GYR0_Y_signal_[8] = {23, 16, 0.0076293, -250, 250, -250, 0, 1};
  const double GYR0_Z_signal_[8] = {39, 16, 0.0076293, -250, 250, -250, 0, 1};
  // 0x502
  const double INS_PitchAngle_signal_[8] = {7,   16,   0.010986, -360,
                                            360, -360, 0,        1};
  const double INS_RollAngle_signal_[8] = {23,  16,   0.010986, -360,
                                           360, -360, 0,        1};
  const double INS_HeadingAngle_signal_[8] = {39,  16,   0.010986, -360,
                                              360, -360, 0,        1};
  // 0x503 height and time
  const double INS_LocatHeight_signal_[8] = {7,     32,     0.001, -10000,
                                             10000, -10000, 0,     1};
  const double INS_Time_signal_[8] = {39, 32, 1, 0, 4.29497e9, 0, 0, 1};
  // 0x504 latitude Longitude
  const double INS_Latitude_signal_[8] = {7,   32,   0.0000001, -180,
                                          180, -180, 0,         1};
  const double INS_Longitude_signal_[8] = {39,  32,   0.0000001, -180,
                                           180, -180, 0,         1};
  // 0x505 INS_Speed
  const double INS_NorthSpd_signal_[8] = {7,   16,   0.0030517, -100,
                                          100, -100, 0,         1};
  const double INS_EastSpd_signal_[8] = {23,  16,   0.0030517, -100,
                                         100, -100, 0,         1};
  const double INS_ToGroundSpd_signal_[8] = {39,  16,   0.0030517, -100,
                                             100, -100, 0,         1};
  // 0x506 INS_DataInfo
  const double INS_GpsFlag_Pos_signal_[8] = {7, 8, 1, 0, 0, 255, 0, 1};
  const double INS_NumSV_signal_[8] = {15, 8, 1, 0, 0, 255, 0, 1};
  const double INS_GpsFlag_Heading_signal_[8] = {23, 8, 1, 0, 0, 255, 0, 1};
  const double INS_Gps_Age_signal_[8] = {31, 8, 1, 0, 0, 255, 0, 1};
  const double INS_Car_Status_signal_[8] = {39, 8, 1, 0, 0, 255, 0, 1};
  const double INS_Status_signal_[8] = {47, 8, 1, 0, 0, 255, 0, 1};
  // 0x507
  const double INS_Std_Lat_signal_[8] = {7, 16, 0.001, 0, 0, 65.535, 0, 1};
  const double INS_Std_Lon_signal_[8] = {23, 16, 0.001, 0, 0, 65.535, 0, 1};
  const double INS_Std_LocatHeight_signal_[8] = {39, 16,     0.001, 0,
                                                 0,  65.535, 0,     1};
  const double INS_Std_Heading_signal_[8] = {55, 16, 0.01, 0, 0, 655.35, 0, 1};

  init_ins_singal(&ACC_X, ACC_X_signal_);
  init_ins_singal(&ACC_Y, ACC_Y_signal_);
  init_ins_singal(&ACC_Z, ACC_Z_signal_);
  init_ins_singal(&GYR0_X, GYR0_X_signal_);
  init_ins_singal(&GYR0_Y, GYR0_Y_signal_);
  init_ins_singal(&GYR0_Z, GYR0_Z_signal_);
  init_ins_singal(&INS_PitchAngle, INS_PitchAngle_signal_);
  init_ins_singal(&INS_RollAngle, INS_RollAngle_signal_);
  init_ins_singal(&INS_HeadingAngle, INS_HeadingAngle_signal_);
  init_ins_singal(&INS_LocatHeight, INS_LocatHeight_signal_);
  init_ins_singal(&INS_Time, INS_Time_signal_);
  init_ins_singal(&INS_Latitude, INS_Latitude_signal_);
  init_ins_singal(&INS_Longitude, INS_Longitude_signal_);
  init_ins_singal(&INS_NorthSpd, INS_NorthSpd_signal_);
  init_ins_singal(&INS_EastSpd, INS_EastSpd_signal_);
  init_ins_singal(&INS_ToGroundSpd, INS_ToGroundSpd_signal_);
  init_ins_singal(&INS_GpsFlag_Pos, INS_GpsFlag_Pos_signal_);
  init_ins_singal(&INS_NumSV, INS_NumSV_signal_);
  init_ins_singal(&INS_GpsFlag_Heading, INS_GpsFlag_Heading_signal_);
  init_ins_singal(&INS_Gps_Age, INS_Gps_Age_signal_);
  init_ins_singal(&INS_Car_Status, INS_Car_Status_signal_);
  init_ins_singal(&INS_Status, INS_Status_signal_);
  init_ins_singal(&INS_Std_Lat, INS_Std_Lat_signal_);
  init_ins_singal(&INS_Std_Lon, INS_Std_Lon_signal_);
  init_ins_singal(&INS_Std_LocatHeight, INS_Std_LocatHeight_signal_);
  init_ins_singal(&INS_Std_Heading, INS_Std_Heading_signal_);
}

bool InsParser570d::init_ins_parser() { return true; }

uint64_t InsParser570d::get_packet_timestamp(const Packet* packet) { (void)packet; return 1; }

double InsParser570d::unpack_ins_signal(InsSignal s, uint8_t* data) {
  // --------------- START Unpacking Signal ------------------
  //   startBit                = s.startBit
  //   length                  = s.length
  //   desiredSignalByteLayout = s.dataType
  //   dataType                = s.is_unsigned
  //   factor                  = s.factor
  //   offset                  = s.offset
  //  -----------------------------------------------------------------------

  int startBit = s.startBit;
  {
    // if the motolora type <BEGENDIAN> the startbit needs to be recalculated
    if (!s.dataType) {
      int tmp1 = startBit / 8;
      int tmp2 = tmp1 * 8 + 7 - (startBit % 8) + s.length - 1;
      int tmp3 = tmp2 / 8;
      startBit = tmp3 * 8 + 7 - tmp2 % 8;
    }
  }

  int startIndex = startBit / 8;
  int shift = startBit % 8;

  double out_value = 0;
  int64_t bitValue = pow(2, s.length);
  double max, min;
  if (s.is_unsigned) {
    max = (bitValue - 1) * s.factor + s.offset;
    min = 0.0 * s.factor + s.offset;
  } else {
    max = (bitValue / 2.0 - 1.0);
    min = (-1.0) * max - 1.0;
    max = max * s.factor + s.offset;
    min = min * s.factor + s.offset;
  }

  if (s.length <= 8) {
    uint8_t unpacked_value = 0;
    out_value = unpack_by_bit(s, data, startIndex, shift, unpacked_value);
  } else if (s.length > 8 && s.length <= 16) {
    uint16_t unpacked_value = 0;
    out_value = unpack_by_bit(s, data, startIndex, shift, unpacked_value);
  } else if (s.length > 16 && s.length <= 32) {
    uint32_t unpacked_value = 0;
    out_value = unpack_by_bit(s, data, startIndex, shift, unpacked_value);
  } else if (s.length > 32) {
    uint64_t unpacked_value = 0;
    out_value = unpack_by_bit(s, data, startIndex, shift, unpacked_value);
  }

  // TODO(shichong.wang): mask for the signed value
  double result = static_cast<double>(out_value);
  result = (result * s.factor) + s.offset;
  if (result < min) {
    result = min;
  }

  if (result > max) {
    result = max;
  }

  return result;
}

int InsParser570d::init_ins_singal(InsSignal* s,
                                   const double singal_[SIGNAL_LENGTH]) {
  s->startBit = singal_[0];
  s->length = singal_[1];
  s->factor = singal_[2];
  s->offset = singal_[3];
  s->maximum = singal_[4];
  s->minimum = singal_[5];
  s->dataType = singal_[6];
  s->is_unsigned = singal_[7];

  return 0;
}

void InsParser570d::parse_ins_can_frame_to_struct(InsFrame* frame,
                                                  CanSignalIns570d* signal) {
  if (frame->frame_id == 0x500) {
    signal->ACC_X = unpack_ins_signal(ACC_X, frame->frame_data);
    signal->ACC_Y = unpack_ins_signal(ACC_Y, frame->frame_data);
    signal->ACC_Z = unpack_ins_signal(ACC_Z, frame->frame_data);
  } else if (frame->frame_id == 0x501) {
    signal->GYR0_X = unpack_ins_signal(GYR0_X, frame->frame_data);
    signal->GYR0_Y = unpack_ins_signal(GYR0_Y, frame->frame_data);
    signal->GYR0_Z = unpack_ins_signal(GYR0_Z, frame->frame_data);
  } else if (frame->frame_id == 0x502) {
    signal->INS_PitchAngle =
        unpack_ins_signal(INS_PitchAngle, frame->frame_data);
    signal->INS_RollAngle = unpack_ins_signal(INS_RollAngle, frame->frame_data);
    signal->INS_HeadingAngle =
        unpack_ins_signal(INS_HeadingAngle, frame->frame_data);
  } else if (frame->frame_id == 0x503) {
    signal->INS_LocatHeight =
        unpack_ins_signal(INS_LocatHeight, frame->frame_data);
    signal->INS_Time = unpack_ins_signal(INS_Time, frame->frame_data);
  } else if (frame->frame_id == 0x504) {
    signal->INS_Latitude = unpack_ins_signal(INS_Latitude, frame->frame_data);
    signal->INS_Longitude = unpack_ins_signal(INS_Longitude, frame->frame_data);
  } else if (frame->frame_id == 0x505) {
    signal->INS_NorthSpd = unpack_ins_signal(INS_NorthSpd, frame->frame_data);
    signal->INS_EastSpd = unpack_ins_signal(INS_EastSpd, frame->frame_data);
    signal->INS_ToGroundSpd =
        unpack_ins_signal(INS_ToGroundSpd, frame->frame_data);
  } else if (frame->frame_id == 0x506) {
    signal->INS_GpsFlag_Pos =
        unpack_ins_signal(INS_GpsFlag_Pos, frame->frame_data);
    signal->INS_NumSV = unpack_ins_signal(INS_NumSV, frame->frame_data);
    signal->INS_GpsFlag_Heading =
        unpack_ins_signal(INS_GpsFlag_Heading, frame->frame_data);
    signal->INS_Gps_Age = unpack_ins_signal(INS_Gps_Age, frame->frame_data);
    signal->INS_Car_Status =
        unpack_ins_signal(INS_Car_Status, frame->frame_data);
    signal->INS_Status = unpack_ins_signal(INS_Status, frame->frame_data);
  } else if (frame->frame_id == 0x507) {
    signal->INS_Std_Lat = unpack_ins_signal(INS_Std_Lat, frame->frame_data);
    signal->INS_Std_Lon = unpack_ins_signal(INS_Std_Lon, frame->frame_data);
    signal->INS_Std_LocatHeight =
        unpack_ins_signal(INS_Std_LocatHeight, frame->frame_data);
    signal->INS_Std_Heading =
        unpack_ins_signal(INS_Std_Heading, frame->frame_data);
  } else {
    printf("570D CAN ID number error.\n");
  }
}

void InsParser570d::parse_ins_can_frame(char* can_frame_) {
  CanSignalIns570d can_signal_ins570d = CanSignalIns570d();
  InsFrame can_frame = InsFrame();

  for (int i = 0; i < SIGNAL_LENGTH; i++) {
    // TODO(shichong.wang): replace "17" by macro
    memcpy(&can_frame, &can_frame_[i * 17], sizeof(can_frame));
    can_frame.frame_id = ntohl(can_frame.frame_id);
    can_frame.frame_flag = ntohl(can_frame.frame_flag);
#ifdef DEBUG_INS_570D
    printf("#can frame id use x = %x\n", can_frame.frame_id);
    printf("#can frame id use d = %d\n", can_frame.frame_id);
    printf("#can frame flag = %d\n", can_frame.frame_flag);
#endif

    parse_ins_can_frame_to_struct(&can_frame, &can_signal_ins570d);
  }

#ifdef WITH_ROS2
  auto& linear_velocity = ins_data_->proto_ins_data_->linear_velocity;
  linear_velocity.x = static_cast<double>(can_signal_ins570d.INS_EastSpd);
  linear_velocity.y = static_cast<double>(can_signal_ins570d.INS_NorthSpd);
  linear_velocity.z =
      static_cast<double>(can_signal_ins570d.INS_ToGroundSpd);

  auto& position = ins_data_->proto_ins_data_->position;
  position.lon = static_cast<double>(can_signal_ins570d.INS_Longitude);
  position.lat = static_cast<double>(can_signal_ins570d.INS_Latitude);
  position.height = static_cast<double>(can_signal_ins570d.INS_LocatHeight);

  auto& euler_angles = ins_data_->proto_ins_data_->euler_angles;
  euler_angles.x = static_cast<double>(can_signal_ins570d.INS_RollAngle);
  euler_angles.y = static_cast<double>(can_signal_ins570d.INS_PitchAngle);
  euler_angles.z = static_cast<double>(can_signal_ins570d.INS_HeadingAngle);

  auto& angular_velocity =
      ins_data_->proto_ins_data_->angular_velocity;
  angular_velocity.x = static_cast<double>(can_signal_ins570d.GYR0_X);
  angular_velocity.y = static_cast<double>(can_signal_ins570d.GYR0_Y);
  angular_velocity.z = static_cast<double>(can_signal_ins570d.GYR0_Z);

  auto& linear_acceleration =
      ins_data_->proto_ins_data_->linear_acceleration;
  linear_acceleration.x = static_cast<double>(can_signal_ins570d.ACC_X);
  linear_acceleration.y = static_cast<double>(can_signal_ins570d.ACC_Y);
  linear_acceleration.z = static_cast<double>(can_signal_ins570d.ACC_Z);
#else
  auto linear_velocity = ins_data_->proto_ins_data_->mutable_linear_velocity();
  linear_velocity->set_x(static_cast<double>(can_signal_ins570d.INS_EastSpd));
  linear_velocity->set_y(static_cast<double>(can_signal_ins570d.INS_NorthSpd));
  linear_velocity->set_z(
      static_cast<double>(can_signal_ins570d.INS_ToGroundSpd));

  auto position = ins_data_->proto_ins_data_->mutable_position();
  position->set_lon(static_cast<double>(can_signal_ins570d.INS_Longitude));
  position->set_lat(static_cast<double>(can_signal_ins570d.INS_Latitude));
  position->set_height(static_cast<double>(can_signal_ins570d.INS_LocatHeight));

  // TODO(shichong.wang): 570D is in degree
  auto euler_angles = ins_data_->proto_ins_data_->mutable_euler_angles();
  euler_angles->set_x(static_cast<double>(can_signal_ins570d.INS_RollAngle));
  euler_angles->set_y(static_cast<double>(can_signal_ins570d.INS_PitchAngle));
  euler_angles->set_z(static_cast<double>(can_signal_ins570d.INS_HeadingAngle));

  // TODO(shichong.wang): 570D is in degree/s, positive direction: front, right,
  // down
  auto angular_velocity =
      ins_data_->proto_ins_data_->mutable_angular_velocity();
  angular_velocity->set_x(static_cast<double>(can_signal_ins570d.GYR0_X));
  angular_velocity->set_y(static_cast<double>(can_signal_ins570d.GYR0_Y));
  angular_velocity->set_z(static_cast<double>(can_signal_ins570d.GYR0_Z));

  // TODO(shichong.wang): 570D is in g, positive direction: front, right, down
  auto linear_acceleration =
      ins_data_->proto_ins_data_->mutable_linear_acceleration();
  linear_acceleration->set_x(static_cast<double>(can_signal_ins570d.ACC_X));
  linear_acceleration->set_y(static_cast<double>(can_signal_ins570d.ACC_Y));
  linear_acceleration->set_z(static_cast<double>(can_signal_ins570d.ACC_Z));
#endif
}

}  // namespace airi
}  // namespace crdc
