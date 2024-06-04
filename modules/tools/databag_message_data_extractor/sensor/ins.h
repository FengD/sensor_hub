// Copyright (C) 2021 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Binqi Fan
// Description: parse ins data

#ifndef MODULES_TOOLS_DATABAG_MESSAGE_DATA_EXTRACTOR_SENSOR_INS_H_
#define MODULES_TOOLS_DATABAG_MESSAGE_DATA_EXTRACTOR_SENSOR_INS_H_

#include "memory"
#ifndef WITH_ROS2
#include "cyber/cyber.h"
#include "cyber/sensor_proto/ins.pb.h"
#else
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "sensor_msg/msg/ins.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#endif

namespace crdc {
namespace airi {
namespace ins {

template <typename T>
class StructInit {
 public:
    StructInit() {
        memset(this, -1, sizeof(T));
    }
};

struct CanSignalIns570d : StructInit<CanSignalIns570d> {
    // DR old info
    double time_stamp;
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
};

class INS {
 private:
    struct CanSignalIns570d can_signal_ins570d;

 public:
    INS() = default;
    virtual ~INS() = default;

    /**
     * @brief parse ins data
     * @return none
     */
#ifndef WITH_ROS2
    void parse_ins_data(const std::shared_ptr<crdc::airi::Ins> &message,
                        CanSignalIns570d *can_signal_ins570d);
#else
    void parse_ins_data(const std::shared_ptr<sensor_msg::msg::Ins> &message,
                         CanSignalIns570d *can_signal_ins570d);
#endif

    void ins570d_struct2str(char *dst, CanSignalIns570d *src);
};

}  // namespace ins
}  // namespace airi
}  // namespace crdc

#endif  // MODULES_TOOLS_DATABAG_MESSAGE_DATA_EXTRACTOR_SENSOR_INS_H_
