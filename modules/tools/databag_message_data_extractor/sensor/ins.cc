#include "tools/databag_message_data_extractor/sensor/ins.h"

namespace crdc {
namespace airi {
namespace ins {

#ifndef WITH_ROS2
void INS::parse_ins_data(const std::shared_ptr<crdc::airi::Ins> &message,
                         CanSignalIns570d *can_signal_ins570d) {
    // std::cout << "Entering parse_ins_data " << std::endl;
    can_signal_ins570d->time_stamp = message->header().timestamp_sec();

    can_signal_ins570d->ACC_X = static_cast<float>(message->linear_acceleration().x());
    can_signal_ins570d->ACC_Y = static_cast<float>(message->linear_acceleration().y());
    can_signal_ins570d->ACC_Z = static_cast<float>(message->linear_acceleration().z());

    can_signal_ins570d->GYR0_X = static_cast<float>(message->angular_velocity().x());
    can_signal_ins570d->GYR0_Y = static_cast<float>(message->angular_velocity().y());
    can_signal_ins570d->GYR0_Z = static_cast<float>(message->angular_velocity().z());

    can_signal_ins570d->INS_PitchAngle = static_cast<float>(message->euler_angles().y());
    can_signal_ins570d->INS_RollAngle = static_cast<float>(message->euler_angles().x());
    can_signal_ins570d->INS_HeadingAngle = static_cast<float>(message->euler_angles().z());

    // can_signal_ins570d->INS_LocatHeight =  message->position().set_height();
    can_signal_ins570d->INS_LocatHeight = message->position().height();
    can_signal_ins570d->INS_Latitude = message->position().lat();
    can_signal_ins570d->INS_Longitude = message->position().lon();

    can_signal_ins570d->INS_NorthSpd = static_cast<float>(message->linear_velocity().y());
    can_signal_ins570d->INS_EastSpd = static_cast<float>(message->linear_velocity().x());
    can_signal_ins570d->INS_ToGroundSpd = static_cast<float>(message->linear_velocity().z());

    // 0x506 INS_DataInfo
    can_signal_ins570d->INS_GpsFlag_Pos = 0;
    can_signal_ins570d->INS_NumSV = 0;
    can_signal_ins570d->INS_GpsFlag_Heading = 0;
    can_signal_ins570d->INS_Gps_Age = 0;
    can_signal_ins570d->INS_Car_Status = 0;
    can_signal_ins570d->INS_Status = 0;

    // 0x507
    can_signal_ins570d->INS_Std_Lat = 0;
    can_signal_ins570d->INS_Std_Lon = 0;
    can_signal_ins570d->INS_Std_LocatHeight = 0;
    can_signal_ins570d->INS_Std_Heading = 0;

    // integral for 0x505 INS_Speed, to get pose for origon
    can_signal_ins570d->HR_NorthDis = 0;
    can_signal_ins570d->HR_EastDis = 0;
    can_signal_ins570d->HR_ToGroundDis = 0;
}
#else
void INS::parse_ins_data(const std::shared_ptr<sensor_msg::msg::Ins> &message,
                         CanSignalIns570d *can_signal_ins570d) {
    can_signal_ins570d->time_stamp = message->header.timestamp_sec;

    can_signal_ins570d->ACC_X = static_cast<float>(message->linear_acceleration.x);
    can_signal_ins570d->ACC_Y = static_cast<float>(message->linear_acceleration.y);
    can_signal_ins570d->ACC_Z = static_cast<float>(message->linear_acceleration.z);

    can_signal_ins570d->GYR0_X = static_cast<float>(message->angular_velocity.x);
    can_signal_ins570d->GYR0_Y = static_cast<float>(message->angular_velocity.y);
    can_signal_ins570d->GYR0_Z = static_cast<float>(message->angular_velocity.z);

    can_signal_ins570d->INS_PitchAngle = static_cast<float>(message->euler_angles.y);
    can_signal_ins570d->INS_RollAngle = static_cast<float>(message->euler_angles.x);
    can_signal_ins570d->INS_HeadingAngle = static_cast<float>(message->euler_angles.z);

    can_signal_ins570d->INS_LocatHeight = message->position.height;
    can_signal_ins570d->INS_Latitude = message->position.lat;
    can_signal_ins570d->INS_Longitude = message->position.lon;

    can_signal_ins570d->INS_NorthSpd = static_cast<float>(message->linear_velocity.y);
    can_signal_ins570d->INS_EastSpd = static_cast<float>(message->linear_velocity.x);
    can_signal_ins570d->INS_ToGroundSpd = static_cast<float>(message->linear_velocity.z);

    // 0x506 INS_DataInfo
    can_signal_ins570d->INS_GpsFlag_Pos = 0;
    can_signal_ins570d->INS_NumSV = 0;
    can_signal_ins570d->INS_GpsFlag_Heading = 0;
    can_signal_ins570d->INS_Gps_Age = 0;
    can_signal_ins570d->INS_Car_Status = 0;
    can_signal_ins570d->INS_Status = 0;

    // 0x507
    can_signal_ins570d->INS_Std_Lat = 0;
    can_signal_ins570d->INS_Std_Lon = 0;
    can_signal_ins570d->INS_Std_LocatHeight = 0;
    can_signal_ins570d->INS_Std_Heading = 0;

    // integral for 0x505 INS_Speed, to get pose for origon
    can_signal_ins570d->HR_NorthDis = 0;
    can_signal_ins570d->HR_EastDis = 0;
    can_signal_ins570d->HR_ToGroundDis = 0;
}
#endif

void INS::ins570d_struct2str(char *dst, CanSignalIns570d *src) {
    static int32_t frame_id = 0;
    std::string timestamp = std::to_string(src->time_stamp);
    timestamp.erase(std::remove(timestamp.begin(), timestamp.end(), '.'), timestamp.end());

    sprintf(dst, "%09d %.16s %f %f %f %f %f %f %f %f %f %lf %lu %.8lf %.8lf \
            %f %f %f %d %d %d %d %d %d %f %f %f %f %.8lf %.8lf %.8lf\n ", frame_id,
            timestamp.c_str(), src->ACC_X, src->ACC_Y, src->ACC_Z, src->GYR0_X,
            src->GYR0_Y, src->GYR0_Z, src->INS_PitchAngle, src->INS_RollAngle,
            src->INS_HeadingAngle, src->INS_LocatHeight, src->INS_Time,
            src->INS_Latitude, src->INS_Longitude, src->INS_NorthSpd,
            src->INS_EastSpd, src->INS_ToGroundSpd, src->INS_GpsFlag_Pos,
            src->INS_NumSV, src->INS_GpsFlag_Heading, src->INS_Gps_Age,
            src->INS_Car_Status, src->INS_Status, src->INS_Std_Lat,
            src->INS_Std_Lon, src->INS_Std_LocatHeight, src->INS_Std_Heading,
            src->HR_EastDis, src->HR_NorthDis, src->HR_ToGroundDis);
    frame_id++;
}

}  // namespace ins
}  // namespace airi
}  // namespace crdc
