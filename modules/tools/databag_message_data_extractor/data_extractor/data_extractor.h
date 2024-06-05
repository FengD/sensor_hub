// Copyright (C) 2021 FengD
// License: Modified BSD Software License Agreement
// Author: Binqi Fan,  Jianfei Jiang
// Description: basic cyber tool

#ifndef MODULES_TOOLS_DATABAG_MESSAGE_DATA_EXTRACTOR_DATA_EXTRACTOR_DATA_EXTRACTOR_H_
#define MODULES_TOOLS_DATABAG_MESSAGE_DATA_EXTRACTOR_DATA_EXTRACTOR_DATA_EXTRACTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#ifndef WITH_ROS2
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_reader.h"
#include "cyber/record/record_base.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/sensor_proto/eth_packet.pb.h"
#include "cyber/sensor_proto/image.pb.h"
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
#include "sensor_msg/msg/image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#endif
#include "limits"
#include "memory"
#include "string"
#include "sys/stat.h"
#include "tools/databag_message_data_extractor/sensor/ins.h"
#include "utility"
#include "vector"
#define CAMERA_TYPE "AR0233"
#define SENSORHUB_CHANNEL_LEFT "camera/TDA4_AR0233_IMAGE/0"
#define SENSORHUB_CHANNEL_FRONT "camera/TDA4_AR0233_IMAGE/1"
#define SENSORHUB_CHANNEL_RIGHT "camera/TDA4_AR0233_IMAGE/2"
#define SENSORHUB_CHANNEL_REAR "camera/TDA4_AR0233_IMAGE/3"
#define SENSORHUB_CHANNEL_FRONT_VIEW "camera/TDA4_AR0233_IMAGE/4"
#define INS_TYPE "570D"
#define SENSORHUB_CHANNEL_INS "INS_DATA"

namespace crdc {
namespace airi {
namespace data_extractor {

class DataExtractor {
 public:
    std::vector<std::string> camera_names_;
    std::vector<std::string> input_camera_channel_names_;

    std::vector<std::string> ins_names_;
    std::vector<std::string> input_ins_channel_names_;

 public:
    DataExtractor() = default;
    virtual ~DataExtractor() = default;

#ifndef WITH_ROS2
    /**
     * @brief extract image and ins data
     * @return status
     */
    int extract_frame(const std::string pcaket_path,
                     const std::string output_path, const std::string type);

    /**
     * @brief cyber init
     * @return status
     */
    int init_cyber_env();

    /**
     * @brief cyber shutdown
     * @return none
     */
    void wait_for_shutdown();

    /**
     * @brief extract ins data
     * @return none
     */
    void extract_ins(const std::shared_ptr<crdc::airi::Ins> &ptr_ins,
                     const std::string &file_name);

    /**
     * @brief extract camera image2 data
     * @return none
     */
    void extract_camera(const std::shared_ptr<crdc::airi::Image2> &ptr_img2,
                        const std::string &cam_channel_name,
                        const std::string &output_path,
                        const std::string &type);

#else
    /**
     * @brief extract image and ins data
     * @return status
     */
    void extract_frame(const std::string pcaket_path,
                     const std::string output_path, const std::string type);

    /**
     * @brief extract custom image data
     * @return none
     */
    void extract_custom_image(const std::string output_path, const std::string type);

    /**
     * @brief extract ROS2 image data
     * @return none
     */
    void extract_ros2_image(const std::string output_path, const std::string type);
#endif
    /**
     * @brief double to string
     * @return string
     */
    std::string double_to_string(const double value) {
        std::ostringstream out;
        out.precision(std::numeric_limits<double>::digits10);
        out << value;
        std::string res = std::move(out.str());
        auto pos = res.find('.');
        std::string time = res.substr(0, pos) + res.substr(pos + 1);
        return time;
    }

    void split_string(const std::string &s, std::vector<std::string> &v, const std::string &c) {
        std::string::size_type pos1, pos2;
        pos2 = s.find(c);
        pos1 = 0;
        while (std::string::npos != pos2) {
            v.push_back(s.substr(pos1, pos2 - pos1));
            pos1 = pos2 + c.size();
            pos2 = s.find(c, pos1);
        }
        if (pos1 != s.length())
            v.push_back(s.substr(pos1));
    }

    /**
     * @brief write file
     * @return none
     */
    void write_file(std::string file_name, std::string file_path, void *data) {
        if (file_path[file_path.size() - 1] != '/') {
            file_path.push_back('/');
        }
        std::string filename = file_path + file_name;
        FILE *fp = fopen(filename.c_str(), "wb+");
        if (NULL == fp) {
            fprintf(stderr, "Input: file opened Failed \n");
        }
        fwrite(data, 1, data_size, fp);
        fclose(fp);
    }

    /**
     * @brief write ins file
     * @return none
     */
    void write_ins_file(std::string file_name, std::string file_path, char *data) {
        if (file_path[file_path.size() - 1] != '/') {
            file_path.push_back('/');
        }
        std::string filename = file_path + file_name;
        FILE *fp = fopen(filename.c_str(), "a");
        if (NULL == fp) {
            fprintf(stderr, "Input: file opened Failed \n");
        }
        fprintf(fp, "%s", data);
        fclose(fp);
    }

 private:
    std::mutex camera_mutex_;
    std::mutex ins_mutex_;
    const std::string sensor_msg_type_image_ = "sensor_msgs/msg/Image";
    const std::string type_custom_image_ = "sensor_msg/msg/Image";
    const std::string sensor_msg_type_ins_ = "sensor_msg/msg/Ins";
    std::shared_ptr<ins::INS> ins;
    struct crdc::airi::ins::CanSignalIns570d can_signal_ins570d;
#ifndef WITH_ROS2
    std::shared_ptr<apollo::cyber::Node> listener_node;
#else
    sensor_msgs::msg::Image image_msg_;
    sensor_msg::msg::Image custom_image_msg_;
    sensor_msg::msg::Ins ins_msg_;
    std::string channel_name, timestamp, camera_channel_name, image_name;
    int height, width;
#endif
    const std::string type_png = "png";
    const std::string type_jpg = "jpg";
    const std::string extension_jpg = ".jpg";
    const std::string extension_png = ".png";
    const std::string file_separator = "/";
    const std::string under_line = "_";
    const std::string ins_file_name = "INS_DATA.txt";
    int32_t srv_flag = 0;
    std::string processed_file_folder;
    std::string readfile;
    int32_t data_size = 1920 * 1280 * 1.5;
    unsigned char *input_data = (unsigned char *)malloc(data_size);
    char dr_info[2048];
};

}  // namespace data_extractor
}  // namespace airi
}  // namespace crdc

#endif  // MODULES_TOOLS_DATABAG_MESSAGE_DATA_EXTRACTOR_DATA_EXTRACTOR_DATA_EXTRACTOR_H_
