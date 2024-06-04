#include "tools/databag_message_data_extractor/sensor/point_cloud.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace crdc {
namespace airi {
namespace pcd {

PointCloud::PointCloud(const std::string &input_file, const std::string &save_path,
               const std::string &format, const std::vector<std::string> &channel_name) {
    input_file_ = input_file;
    save_path_ = save_path;
    format_ = format;
    channel_name_ = channel_name;
    LOG(INFO) <<  "input_file: "<< input_file_ << " save_path: " <<  save_path_ << " format: "
                << format_;
#ifndef WITH_ROS2
    save();
#else
    save_ros2_pcd();
#endif
}

template <typename T>
void PointCloud::save_pcd(const T &cloud) {
    if (format_.compare(format_ascii_) == 0) {
        pcl::io::savePCDFileASCII(file_name_, cloud);
    } else if (format_.compare(format_binary_) == 0) {
        pcl::io::savePCDFileBinary(file_name_, cloud);
    } else {
        LOG(FATAL) << "please enter the correct format";
    }
}

#ifndef WITH_ROS2
void PointCloud::save() {
    apollo::cyber::record::RecordReader reader(input_file_);
    apollo::cyber::record::RecordMessage message;
    int frame = 0;
    std::string channel_name;
    while (reader.ReadMessage(&message)) {
        std::shared_ptr<PointCloud2> ptr_pcd2 = std::make_shared<PointCloud2>();
        ptr_pcd2->ParseFromString(message.content);
        std::string timestamp = std::to_string(ptr_pcd2->header().timestamp_sec());
        timestamp.erase(std::remove(timestamp.begin(), timestamp.end(), '.'), timestamp.end());
        for (auto i = 0; i < channel_name_.size(); ++i) {
          auto pos = channel_name_.at(i).rfind('/');
          if (pos == -1) {
            channel_name = channel_name_.at(i);
          } else {
            channel_name = channel_name_.at(i).substr(pos + 1);
          }
          if (message.channel_name == channel_name_.at(i)) {
            file_name_ = save_path_ + message.channel_name + file_separator_ +
                         channel_name + underline_ + timestamp + pcd_extension_;
            transform_to_pcd(ptr_pcd2, channel_name);
          }
        }
        frame++;
    }
    LOG(INFO) << "Total frame is " << frame;
}

void PointCloud::transform_to_pcd(const std::shared_ptr<PointCloud2> &pcd2_msg,
                                  const std::string &channel_name) {
  std::unordered_map<std::string, std::vector<double>> fields_data;
  for (uint32_t h = 0; h < pcd2_msg->height(); ++h) {
    auto data = pcd2_msg->data().data() + pcd2_msg->row_step() * h;
        for (uint32_t w = 0; w < pcd2_msg->width(); ++w) {
            for (auto it_field = pcd2_msg->fields().begin();
                it_field != pcd2_msg->fields().end(); ++it_field) {
                auto p = data + it_field->offset();
                switch (it_field->datatype()) {
                    case crdc::airi::PointField_PointFieldType_INT8:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<int8_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_INT16:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<int16_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_INT32:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<int32_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_UINT8:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<uint8_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_UINT16:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<uint16_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_UINT32:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<uint32_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_FLOAT32:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<float*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_FLOAT64:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<double*>(const_cast<char*>(p))));
                    break;
                    default:
                    LOG(WARNING) << "Unknown pcl field type: " << int(it_field->datatype());
                    break;
                }
            }
            data += pcd2_msg->point_step();
        }
    }
    auto iter = fields_data.begin();
    int size = iter->second.size();
    if (channel_name.find(channel_type_lidar_) != -1) {
        pcl::PointXYZI point;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        for (int i = 0; i < size; i++) {
            point.x = fields_data["x"][i];
            point.y = fields_data["y"][i];
            point.z = fields_data["z"][i];
            point.intensity = fields_data["intensity"][i];
            cloud.push_back(point);
        }
        save_pcd(cloud);
    } else if (channel_name.find(channel_type_radar_) != -1) {
        POINT_TYPE_RADAR point;
        pcl::PointCloud<POINT_TYPE_RADAR> cloud;
        for (int i = 0; i < size; i++) {
            point.x = fields_data["x"][i];
            point.y = fields_data["y"][i];
            point.z = fields_data["z"][i];
            point.doppler = fields_data["doppler"][i];
            point.range = fields_data["range"][i];
            point.snr = fields_data["snr"][i];
            point.power = fields_data["power"][i];
            point.azimuth = fields_data["azimuth"][i];
            point.elevation = fields_data["elevation"][i];
            point.elevation_bin = fields_data["elevation_bin"][i];
            point.azimuth_bin = fields_data["azimuth_bin"][i];
            point.doppler_bin = fields_data["doppler_bin"][i];
            point.range_bin = fields_data["range_bin"][i];
            point.power_bin = fields_data["power_bin"][i];
            cloud.push_back(point);
        }
        save_pcd(cloud);
    }
    fields_data.clear();
}

void PointCloud::transform_to_pcd(const std::shared_ptr<PointCloud2> &pcd2_msg,
                                  const std::string &format,
                                  const std::string &file_name) {
  std::unordered_map<std::string, std::vector<double>> fields_data;
  for (uint32_t h = 0; h < pcd2_msg->height(); ++h) {
    auto data = pcd2_msg->data().data() + pcd2_msg->row_step() * h;
        for (uint32_t w = 0; w < pcd2_msg->width(); ++w) {
            for (auto it_field = pcd2_msg->fields().begin();
                it_field != pcd2_msg->fields().end(); ++it_field) {
                auto p = data + it_field->offset();
                switch (it_field->datatype()) {
                    case crdc::airi::PointField_PointFieldType_INT8:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<int8_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_INT16:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<int16_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_INT32:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<int32_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_UINT8:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<uint8_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_UINT16:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<uint16_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_UINT32:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<uint32_t*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_FLOAT32:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<float*>(const_cast<char*>(p))));
                    break;
                    case crdc::airi::PointField_PointFieldType_FLOAT64:
                    fields_data[it_field->name()].push_back(
                            *(reinterpret_cast<double*>(const_cast<char*>(p))));
                    break;
                    default:
                    LOG(WARNING) << "Unknown pcl field type: " << int(it_field->datatype());
                    break;
                }
            }
            data += pcd2_msg->point_step();
        }
    }
    auto iter = fields_data.begin();
    int size = iter->second.size();
    pcl::PointXYZI point;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for (int i = 0; i < size; i++) {
        point.x = fields_data["x"][i];
        point.y = fields_data["y"][i];
        point.z = fields_data["z"][i];
        point.intensity = fields_data["intensity"][i];
        cloud.push_back(point);
    }

    fields_data.clear();
    if (format.compare(format_ascii_) == 0) {
        pcl::io::savePCDFileASCII(file_name, cloud);
    } else if (format.compare(format_binary_) == 0) {
        pcl::io::savePCDFileBinary(file_name, cloud);
    } else {
        LOG(FATAL) << "please enter the correct format";
    }
}
#else
void PointCloud::save_ros2_pcd() {
    rosbag2_cpp::Reader reader_;
    reader_.open(input_file_);  // open file
    std::unordered_map<std::string, std::string> topic_and_type;
    auto topics = reader_.get_all_topics_and_types();
    for (const auto & topic_with_type : topics) {
        topic_and_type[topic_with_type.name] = topic_with_type.type;
    }
    std::string channel_name;
    std::string timestamp;
    while (reader_.has_next()) {  // if not end
        auto bag_message = reader_.read_next();
        if (0 == topic_and_type[bag_message->topic_name].compare(sensor_msg_type_)) {
            rclcpp::SerializedMessage serialization_message(*bag_message->serialized_data);
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_lidar_message;
            serialization_lidar_message.deserialize_message(&serialization_message, &lidar_message);
            timestamp.clear();
            uint64_t sec = lidar_message.header.stamp.sec;
            uint64_t nanosec = lidar_message.header.stamp.nanosec;
            uint64_t stamp_int = sec * 1000000 + nanosec / 1000;
            timestamp = std::to_string(stamp_int);
            channel_name.clear();
            file_name_.clear();
            channel_name = bag_message->topic_name;
            std::shared_ptr<PointCloud2> pcd2_msg = std::make_shared<PointCloud2>(lidar_message);
            extract_point_cloud(pcd2_msg, channel_name, timestamp);
        } else if (0 == topic_and_type[bag_message->topic_name].compare(custom_msg_type_)) {
            rclcpp::SerializedMessage serialization_message(*bag_message->serialized_data);
            rclcpp::Serialization<sensor_msg::msg::PointCloud2> serialization_lidar_message;
            serialization_lidar_message.deserialize_message(&serialization_message,
                                                            &lidar_message_custom);
            timestamp.clear();
            timestamp = std::to_string(lidar_message_custom.header.timestamp_sec);
            timestamp.erase(std::remove(timestamp.begin(), timestamp.end(), '.'), timestamp.end());
            channel_name.clear();
            channel_name = bag_message->topic_name;
            file_name_.clear();
            std::shared_ptr<CustomPointCloud2> pcd2_msg
            = std::make_shared<CustomPointCloud2>(lidar_message_custom);
            extract_point_cloud(pcd2_msg, channel_name, timestamp);
        } else if (0 == topic_and_type[bag_message->topic_name].
                                                    compare("geometry_msgs/msg/PoseStamped")) {
            rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization_aduinfo;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization_aduinfo.deserialize_message(&extracted_serialized_msg, &adu_msg_);
            std::string folder_path = save_path_ + bag_message->topic_name;
            if (system(("mkdir -p " + folder_path).c_str())) {
              LOG(WARNING) << "[DataExtractor] Failed to create dir " << folder_path;
            }
            file_name_ = folder_path + file_separator_ + "adu_result.txt";
            result_str_ = result_str_ +
                    "{\"timestamp\":" + std::to_string(
                            static_cast<uint64_t>(adu_msg_.header.stamp.sec) * 1000000 +
                            static_cast<uint64_t>(adu_msg_.header.stamp.nanosec) / 1000) +
                    ",\"pose.position.x\":" + std::to_string(adu_msg_.pose.position.x) +
                    ",\"pose.position.y\":" + std::to_string(adu_msg_.pose.position.y) +
                    ",\"pose.position.z\":" + std::to_string(adu_msg_.pose.position.z) +
                    ",\"pose.orientation.x\":" + std::to_string(adu_msg_.pose.orientation.x) +
                    ",\"pose.orientation.y\":" + std::to_string(adu_msg_.pose.orientation.y) +
                    ",\"pose.orientation.z\":" + std::to_string(adu_msg_.pose.orientation.z) +
                    ",\"pose.orientation.w\":" + std::to_string(adu_msg_.pose.orientation.w) + "}";

            std::ofstream model_result_file(file_name_);
            result_str_ = result_str_ + "\n";
            model_result_file << result_str_;
            model_result_file.close();
        }
    }
    reader_.close();
}

template <typename T>
void PointCloud::extract_point_cloud(T &pcd2_msg,
                                     std::string channel_name,
                                     std::string timestamp) {
    for (size_t i = 0; i < channel_name_.size(); ++i) {
        if (channel_name == channel_name_.at(i)) {
            auto pos = channel_name.rfind('/');
            if (pos == std::string::npos) {
                channel_name = channel_name_.at(i);
            } else {
                channel_name = channel_name_.at(i).substr(pos + 1);
            }
            std::string folder_path = save_path_ + channel_name_.at(i);
            if (system(("mkdir -p " + folder_path).c_str())) {
              LOG(WARNING) << "[DataExtractor] Failed to create dir " << folder_path;
            }
            file_name_ = folder_path + file_separator_ +
                        channel_name + underline_ + timestamp + pcd_extension_;
            extract_points(pcd2_msg, channel_name);
        }
    }
}

template <typename T>
void PointCloud::extract_points(const T &pcd2_msg,
                                  const std::string &channel_name) {
    if (channel_name.find(channel_type_lidar_) != std::string::npos) {
        pcl::PointXYZI point;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        int point_count = 0;
        for (uint32_t h = 0; h < pcd2_msg->height; ++h) {
            auto data = pcd2_msg->data.data() + pcd2_msg->row_step * h;
            for (uint32_t w = 0; w < pcd2_msg->width; ++w) {
                auto p = data + 8;
                memcpy(&point, p, sizeof(pcl::PointXYZI));
                cloud.push_back(point);
                data += pcd2_msg->point_step;
                ++point_count;
            }
        }
        if (point_count > 0) {
            save_pcd(cloud);
        }
    } else if (channel_name.find(channel_type_radar_) != std::string::npos) {
        POINT_TYPE_RADAR point;
        pcl::PointCloud<POINT_TYPE_RADAR> cloud;
        int point_count = 0;
        for (uint32_t h = 0; h < pcd2_msg->height; ++h) {
            auto data = pcd2_msg->data.data() + pcd2_msg->row_step * h;
            for (uint32_t w = 0; w < pcd2_msg->width; ++w) {
                auto p = data + 8;
                memcpy(&point, p, sizeof(POINT_TYPE_RADAR));
                cloud.push_back(point);
                data += pcd2_msg->point_step;
                ++point_count;
            }
        }
        if (point_count > 0) {
            save_pcd(cloud);
        }
    }
}

#endif

}  // namespace pcd
}  // namespace airi
}  // namespace crdc
