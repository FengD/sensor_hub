#include "tools/databag_message_data_extractor/data_extractor/data_extractor.h"

namespace crdc {
namespace airi {
namespace data_extractor {

#ifndef WITH_ROS2
int DataExtractor::init_cyber_env() {
  apollo::cyber::Init("databag_message_data_extractor");
  listener_node = apollo::cyber::CreateNode("parse_node");

  camera_names_.push_back(CAMERA_TYPE);
  input_camera_channel_names_.push_back(SENSORHUB_CHANNEL_LEFT);
  input_camera_channel_names_.push_back(SENSORHUB_CHANNEL_FRONT);
  input_camera_channel_names_.push_back(SENSORHUB_CHANNEL_RIGHT);
  input_camera_channel_names_.push_back(SENSORHUB_CHANNEL_REAR);

  ins_names_.push_back(INS_TYPE);
  input_ins_channel_names_.push_back(SENSORHUB_CHANNEL_INS);
  return apollo::cyber::SUCC;
}

void DataExtractor::extract_ins(const std::shared_ptr<crdc::airi::Ins> &ptr_ins,
                              const std::string &file_name) {
  if (ptr_ins == nullptr) {
    throw("Fatal Error: ins msg is NULL");
  }
  std::shared_ptr<crdc::airi::ins::INS> ins;
  ins->parse_ins_data(ptr_ins, &can_signal_ins570d);
  ins->ins570d_struct2str(dr_info, &can_signal_ins570d);
  write_ins_file(ins_file_name, file_name, dr_info);
}

void DataExtractor::extract_camera(const std::shared_ptr<crdc::airi::Image2> &ptr_img2,
                 const std::string &cam_channel_name, const std::string &output_path,
                 const std::string &type) {
  std::string timestamp = std::to_string(ptr_img2->header().timestamp_sec());
  timestamp.erase(std::remove(timestamp.begin(), timestamp.end(), '.'), timestamp.end());
  void *data = static_cast<void *>(const_cast<char *>(ptr_img2->data().data()));
  int height = static_cast<int>(ptr_img2->height());
  int width = static_cast<int>(ptr_img2->width());

  std::string camera_channel_name;
  auto pos = cam_channel_name.rfind('/');
  if (pos == -1) {
    camera_channel_name = cam_channel_name;
  } else {
    camera_channel_name = cam_channel_name.substr(pos + 1);
  }
  std::string img_name;
  if (type == type_png) {
    img_name = output_path + file_separator + camera_channel_name + under_line
             + timestamp + extension_png;
    cv::Mat yuv(height * 3 / 2, width, CV_8UC1, data);
    cv::Mat png(height, width, CV_8UC3);
    cv::cvtColor(yuv, png, cv::COLOR_YUV2BGR_NV12);
    cv::imwrite(img_name, png);
  } else {
    img_name = output_path + file_separator + camera_channel_name + under_line
             + timestamp + extension_jpg;
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100);
    std::vector<uint8_t> compressed_buffer(ptr_img2->data().begin(),
                                           ptr_img2->data().end());
    cv::Mat image = cv::imdecode(compressed_buffer, CV_LOAD_IMAGE_UNCHANGED);
    if (!image.empty()) cv::imwrite(img_name, image, compression_params);
  }
}
#endif

#ifndef WITH_ROS2
int DataExtractor::extract_frame(const std::string pcaket_path, const std::string output_path,
                              const std::string type) {
  // init_data_folder(output_path);
  const std::string &camera_name = camera_names_[0];
  if (camera_name == "AR0233") {
    apollo::cyber::record::RecordReader reader(pcaket_path);
    apollo::cyber::record::RecordMessage message;
    int frame = 0;
    std::string channel_name;
    while (reader.ReadMessage(&message)) {
      channel_name = message.channel_name;
      if (channel_name == "INS_DATA") {
        std::shared_ptr<crdc::airi::Ins> ptr_ins(new crdc::airi::Ins);
        ptr_ins->ParseFromString(message.content);
        if (ptr_ins == nullptr) {
          throw("Fatal Error: ins msg is NULL");
        }
        ins->parse_ins_data(ptr_ins, &can_signal_ins570d);
        ins->ins570d_struct2str(dr_info, &can_signal_ins570d);
        write_ins_file(ins_file_name, output_path + message.channel_name, dr_info);
      } else if (channel_name.find("camera") != -1) {
        crdc::airi::Image2 img2;
        crdc::airi::Image2 *ptr_img2 = &img2;
        ptr_img2->ParseFromString(message.content);
        std::string timestamp = std::to_string(ptr_img2->header().timestamp_sec());
        timestamp.erase(std::remove(timestamp.begin(), timestamp.end(), '.'), timestamp.end());

        void *data = static_cast<void *>(const_cast<char *>(ptr_img2->data().data()));
        int height = static_cast<int>(ptr_img2->height());
        int width = static_cast<int>(ptr_img2->width());

        std::string camera_channel_name;
        auto pos = channel_name.rfind('/');
        if (pos == -1) {
          camera_channel_name = channel_name;
        } else {
          camera_channel_name = channel_name.substr(pos + 1);
        }
        std::string img_name;
        if (type == type_png) {
          img_name = output_path + channel_name + file_separator + camera_channel_name +
                     under_line + timestamp + extension_png;
          cv::Mat yuv(height * 3 / 2, width, CV_8UC1, data);
          cv::Mat png(height, width, CV_8UC3);
          cv::cvtColor(yuv, png, cv::COLOR_YUV2BGR_NV12);
          cv::imwrite(img_name, png);
        } else {
          img_name = output_path + channel_name + file_separator + camera_channel_name +
                     under_line + timestamp + extension_jpg;
          std::vector<int> compression_params;
          compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
          compression_params.push_back(100);
          std::vector<uint8_t> compressed_buffer(ptr_img2->data().begin(),
                                                 ptr_img2->data().end());
          cv::Mat image = cv::imdecode(compressed_buffer, CV_LOAD_IMAGE_UNCHANGED);
          if (!image.empty()) cv::imwrite(img_name, image, compression_params);
        }
      }
      frame++;
    }
    printf("Total frame is %d \n", frame);
  }
  return apollo::cyber::SUCC;
}
#else
void DataExtractor::extract_frame(const std::string pcaket_path, const std::string output_path,
                              const std::string type) {
  rosbag2_cpp::Reader reader_;
  reader_.open(pcaket_path);  // open file
  std::unordered_map<std::string, std::string> topic_and_type;
  auto topics = reader_.get_all_topics_and_types();
  for (const auto & topic_with_type : topics) {
      topic_and_type[topic_with_type.name] = topic_with_type.type;
  }
  uint64_t nanosec, sec, stamp_int;
  while (reader_.has_next()) {  // if not end
      auto bag_message = reader_.read_next();
      if (0 == topic_and_type[bag_message->topic_name].compare(sensor_msg_type_image_)) {
          rclcpp::SerializedMessage serialization_message(*bag_message->serialized_data);
          rclcpp::Serialization<sensor_msgs::msg::Image> serialization_camera_message;
          serialization_camera_message.deserialize_message(&serialization_message, &image_msg_);
          if (image_msg_.data.data() == NULL) {
            continue;
          }
          height = static_cast<int>(image_msg_.height);
          width = static_cast<int>(image_msg_.width);
          sec = image_msg_.header.stamp.sec;
          nanosec = image_msg_.header.stamp.nanosec;
          stamp_int = sec * 1000000 + nanosec/1000;
          timestamp = std::to_string(stamp_int);
          channel_name = bag_message->topic_name;
          auto pos = channel_name.rfind('/');
          if (pos == std::string::npos) {
            camera_channel_name = channel_name;
          } else {
            camera_channel_name = channel_name.substr(pos + 1);
          }
          extract_ros2_image(output_path, type);
      } else if (0 == topic_and_type[bag_message->topic_name].compare(type_custom_image_)) {
          rclcpp::SerializedMessage serialization_message(*bag_message->serialized_data);
          rclcpp::Serialization<sensor_msg::msg::Image> serialization_camera_message;
          serialization_camera_message.deserialize_message(&serialization_message,
                                                           &custom_image_msg_);
          if (custom_image_msg_.data.data() == NULL) {
            continue;
          }
          channel_name = bag_message->topic_name;
          extract_custom_image(output_path, type);
      } else if (0 == topic_and_type[bag_message->topic_name].compare(sensor_msg_type_ins_)) {
          rclcpp::SerializedMessage serialization_message(*bag_message->serialized_data);
          rclcpp::Serialization<sensor_msg::msg::Ins> serialization_ins_message;
          serialization_ins_message.deserialize_message(&serialization_message, &ins_msg_);
          std::shared_ptr<sensor_msg::msg::Ins> ptr_msg
          = std::make_shared<sensor_msg::msg::Ins>(ins_msg_);
          ins->parse_ins_data(ptr_msg, &can_signal_ins570d);
          ins->ins570d_struct2str(dr_info, &can_signal_ins570d);
          if (dr_info != NULL) {
            std::string folder_path = output_path + bag_message->topic_name;
            if (system(("mkdir -p " + folder_path).c_str())) {
              LOG(WARNING) << "[DataExtractor] Failed to create dir " << folder_path;
            }
            write_ins_file(ins_file_name, folder_path, dr_info);
          }
      }
  }
  reader_.close();
}

void DataExtractor::extract_ros2_image(const std::string output_path,
                                       const std::string type) {
          std::string folder_path = output_path + channel_name;
          if (system(("mkdir -p " + folder_path).c_str())) {
            LOG(WARNING) << "[DataExtractor] Failed to create dir " << folder_path;
          }
          if (0 == type.compare(type_jpg)) {
            image_name = folder_path + file_separator +
                                  camera_channel_name + under_line + timestamp +
                                   extension_jpg;
            std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(100);
            std::vector<unsigned char> compressed_buffer(image_msg_.data.begin(),
                                                  image_msg_.data.end());
            cv::Mat image = cv::imdecode(compressed_buffer, CV_LOAD_IMAGE_UNCHANGED);
            if (!image.empty()) cv::imwrite(image_name, image, compression_params);
          } else if (0 == type.compare(type_png)) {
            image_name = folder_path + file_separator +
                                  camera_channel_name + under_line + timestamp +
                                   extension_png;
            cv::Mat yuv_input(height*3/2, width, CV_8UC1,
            static_cast<void*>(const_cast<unsigned char*>(image_msg_.data.data())));
            cv::Mat res(height, width, CV_8UC3);
            cv::cvtColor(yuv_input, res, cv::COLOR_YUV2BGR_NV12);
            cv::imwrite(image_name, res);
          }
}

void DataExtractor::extract_custom_image(const std::string output_path,
                                         const std::string type) {
          std::string folder_path = output_path + channel_name;
          if (system(("mkdir -p " + folder_path).c_str())) {
            LOG(WARNING) << "[DataExtractor] Failed to create dir " << folder_path;
          }
          height = static_cast<int>(custom_image_msg_.height);
          width = static_cast<int>(custom_image_msg_.width);
          timestamp = std::to_string(custom_image_msg_.header.camera_timestamp);
          auto pos = channel_name.rfind('/');
          if (pos == std::string::npos) {
            camera_channel_name = channel_name;
          } else {
            camera_channel_name = channel_name.substr(pos + 1);
          }
          cv::Mat yuv_input(height*3/2, width, CV_8UC1,
          static_cast<void*>(const_cast<unsigned char*>(custom_image_msg_.data.data())));
          cv::Mat res(height, width, CV_8UC3);
          cv::cvtColor(yuv_input, res, cv::COLOR_YUV2BGR_NV12);
          if (0 == type.compare(type_jpg)) {
            image_name = folder_path + file_separator +
                                  camera_channel_name + under_line + timestamp +
                                   extension_jpg;
            cv::imwrite(image_name, res);
          } else if (0 == type.compare(type_png)) {
            image_name = folder_path + file_separator +
                                  camera_channel_name + under_line + timestamp +
                                   extension_png;
            cv::imwrite(image_name, res);
          }
}
#endif

}  // namespace data_extractor
}  // namespace airi
}  // namespace crdc
