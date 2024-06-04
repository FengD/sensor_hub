#include "camera_drivers/input/ros2/ros2_input.h"

namespace crdc {
namespace airi {

bool Ros2Input::camera_init() {
  if (!config_.has_ros2_config()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] " << "has no ros2 input config";
    return false;
  }

  if (config_.ros2_config().file_path().size() == 0) {
    LOG(ERROR) << "No ros2 record file given!";
    return false;
  }

  LOG(INFO) << "ros2 init success";
  return true;
}

bool Ros2Input::camera_start() {
  LOG(INFO) << "[" << config_.frame_id() << "] ros2 input start.";
  auto data_thread = new std::thread(&Ros2Input::get_data, this);
  return true;
}

bool Ros2Input::camera_stop() {
  LOG(INFO) << "[" << config_.frame_id() << "] camera input stopped.";
  return true;
}

void Ros2Input::bgr_to_nv12(cv::Mat& bgr_image, cv::Mat& nv12_image) {
  cv::Mat img_yuv_yv12;
  int height = bgr_image.rows;
  int width = bgr_image.cols;
  cv::cvtColor(bgr_image, img_yuv_yv12, cv::COLOR_BGR2YUV_YV12);
  memcpy(nv12_image.data, img_yuv_yv12.data, height * width);
  char *v = reinterpret_cast<char*>(img_yuv_yv12.data) + height * width;
  char *u = v + height * width / 4;
  char *dst = reinterpret_cast<char*>(nv12_image.data) + height * width;
  for (int i = 0; i < height * width / 4; ++i) {
    dst[2 * i] = u[i];
    dst[2 * i + 1] = v[i];
  }
}

bool Ros2Input::get_topic_name(std::string* topic_name,
                               std::shared_ptr<const CameraRawData>& raw_data) {
  if (raw_data->data_type != "jpg") {
    return false;
  }

  if (ros2_topic_queue_.wait_for_dequeue(topic_name, 100000)) {
    return true;
  } else {
    return false;
  }
}

void Ros2Input::parse_rosbag() {
  while (reader_.has_next()) {
    auto start = get_now_microsecond();
    auto bag_message = reader_.read_next();
    std::string topic_name = bag_message->topic_name;
    if (topic_and_type_[topic_name] == "sensor_msgs/msg/CompressedImage") {
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      rclcpp::Serialization<sensor_msgs::msg::CompressedImage>
                                    serialization_compressed_image;
      serialization_compressed_image.deserialize_message(
                        &extracted_serialized_msg, &sensor_msgs_compressed_image_);
      uint64_t timestamp =
                static_cast<uint64_t>(sensor_msgs_compressed_image_.header.stamp.sec) * 1000000 +
                static_cast<uint64_t>(sensor_msgs_compressed_image_.header.stamp.nanosec) / 1000;
      if (0 != topic_name.compare(config_.ros2_config().channel_encode_name())) {
        // topic name enqueue, used for distinguish mask
        ros2_topic_queue_.enqueue(topic_name);
        // save mask image, jpeg format
        std::shared_ptr<CameraRawData> raw_data = get_raw_data
        (0, timestamp, (unsigned char *)(sensor_msgs_compressed_image_.data.data()));
        raw_data->data_type = "jpg";
        raw_data->data_size = sensor_msgs_compressed_image_.data.size();
        raw_data_queue_.enqueue(raw_data);
        matching_fps_by_sleep(start, get_now_microsecond());
      } else {
        std::vector<uint8_t> compressed_buffer
        (sensor_msgs_compressed_image_.data.begin(), sensor_msgs_compressed_image_.data.end());
        cv::Mat image = cv::imdecode(compressed_buffer, cv::IMREAD_UNCHANGED);
        cv::Mat nv12_image(image.rows * 3 / 2, image.cols, CV_8UC1);
        bgr_to_nv12(image, nv12_image);
        // save raw image, nv12 format
        sensor_msgs_image_.data.resize(nv12_image.rows * nv12_image.cols);
        std::memcpy(sensor_msgs_image_.data.data(),
                              nv12_image.data, nv12_image.rows * nv12_image.cols);
        std::shared_ptr<CameraRawData> raw_data = get_raw_data
        (0, timestamp, (unsigned char *)(sensor_msgs_image_.data.data()));
        raw_data->data_type = "NV12";
        raw_data->data_size = nv12_image.rows * nv12_image.cols;
        raw_data_queue_.enqueue(raw_data);
        matching_fps_by_sleep(start, get_now_microsecond());
      }
    } else {
      matching_fps_by_sleep(start, get_now_microsecond());
      continue;
    }
  }
}

void Ros2Input::get_data() {
  for (int i = 0; i < config_.ros2_config().file_path().size(); ++i) {
    reader_.open(config_.ros2_config().file_path(i));
    LOG(INFO) << "file path: " << config_.ros2_config().file_path(i);
    auto topics = reader_.get_all_topics_and_types();
    if (!topics.empty()) {
      for (const auto& topic_with_type : topics) {
        topic_and_type_[topic_with_type.name] = topic_with_type.type;
      }
      // compressed image
      CHECK_EQ(topic_and_type_[config_.ros2_config().channel_encode_name()],
                                       "sensor_msgs/msg/CompressedImage");
      // ld mask
      CHECK_EQ(topic_and_type_[config_.ros2_config().channel_name(0)],
                                       "sensor_msgs/msg/CompressedImage");
      // fs mask
      CHECK_EQ(topic_and_type_[config_.ros2_config().channel_name(1)],
                                       "sensor_msgs/msg/CompressedImage");
    } else {
      LOG(ERROR) << "[" << config_.frame_id() << "] "
           << "There are no messages in " << config_.ros2_config().file_path(i);
      break;
    }
    parse_rosbag();
    topic_and_type_.clear();
  }
  LOG(FATAL) << "Read ros2 Files Finished!";
}

}  // namespace airi
}  // namespace crdc
