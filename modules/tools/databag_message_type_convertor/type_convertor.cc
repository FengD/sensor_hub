#include "tools/databag_message_type_convertor/type_convertor.h"
DEFINE_string(ins_config_path,
              "params/drivers/ins/databag_message_type_convertor/ins_config.prototxt",
              "path of config file");
DEFINE_string(lidar_config_path,
              "params/drivers/lidar/databag_message_type_convertor/lidar_config.prototxt",
              "path of config file");
DEFINE_string(camera_config_path,
              "params/drivers/camera/databag_message_type_convertor/camera_config.prototxt",
              "path of config file");

namespace crdc {
namespace airi {
#ifndef WITH_ROS2
void TypeConvertor::extract_cloud(const crdc::airi::Packets& packets,
                   apollo::cyber::record::RecordWriter& writer,
                   const std::shared_ptr<crdc::airi::LidarParser>& lidar_parser,
                   const std::string& channel_name, const std::string& frame_id) {
  std::shared_ptr<crdc::airi::LidarPointCloud> cloud;
  for (int i = 0; i < packets.packet_size(); ++i) {
    auto raw_packet = packets.packet(i);
    if (lidar_parser->parse_lidar_packet(&raw_packet, &cloud)) {
      uint64_t msgs_time = raw_packet.time_system() * 1000;
      cloud->proto_cloud_->mutable_header()->set_frame_id(frame_id);
      cloud->proto_cloud_->mutable_header()->set_module_name("LidarExtract");
      cloud->proto_cloud_->mutable_header()->set_lidar_timestamp(raw_packet.time_system());
      cloud->proto_cloud_->mutable_header()->set_timestamp_sec(
          static_cast<double>(msgs_time) / 1e9);
      std::string transform = std::string(std::getenv("TF"));
      std::shared_ptr<crdc::airi::PointCloud2> temp_cloud;
      if (0 == transform.compare("yes")) {
        temp_cloud = compensator_.cloud_transform(cloud);
      } else {
        temp_cloud = std::make_shared<crdc::airi::PointCloud2>(*cloud->proto_cloud_);
      }
      writer.WriteMessage(channel_name, *temp_cloud, msgs_time);
    }
  }
}

void TypeConvertor::extract_ins(const crdc::airi::Packets& packets,
                 apollo::cyber::record::RecordWriter& writer,
                 const std::shared_ptr<crdc::airi::InsParser>& ins_parser,
                 const std::string& channel_name, const std::string& frame_id) {
  std::shared_ptr<crdc::airi::InsData> ins_data;
  for (int i = 0; i < packets.packet_size(); ++i) {
    auto raw_packet = packets.packet(i);
    if (ins_parser->parse_ins_packet(&raw_packet, &ins_data)) {
      uint64_t msgs_time = raw_packet.time_system() * 1000;
      ins_data->proto_ins_data_->mutable_header()->set_frame_id(frame_id);
      ins_data->proto_ins_data_->mutable_header()->set_module_name("InsExtract");
      ins_data->proto_ins_data_->mutable_header()->set_timestamp_sec(
          static_cast<double>(msgs_time) / 1e9);
      auto temp_ins = std::make_shared<crdc::airi::Ins>(*ins_data->proto_ins_data_);
      writer.WriteMessage(channel_name, *temp_ins, msgs_time);
    }
  }
}
#endif
std::shared_ptr<crdc::airi::LidarParser> TypeConvertor::init_lidar_parser(
    const std::string& lidar_config_path, std::string frame_id) {
  std::shared_ptr<crdc::airi::LidarParser> lidar_parser = nullptr;
  crdc::airi::LidarConfig lidar_config;
  crdc::airi::util::get_proto_from_file(lidar_config_path, &lidar_config);
  if (lidar_config.has_parser_config()) {
    lidar_config.mutable_parser_config()->set_frame_id(frame_id);
    lidar_parser = crdc::airi::LidarParserFactory::get(
        lidar_config.parser_config().name());
    if (!lidar_parser) {
      LOG(FATAL) << "Failed to get lidar parser ptr: " << lidar_config.parser_config().name();
    }
    if (!lidar_parser->init(lidar_config.parser_config())) {
      LOG(FATAL) << "Failed to init lidar parser: " << lidar_config.parser_config().name();
    }
  }
  return lidar_parser;
}

std::shared_ptr<crdc::airi::InsParser> TypeConvertor::init_ins_parser(
    const std::string& ins_config_path, std::string frame_id) {
  std::shared_ptr<crdc::airi::InsParser> ins_parser = nullptr;
  crdc::airi::InsConfig ins_config;
  crdc::airi::util::get_proto_from_file(ins_config_path, &ins_config);
  if (ins_config.has_parser_config()) {
    ins_config.mutable_parser_config()->set_frame_id(frame_id);
    ins_parser = InsParserFactory::get(ins_config.parser_config().name());
    if (!ins_parser) {
      LOG(FATAL) << "Failed to get ins parser ptr: " << ins_config.parser_config().name();
    }

    if (!ins_parser->init(ins_config.parser_config())) {
      LOG(FATAL) << "Failed to init ins parser: " << ins_config.parser_config().name();
    }
  }
  return ins_parser;
}

#ifndef WITH_ROS2
void TypeConvertor::extract_packet(const std::string& input_pcaket_path,
                                 const std::string& save_pcaket_path) {
  std::unordered_map<std::string, std::shared_ptr<crdc::airi::InsParser>> ins_parsers;
  std::unordered_map<std::string, std::string> ins_channel_names;
  std::unordered_map<std::string, std::string> ins_frame_id;
  std::string ins_config = std::string(std::getenv("CRDC_WS")) + "/../" + FLAGS_ins_config_path;
  InsComponent ins_component;
  crdc::airi::util::get_proto_from_file(ins_config, &ins_component);
  for (const auto& ins_component_config : ins_component.component_config()) {
    std::string config_file_path = std::string(std::getenv("CRDC_WS"))
                                + "/../" + ins_component_config.config_file();
    std::string frame_id = ins_component_config.frame_id();
    auto ins_parser = init_ins_parser(config_file_path, frame_id);
    ins_parsers[ins_component_config.raw_data_channel_name()] = ins_parser;
    ins_channel_names[ins_component_config.raw_data_channel_name()] =
        ins_component_config.channel_name();
    ins_frame_id[ins_component_config.raw_data_channel_name()] = frame_id;
  }

  std::unordered_map<std::string, std::shared_ptr<crdc::airi::LidarParser>> lidar_parsers;
  std::unordered_map<std::string, std::string> lidar_channel_names;
  std::unordered_map<std::string, std::string> lidar_frame_id;
  std::string lidar_config = std::string(std::getenv("CRDC_WS")) + "/../" + FLAGS_lidar_config_path;
  LidarComponent lidar_component;
  crdc::airi::util::get_proto_from_file(lidar_config, &lidar_component);
  compensator_.init(lidar_component);
  for (const auto& lidar_component_config : lidar_component.component_config()) {
    std::string config_file_path = std::string(std::getenv("CRDC_WS"))
                                + "/../" + lidar_component_config.config_file();
    std::string frame_id = lidar_component_config.frame_id();
    auto lidar_parser = init_lidar_parser(config_file_path, frame_id);
    lidar_parsers[lidar_component_config.raw_data_channel_name()] = lidar_parser;
    lidar_channel_names[lidar_component_config.raw_data_channel_name()] =
        lidar_component_config.channel_name();
    lidar_frame_id[lidar_component_config.raw_data_channel_name()] = frame_id;
  }

  apollo::cyber::record::RecordReader reader(input_pcaket_path);
  apollo::cyber::record::RecordMessage message;
  apollo::cyber::record::RecordWriter writer;
  writer.SetSizeOfFileSegmentation(2000000);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(save_pcaket_path);
  while (reader.ReadMessage(&message)) {
    std::string channel_name = message.channel_name;
    std::string message_type = reader.GetMessageType(channel_name);
    if (0 == message_type.compare("crdc.airi.Image2")) {
      auto msg = std::make_shared<crdc::airi::Image2>();
      msg->ParseFromString(message.content);
      writer.WriteMessage(channel_name, *msg, message.time);
    } else if (0 == message_type.compare("crdc.airi.Packets")) {
      crdc::airi::Packets msg;
      msg.ParseFromString(message.content);
      if (lidar_parsers.find(channel_name) != lidar_parsers.end()) {
        extract_cloud(msg, writer, lidar_parsers[channel_name], lidar_channel_names[channel_name],
                    lidar_frame_id[channel_name]);
      } else if (ins_parsers.find(channel_name) != ins_parsers.end()) {
        extract_ins(msg, writer, ins_parsers[channel_name], ins_channel_names[channel_name],
                    ins_frame_id[channel_name]);
      }
    } else if (0 == message_type.compare("crdc.airi.PerceptionObstacles")) {
       crdc::airi::PerceptionObstacles msg;
       msg.ParseFromString(message.content);
       writer.WriteMessage(channel_name, msg, message.time);
    }
  }

  std::cout << save_pcaket_path << " write over!" << std::endl;
  writer.Close();
}
#else
void TypeConvertor::extract_ros2cloud(const Packet* raw_packet,
                   rosbag2_cpp::Writer& writer, rcutils_time_point_value_t time_stamp,
                   const std::shared_ptr<crdc::airi::LidarParser>& lidar_parser,
                   const std::string& channel_name, const std::string& frame_id) {
  std::shared_ptr<crdc::airi::LidarPointCloud> cloud;
  if (lidar_parser->parse_lidar_packet(raw_packet, &cloud)) {
    uint64_t msgs_time = raw_packet->time_system * 1000;
    cloud->cloud_msg_->header.frame_id = frame_id;
    cloud->cloud_msg_->header.stamp.sec = msgs_time / 1000000000;
    cloud->cloud_msg_->header.stamp.nanosec = msgs_time % 1000000000;

    sensor_msgs::msg::PointCloud2 temp_cloud;
    if (!std::getenv("TF")) {
      LOG(FATAL) << "TF not setting! please check the script or env!!";
    }
    std::string transform = std::string(std::getenv("TF"));
    if (0 == ispacket_.compare("yes") && 0 == transform.compare("yes")) {
      temp_cloud = *compensator_.cloud_transform(cloud);
    } else {
      temp_cloud = *cloud->cloud_msg_;
    }
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
                    std::make_shared<rclcpp::SerializedMessage>();
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    serialization.serialize_message(&temp_cloud, serialized_msg.get());

    auto write_bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    rosbag2_storage::TopicMetadata tm;
    tm.name = channel_name;
    tm.type = "sensor_msgs/msg/PointCloud2";
    tm.serialization_format = "cdr";
    writer.create_topic(tm);
    write_bag_message->time_stamp = time_stamp;
    write_bag_message->topic_name = tm.name;
    write_bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                              &serialized_msg->get_rcl_serialized_message(),
                              [](rcutils_uint8_array_t * /* data */) {});
    *write_bag_message->serialized_data = serialized_msg->release_rcl_serialized_message();
    writer.write(write_bag_message);
  }
}

void TypeConvertor::extract_ros2ins(const sensor_msg::msg::Packets& packets_msg_,
                   rosbag2_cpp::Writer& writer, rcutils_time_point_value_t time_stamp,
                   const std::shared_ptr<crdc::airi::InsParser>& ins_parser,
                   const std::string& channel_name, const std::string& frame_id) {
  std::shared_ptr<crdc::airi::InsData> ins_data;
  for (size_t i = 0; i < packets_msg_.packet.size(); ++i) {
    auto raw_packet = packets_msg_.packet[i];
    if (ins_parser->parse_ins_packet(&raw_packet, &ins_data)) {
      uint64_t msgs_time = raw_packet.time_system * 1000;
      ins_data->proto_ins_data_->header.frame_id = frame_id;
      ins_data->proto_ins_data_->header.timestamp_sec = static_cast<double>(msgs_time) / 1e9;

      sensor_msg::msg::Ins temp_Ins;
      temp_Ins = *ins_data->proto_ins_data_;

      std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
                      std::make_shared<rclcpp::SerializedMessage>();
      rclcpp::Serialization<sensor_msg::msg::Ins> serialization;
      serialization.serialize_message(&temp_Ins, serialized_msg.get());

      auto write_bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      rosbag2_storage::TopicMetadata tm;
      tm.name = channel_name;
      tm.type = "sensor_msg/msg/Ins";
      tm.serialization_format = "cdr";
      writer.create_topic(tm);
      write_bag_message->time_stamp = time_stamp;
      write_bag_message->topic_name = tm.name;
      write_bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                                &serialized_msg->get_rcl_serialized_message(),
                                [](rcutils_uint8_array_t * /* data */) {});
      *write_bag_message->serialized_data = serialized_msg->release_rcl_serialized_message();
      writer.write(write_bag_message);
    }
  }
}

template <typename T>
void TypeConvertor::deserialized_msg_to_bag(rosbag2_cpp::Writer& writer, std::string& channel_type,
                                            std::string& channel_name, T& message,
                                            rcutils_time_point_value_t& time_stamp) {
  auto bag_time = rclcpp::Time(time_stamp);
  rclcpp::Serialization<T> serialization;
  std::shared_ptr<rclcpp::SerializedMessage> serialized =
  std::make_shared<rclcpp::SerializedMessage>();
  serialization.serialize_message(&message, serialized.get());
  writer.write(serialized, channel_name, channel_type, bag_time);
}

void TypeConvertor::create_ros2writer(rosbag2_cpp::Writer& writer,
                                      rosbag2_storage::StorageOptions& storage_options,
                                     const std::string& save_pcaket_path) {
  int nPos = save_pcaket_path.find_last_of('.');
  storage_options.storage_id = "sqlite3";
  storage_options.uri = save_pcaket_path.substr(0, nPos);
  rcpputils::fs::path rosbag_directory(storage_options.uri);
  rcpputils::fs::remove_all(rosbag_directory);  // remove exists directory, overwriter
  if (0 != record_type_.compare("default")) {
    writer.open(storage_options);
  }
}

void TypeConvertor::extract_ros2packet(const std::string& input_pcaket_path,
                                 const std::string& save_pcaket_path) {
  rosbag2_cpp::Reader reader;
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  create_ros2writer(writer, storage_options, save_pcaket_path);
  reader.open(input_pcaket_path);
  image_decode_ = std::make_shared<crdc::airi::ImageDecode>();
  image_decode_->ffmpeg_init();
  image_decode_->DecodeInit();

  std::unordered_map<std::string, std::shared_ptr<crdc::airi::InsParser>> ins_parsers;
  std::unordered_map<std::string, std::string> ins_channel_names;
  std::unordered_map<std::string, std::string> ins_frame_id;
  std::unordered_map<std::string, std::shared_ptr<crdc::airi::LidarParser>> lidar_parsers;
  std::unordered_map<std::string, std::string> lidar_channel_names;
  std::unordered_map<std::string, std::string> lidar_frame_id;
  if (0 == ispacket_.compare("yes")) {
    std::string ins_config = std::string(std::getenv("CRDC_WS")) + "/../" + FLAGS_ins_config_path;
    InsComponent ins_component;
    crdc::airi::util::get_proto_from_file(ins_config, &ins_component);
    for (const auto& ins_component_config : ins_component.component_config()) {
      std::string config_file_path = std::string(std::getenv("CRDC_WS"))
                                  + "/../" + ins_component_config.config_file();
      std::string frame_id = ins_component_config.frame_id();
      auto ins_parser = init_ins_parser(config_file_path, frame_id);
      ins_parsers['/' + ins_component_config.raw_data_channel_name()] = ins_parser;
      ins_channel_names['/' + ins_component_config.raw_data_channel_name()] =
          ins_component_config.channel_name();
      ins_frame_id['/' + ins_component_config.raw_data_channel_name()] = frame_id;
    }

    std::string lidar_config_path = std::string(std::getenv("CRDC_WS")) + "/../"
                                                    + FLAGS_lidar_config_path;
    LidarComponent lidar_component;
    crdc::airi::util::get_proto_from_file(lidar_config_path, &lidar_component);

    for (const auto& lidar_component_config : lidar_component.component_config()) {
      compensator_.init(lidar_component_config);
      std::string config_file_path = std::string(std::getenv("CRDC_WS"))
                                  + "/../" + lidar_component_config.config_file();
      std::string frame_id = lidar_component_config.frame_id();
      auto lidar_parser = init_lidar_parser(config_file_path, frame_id);
      lidar_parsers['/' + lidar_component_config.raw_data_channel_name()] = lidar_parser;
      lidar_channel_names['/' + lidar_component_config.raw_data_channel_name()] =
          lidar_component_config.channel_name();
      lidar_frame_id['/' + lidar_component_config.raw_data_channel_name()] = frame_id;
    }
  }

  std::unordered_map<std::string, std::string> topic_and_type;
  auto topics = reader.get_all_topics_and_types();
  for (const auto & topic_with_type : topics) {
    topic_and_type[topic_with_type.name] = topic_with_type.type;
    std::cout << "name: " << topic_with_type.name << " " << topic_with_type.type << std::endl;
  }
  deflag_ = false;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    path_ = storage_options.uri + bag_message->topic_name;
    if (0 == topic_and_type[bag_message->topic_name].compare("sensor_msg/msg/Packets") &&
                                                        0 == ispacket_.compare("yes")) {
      rclcpp::Serialization<sensor_msg::msg::Packets> serialization_packets;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_packets.deserialize_message(&extracted_serialized_msg, &packets_msg_);
      if (lidar_parsers.find(bag_message->topic_name) != lidar_parsers.end()) {
        for (size_t i = 0; i < packets_msg_.packet.size(); ++i) {
          auto raw_packet = packets_msg_.packet[i];
          extract_ros2cloud(&raw_packet, writer, bag_message->time_stamp,
                          lidar_parsers[bag_message->topic_name],
                          lidar_channel_names[bag_message->topic_name],
                          lidar_frame_id[bag_message->topic_name]);
        }
        deflag_ = true;
      } else if (ins_parsers.find(bag_message->topic_name) != ins_parsers.end()) {
        extract_ros2ins(packets_msg_, writer, bag_message->time_stamp,
                          ins_parsers[bag_message->topic_name],
                          ins_channel_names[bag_message->topic_name],
                          ins_frame_id[bag_message->topic_name]);
        deflag_ = true;
      }
    } else if (0 == topic_and_type[bag_message->topic_name].compare("sensor_msg/msg/Image")) {
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      image_decode_->decode(extracted_serialized_msg, bag_message->topic_name,
                            image_msg_, path_, deflag_, encodeframe_);
    } else if (0 == topic_and_type[bag_message->topic_name].
                                                  compare("sensor_msgs/msg/CompressedImage")) {
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      image_decode_->decode(extracted_serialized_msg, bag_message->topic_name,
                            image_msgs_, path_, deflag_, pressed_msg_);
    } else if (0 == topic_and_type[bag_message->topic_name].
                                                  compare("sensor_msg/msg/PerceptionObstacles")) {
      rclcpp::Serialization<sensor_msg::msg::PerceptionObstacles> serialization_fs_perception;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_fs_perception.deserialize_message(&extracted_serialized_msg,
                                                      &fs_perception_out_);
    } else if (0 == topic_and_type[bag_message->topic_name].compare("sensor_msg/msg/MarkerList")) {
      rclcpp::Serialization<sensor_msg::msg::MarkerList> serialization_ld_perception;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_ld_perception.deserialize_message(&extracted_serialized_msg,
                                                      &ld_perception_out_);
    } else if (0 == topic_and_type[bag_message->topic_name].
                                                    compare("geometry_msgs/msg/PolygonStamped")) {
      rclcpp::Serialization<geometry_msgs::msg::PolygonStamped> serialization_fs_perception;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_fs_perception.deserialize_message(&extracted_serialized_msg,
                                                      &fs_polygon_msg_);
    } else if (0 == topic_and_type[bag_message->topic_name].
                                                    compare("visualization_msgs/msg/Marker")) {
      rclcpp::Serialization<visualization_msgs::msg::Marker> serialization_ld_perception;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_ld_perception.deserialize_message(&extracted_serialized_msg,
                                                      &ld_marker_msgs_);
    }

    if (0 == record_type_.compare("play")) {  // cv image
      if (0 == topic_and_type[bag_message->topic_name].
                                                compare("sensor_msg/msg/PerceptionObstacles")) {
        common::Singleton<TypeConverAFOutput>::get()->write_obstacles(
                                bag_message->topic_name, fs_perception_out_);
        deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name],
                            bag_message->topic_name, fs_perception_out_, bag_message->time_stamp);
      } else if (0 == topic_and_type[bag_message->topic_name].
                                                compare("sensor_msg/msg/MarkerList")) {
        common::Singleton<TypeConverAFOutput>::get()->write_markerlists(
                                bag_message->topic_name, ld_perception_out_);
        deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name],
                            bag_message->topic_name, ld_perception_out_, bag_message->time_stamp);
      } else if (0 == topic_and_type[bag_message->topic_name].
                                                compare("geometry_msgs/msg/PolygonStamped")) {
        common::Singleton<TypeConverAFOutput>::get()->write_polygon_stamped(
                                bag_message->topic_name, fs_polygon_msg_);
        deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name],
                            bag_message->topic_name, fs_polygon_msg_, bag_message->time_stamp);
      } else if (0 == topic_and_type[bag_message->topic_name].
                                                    compare("visualization_msgs/msg/Marker")) {
        common::Singleton<TypeConverAFOutput>::get()->write_marker(
                                bag_message->topic_name, ld_marker_msgs_);
        deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name],
                            bag_message->topic_name, ld_marker_msgs_, bag_message->time_stamp);
      } else if (deflag_ &&
                0 == topic_and_type[bag_message->topic_name].compare("sensor_msg/msg/Image")) {
        if (encodeframe_.compression != sensor_msg::msg::Image::H264) {
          std::string ld = "/hav_ld_mask";
          std::string fs = "/hav_fs_mask";
          image_decode_->get_img_ld(process_img_ld_);
          image_decode_->get_img_fs(process_img_fs_);
          image_decode_->imagepublish(fs_process_image_msg_, process_img_fs_, fs, encodeframe_);
          image_decode_->imagepublish(ld_process_image_msg_, process_img_ld_, ld, encodeframe_);
          deflag_ = false;
          play_save_flag_ = true;
          deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name], fs,
                                  fs_process_image_msg_, bag_message->time_stamp);
          deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name], ld,
                                  ld_process_image_msg_, bag_message->time_stamp);
        } else {
          std::cout << "H264 Image advertising is not supported!" << std::endl;
        }
      } else if (deflag_ && 0 == topic_and_type[bag_message->topic_name].
                                                    compare("sensor_msgs/msg/CompressedImage")) {
        if (pressed_msg_.format != "h264") {
          std::string ld = "/hav_ld_mask";
          std::string fs = "/hav_fs_mask";
          image_decode_->get_img_ld(process_img_ld_);
          image_decode_->get_img_fs(process_img_fs_);
          if (!process_img_ld_.empty()) {
            image_decode_->imagepublish(ld_process_image_msgs_, process_img_ld_, ld, pressed_msg_);
            process_img_ld_.release();
            deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name], ld,
                                  ld_process_image_msgs_, bag_message->time_stamp);
          }
          if (!process_img_fs_.empty()) {
            image_decode_->imagepublish(fs_process_image_msgs_, process_img_fs_, fs, pressed_msg_);
            process_img_fs_.release();
            deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name], fs,
                                  fs_process_image_msgs_, bag_message->time_stamp);
          }
          deflag_ = false;
          play_save_flag_ = true;
        } else {
          std::cout << "H264 Image advertising is not supported!" << std::endl;
        }
      }
    } else if (0 == record_type_.compare("record")) {  // h264 image & point cloud
      std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
                                  std::make_shared<rclcpp::SerializedMessage>();
      std::shared_ptr<rcutils_uint8_array_t> serialized_data = bag_message->serialized_data;
      std::string topic_name = bag_message->topic_name + "_";
      rosbag2_storage::TopicMetadata tm;
      tm.type = topic_and_type[bag_message->topic_name];
      if (0 == topic_and_type[bag_message->topic_name].
                                                  compare("sensor_msgs/msg/CompressedImage")) {
        if (topic_name.compare("/hav_ld_mask_") != 0 && topic_name.compare("/hav_fs_mask_") != 0) {
          if (!std::getenv("TYPE")) {
            LOG(FATAL) << "TYPE not setting! please check the script or env!!";
          }
          std::string extract_type = std::getenv("TYPE");
          if (0 != extract_type.compare("skip")) {
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
            serialization.serialize_message(&image_msgs_, serialized_msg.get());
            serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                                      &serialized_msg->get_rcl_serialized_message(),
                                      [](rcutils_uint8_array_t * /* data */) {});
            topic_name = bag_message->topic_name + "_DECODE";
            tm.type = "sensor_msgs/msg/Image";
          } else {
            serialized_data = bag_message->serialized_data;
            topic_name = bag_message->topic_name;
          }
        } else {
          continue;
        }
      } else if (0 == topic_and_type[bag_message->topic_name].compare("sensor_msg/msg/Image")) {
        if (encodeframe_.compression == sensor_msg::msg::Image::H264) {
          rclcpp::Serialization<sensor_msg::msg::Image> serialization;
          serialization.serialize_message(&image_msg_, serialized_msg.get());
          serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                                    &serialized_msg->get_rcl_serialized_message(),
                                    [](rcutils_uint8_array_t * /* data */) {});
          topic_name = bag_message->topic_name + "_DECODE";
        } else {
          LOG(FATAL) << "This bag does not need to be decoded and saved ";
        }
      } else if (0 == topic_and_type[bag_message->topic_name].compare("sensor_msg/msg/Packets")) {
        continue;
      }
      auto write_bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      tm.name = topic_name;
      tm.serialization_format = "cdr";
      writer.create_topic(tm);
      write_bag_message->time_stamp = bag_message->time_stamp;
      write_bag_message->topic_name = tm.name;
      write_bag_message->serialized_data = serialized_data;
      writer.write(write_bag_message);
    }
  }
  reader.close();
  image_decode_->clear_image_map();
  if (deflag_ || play_save_flag_) {
    std::cout << save_pcaket_path << " write over!" << std::endl;
  } else {
    std::cout << "no data is saved, please check the topic type!!" << std::endl;
  }
}

// pcap -> ros2 msg
void TypeConvertor::extract_pcappacket(const std::string& input_pcaket_path,
                                 const std::string& save_pcaket_path) {
  pcap_t *pcap;
  char errbuf[PCAP_ERRBUF_SIZE];
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  create_ros2writer(writer, storage_options, save_pcaket_path);
  // open an offline pcap file
  if ((pcap = pcap_open_offline(input_pcaket_path.c_str(), errbuf)) == NULL) {
      std::cout << "Error opening pcap file. Please check!" << std::endl;
      return;
  }
  // need to set ip and udp 
  if (!std::getenv("SOURCE_IP") || !std::getenv("UDP_PORT")) {
      LOG(FATAL) << "SOURCE_IP or UDP_PORT not setting! please check the script or env!!";
  }
  // pcap filter 
  std::string source_ip_str = std::string(std::getenv("SOURCE_IP"));
  std::string udp_port = std::string(std::getenv("UDP_PORT"));
  std::stringstream filter;
  bpf_program pcap_packet_filter;
  filter << "src host " << source_ip_str << " && ";
  filter << "udp dst port " << udp_port;
  pcap_compile(pcap, &pcap_packet_filter,
           filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);

  // init config file
  std::unordered_map<uint16_t, std::shared_ptr<crdc::airi::LidarParser>> lidar_parsers;
  std::unordered_map<uint16_t, std::string> lidar_channel_names;
  std::unordered_map<uint16_t, std::string> lidar_frame_id;
  std::unordered_map<uint16_t, int32_t> lidar_packet_size;
  LidarConfig lidar_config;
  LidarComponent lidar_component;
  if (0 == ispacket_.compare("yes")) {
    std::string lidar_config_path = std::string(std::getenv("CRDC_WS")) + "/../"
                                                    + FLAGS_lidar_config_path;
    crdc::airi::util::get_proto_from_file(lidar_config_path, &lidar_component);
    for (const auto& lidar_component_config : lidar_component.component_config()) {
      compensator_.init(lidar_component_config);
      std::string config_file_path = std::string(std::getenv("CRDC_WS"))
                                  + "/../" + lidar_component_config.config_file();
      crdc::airi::util::get_proto_from_file(config_file_path, &lidar_config); 
      std::string frame_id = lidar_component_config.frame_id();
      auto lidar_parser = init_lidar_parser(config_file_path, frame_id);
      lidar_parsers[lidar_config.input_config().lidar_port()] = lidar_parser;
      lidar_channel_names[lidar_config.input_config().lidar_port()] =
          lidar_component_config.channel_name();
      lidar_frame_id[lidar_config.input_config().lidar_port()] = frame_id;
      lidar_packet_size[lidar_config.input_config().lidar_port()] = lidar_config.input_config().packet_size();
    }
  }

  while (1) {
    struct pcap_pkthdr *header;
    const u_char *pkt_data;
    if (pcap_next_ex(pcap, &header, &pkt_data) >= 0) {
      if (0 == pcap_offline_filter(&pcap_packet_filter, header, pkt_data)) {
        continue;
      }
      // get udp port, big-Endian -> little-Endian
      uint16_t network_order_udp;
      memcpy(&network_order_udp, pkt_data + 36, sizeof(uint16_t));
      uint16_t host_order_udp = (network_order_udp >> 8) | (network_order_udp << 8);

      // init packet
      Packet* packet = new Packet();
      packet->data.reserve(lidar_packet_size[host_order_udp]);
      packet->version = 0x0001;
      uint16_t receive_len = header->caplen - 42;
      uint64_t time_stamp = header->ts.tv_sec * 1000000 + header->ts.tv_usec;
      Packet *raw_packet = packet;
      raw_packet->data.resize(lidar_packet_size[host_order_udp]);
      char* data_buf = reinterpret_cast<char*>(raw_packet->data.data());
      memcpy(&data_buf[0], pkt_data + 42, receive_len);
      raw_packet->data.resize(receive_len);
      raw_packet->size = receive_len;
      raw_packet->time_system = time_stamp;
      raw_packet->port = host_order_udp;
      if (lidar_parsers.find(host_order_udp) != lidar_parsers.end()) {
        // auto now = rclcpp::Clock().now();
        extract_ros2cloud(raw_packet, writer, (rcutils_time_point_value_t)(time_stamp * 1000),
                          lidar_parsers[lidar_config.input_config().lidar_port()],
                          lidar_channel_names[lidar_config.input_config().lidar_port()],
                          lidar_frame_id[lidar_config.input_config().lidar_port()]);
      }
      delete packet;
    } else {
      std::cout << "pcap file is read completed" << std::endl;
      break;
    }
    deflag_ = true;
  }

  if (deflag_) {
    std::cout << save_pcaket_path << " write over!" << std::endl;
  } else {
    std::cout << "no data is saved, please check the topic type!!" << std::endl;
  }
}
#endif

void TypeConvertor::creat_folder(const std::string &folder_path) {
  std::string command;
  command = "mkdir -p " + folder_path;
  if (system(command.c_str())) {
    LOG(WARNING) << "[TypeConvertor] Failed to create dir " << folder_path;
  }
}

void TypeConvertor::write_packet(const std::string& path) {
  boost::filesystem::path boost_path(path);
  std::string parent_path = boost_path.parent_path().string();
  std::string filename = boost_path.filename().string();
  std::cout << "parent_path: " << parent_path << " , input_file: " << filename << std::endl;
  std::string folder_path = std::string(std::getenv("SAVE_PATH")) + "/";

  CameraComponent camera_component;
  std::string config = std::string(std::getenv("CRDC_WS")) + "/../" + FLAGS_camera_config_path;
  crdc::airi::util::get_proto_from_file(config, &camera_component);
  auto &camera_component_config = camera_component.component_config(0);
  std::string config_file_path = std::string(std::getenv("CRDC_WS")) + "/../" +
                                 camera_component_config.config_file();
  CameraConfig camera_config;
  crdc::airi::util::get_proto_from_file(config_file_path, &camera_config);

#ifndef WITH_ROS2
  extract_packet(path, folder_path + filename);
#else
  ispacket_ = "no";
  if (std::getenv("ispacket")) {
    ispacket_ = std::string(std::getenv("ispacket"));
  }
  if (!std::getenv("RECORD")) {
    LOG(FATAL) << "RECORD not setting! please check the script or env!!";
  } else {
    record_type_ = std::getenv("RECORD");
  }
  if (!std::getenv("PACKET_FORMAT")) {
    LOG(FATAL) << "PACKET_FORMAT not setting! please check the script or env!!";
  } else {
    packet_format_ = std::string(std::getenv("PACKET_FORMAT"));
  }
  play_save_flag_ = false;
  image_msgs_.header.frame_id = camera_component_config.frame_id();
  image_msgs_.width = camera_config.input_config().width();
  image_msgs_.height = camera_config.input_config().height();
  image_msgs_.step = 3 * camera_config.input_config().width();
  image_msgs_.encoding = "NV12";
  image_msg_.header.frame_id = camera_component_config.frame_id();
  image_msg_.width = camera_config.input_config().width();
  image_msg_.height = camera_config.input_config().height();
  image_msg_.step = 3 * camera_config.input_config().width();
  image_msg_.compression = sensor_msg::msg::Image::RAW;
  if (packet_format_ == "db3") {
    extract_ros2packet(path, folder_path + filename);
  } else if (packet_format_ == "pcap") {
    extract_pcappacket(path, folder_path + filename);
  }
#endif
}

}  // namespace airi
}  // namespace crdc
