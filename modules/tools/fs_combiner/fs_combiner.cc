#include "tools/fs_combiner/fs_combiner.h"
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
void FSCombiner::extract_cloud(const crdc::airi::Packets& packets,
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

void FSCombiner::extract_ins(const crdc::airi::Packets& packets,
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
std::shared_ptr<crdc::airi::LidarParser> FSCombiner::init_lidar_parser(
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

std::shared_ptr<crdc::airi::InsParser> FSCombiner::init_ins_parser(
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
void FSCombiner::extract_packet(const std::string& input_packet_path,
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

  apollo::cyber::record::RecordReader reader(input_packet_path);
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
template <typename T>
void FSCombiner::deserialized_msg_to_bag(rosbag2_cpp::Writer& writer, std::string& channel_type,
                                            std::string& channel_name, T& message,
                                            rcutils_time_point_value_t& time_stamp) {
  auto bag_time = rclcpp::Time(time_stamp);
  rclcpp::Serialization<T> serialization;
  std::shared_ptr<rclcpp::SerializedMessage> serialized =
  std::make_shared<rclcpp::SerializedMessage>();
  serialization.serialize_message(&message, serialized.get());
  writer.write(serialized, channel_name, channel_type, bag_time);
}

template <typename T>
void FSCombiner::timestamp_align(rosbag2_cpp::Reader& reader_lidar, rosbag2_cpp::Writer& writer, auto bag_message,
                                std::unordered_map<std::string, std::string> topic_and_type_lidar,
                                std::unordered_map<std::string, std::string> topic_and_type, 
                                T message, std::string topic_name) {
  while (reader_lidar.has_next()){
    auto bag_message_lidar = reader_lidar.read_next();
    if (0 == topic_and_type_lidar[bag_message_lidar->topic_name].
                                                compare("geometry_msgs/msg/PolygonStamped")) {
      rclcpp::Serialization<geometry_msgs::msg::PolygonStamped> serialization_fs_perception_lidar;
      rclcpp::SerializedMessage extracted_serialized_msg_lidar(*bag_message_lidar->serialized_data);
      serialization_fs_perception_lidar.deserialize_message(&extracted_serialized_msg_lidar,
                                                      &fs_polygon_msg_);
      auto timestamp_camera = message.header.stamp.sec * 1e3 + message.header.stamp.nanosec * 1e-6;
      auto timestamp_lidar = fs_polygon_msg_.header.stamp.sec * 1e3 + fs_polygon_msg_.header.stamp.nanosec * 1e-6;                                               
      if (abs(timestamp_camera - timestamp_lidar) <= diff_timestamp_thres_) {
        deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name], topic_name,
                        message, bag_message->time_stamp);               
        break;                                       
      }                                                
    } 
  }
}

void FSCombiner::extract_ros2packet(const std::string& input_packet_path, const std::string& input_packet_path_lidar,
                                 const std::string& save_pcaket_path) {
  rosbag2_cpp::Reader reader;
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  reader.open(input_packet_path);

  rosbag2_cpp::Reader reader_lidar;
  reader_lidar.open(input_packet_path_lidar);

  rosbag2_cpp::Reader reader_lidar_fs;
  reader_lidar_fs.open(input_packet_path_lidar);

  rosbag2_cpp::Reader reader_lidar_fs_perc;
  reader_lidar_fs_perc.open(input_packet_path_lidar);

  rosbag2_cpp::Reader reader_lidar_ld_perc;
  reader_lidar_ld_perc.open(input_packet_path_lidar);

  int nPos = save_pcaket_path.find_last_of('.');
  storage_options.storage_id = "sqlite3";
  storage_options.uri = save_pcaket_path.substr(0, nPos);

  rcpputils::fs::path rosbag_directory(storage_options.uri);
  rcpputils::fs::remove_all(rosbag_directory);  // remove exists directory, overwriter
  image_decode_ = std::make_shared<crdc::airi::ImageDecodeFS>();
  image_decode_->ffmpeg_init();
  image_decode_->DecodeInit();
  if (0 != record_type_.compare("default")) {
    writer.open(storage_options);
  }

  // camera topics
  std::unordered_map<std::string, std::string> topic_and_type;
  auto topics = reader.get_all_topics_and_types();
  for (const auto & topic_with_type : topics) {
    topic_and_type[topic_with_type.name] = topic_with_type.type;
    std::cout << "name: " << topic_with_type.name << " " << topic_with_type.type << std::endl;
  }
  // lidar topics
  std::unordered_map<std::string, std::string> topic_and_type_lidar;
  auto topics_lidar = reader_lidar.get_all_topics_and_types();
  for (const auto & topic_with_type_lidar : topics_lidar) {
    topic_and_type_lidar[topic_with_type_lidar.name] = topic_with_type_lidar.type;
    std::cout << "name: " << topic_with_type_lidar.name << " " << topic_with_type_lidar.type << std::endl;
  }
  // assign the timestamp of hav_fs_mask to fs_visu_perception
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (0 == bag_message->topic_name.compare("/hav_fs_mask")) {
      rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization_camera;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_camera.deserialize_message(&extracted_serialized_msg,
                                                      &fs_process_image_msgs_);
      fs_process_image_msgs_.header.frame_id = "FS_combine";
      msgs_fs_mask_.push_back(fs_process_image_msgs_);
    } 
  }
  reader.close();

  reader.open(input_packet_path);
  auto counter = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    if (0 == topic_and_type[bag_message->topic_name].
                                                    compare("geometry_msgs/msg/PolygonStamped")) {
      rclcpp::Serialization<geometry_msgs::msg::PolygonStamped> serialization_fs_perception;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_fs_perception.deserialize_message(&extracted_serialized_msg,
                                                      &fs_perc_polygon_msg_);
      common::Singleton<TypeConverAFOutput>::get()->write_polygon_stamped(
                              bag_message->topic_name, fs_perc_polygon_msg_);
      fs_perc_polygon_msg_.header.frame_id = "FS_combine";
      fs_perc_polygon_msg_.header.stamp.sec = msgs_fs_mask_[counter].header.stamp.sec;
      fs_perc_polygon_msg_.header.stamp.nanosec = msgs_fs_mask_[counter].header.stamp.nanosec;
      msgs_fs_visu_.push_back(fs_perc_polygon_msg_);    
      counter++;                                        
    } 
  }
  reader.close();

  reader.open(input_packet_path);
  deflag_ = false;
  diff_timestamp_thres_ = 50;
  counter = 0;
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    path_ = storage_options.uri + bag_message->topic_name;
    if (0 == topic_and_type[bag_message->topic_name].
                                                  compare("sensor_msgs/msg/CompressedImage")) {
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      image_decode_->decode(extracted_serialized_msg, bag_message->topic_name,
                            image_msgs_, path_, deflag_, pressed_msg_);                 
      if (deflag_) {
        if (pressed_msg_.format != "h264") {
          std::string ld = "/hav_ld_mask";
          std::string fs = "/hav_fs_mask";
          image_decode_->get_img_ld(process_img_ld_);
          image_decode_->get_img_fs(process_img_fs_);
          if (!process_img_ld_.empty()) {
            // std::cout << "camera frame_id:" << std::endl;
            image_decode_->imagepublish(ld_process_image_msgs_, process_img_ld_, ld, pressed_msg_);
            process_img_ld_.release();
            ld_process_image_msgs_.header.frame_id = "FS_combine";
            FSCombiner::timestamp_align(reader_lidar, writer, bag_message, topic_and_type_lidar, 
                                        topic_and_type, ld_process_image_msgs_, ld); 
          }
          if (!process_img_fs_.empty()) {
            image_decode_->imagepublish(fs_process_image_msgs_, process_img_fs_, fs, pressed_msg_);
            process_img_fs_.release();
            fs_process_image_msgs_.header.frame_id = "FS_combine";
            FSCombiner::timestamp_align(reader_lidar_fs, writer, bag_message, topic_and_type_lidar, 
                                        topic_and_type, fs_process_image_msgs_, fs);
          }
          deflag_ = false;
          play_save_flag_ = true;
        } else {
          std::cout << "H264 Image advertising is not supported!" << std::endl;
        }
    }
    } else if (0 == topic_and_type[bag_message->topic_name].
                                                    compare("geometry_msgs/msg/PolygonStamped")) {
      fs_perc_polygon_msg_ = msgs_fs_visu_[counter];
      counter++;
      auto timestamp_camera = fs_perc_polygon_msg_.header.stamp.sec * 1e3 + fs_perc_polygon_msg_.header.stamp.nanosec * 1e-6;
      while (reader_lidar_fs_perc.has_next()){
        auto bag_message_lidar = reader_lidar_fs_perc.read_next();
        if (0 == topic_and_type_lidar[bag_message_lidar->topic_name].
                                                    compare("geometry_msgs/msg/PolygonStamped")) {
          rclcpp::Serialization<geometry_msgs::msg::PolygonStamped> serialization_fs_perception_lidar;
          rclcpp::SerializedMessage extracted_serialized_msg_lidar(*bag_message_lidar->serialized_data);
          serialization_fs_perception_lidar.deserialize_message(&extracted_serialized_msg_lidar,
                                                          &fs_polygon_msg_);
          fs_polygon_msg_.header.frame_id = "FS_combine";
          auto timestamp_lidar = fs_polygon_msg_.header.stamp.sec * 1e3 + fs_polygon_msg_.header.stamp.nanosec * 1e-6;                                               
          // std::cout << abs(timestamp_camera - timestamp_lidar) << std::endl;
          if (abs(timestamp_camera - timestamp_lidar) <= diff_timestamp_thres_) {
            // lidar coordinate tranformation
            for (size_t i = 0; i < fs_polygon_msg_.polygon.points.size(); ++i) {
              auto& point = fs_polygon_msg_.polygon.points[i];
              point.x = point.x - 7702.3 * 1e-3;
              point.y = point.y + 1375 * 1e-3;
            }
            // camera coordinate tranformation
            for (size_t i = 0; i < fs_perc_polygon_msg_.polygon.points.size(); ++i) {
              auto& point = fs_perc_polygon_msg_.polygon.points[i];
              point.x = point.x - 7636.5 * 1e-3;
              point.y = point.y - 27.3 * 1e-3;
            }
            // lidar and camera FS fusion
            fs_fusion_lidar_msg_ = fs_polygon_msg_;
            std::string topic_name_fusion = "/hav_fs_fusion";
            for (size_t i = 0; i < fs_perc_polygon_msg_.polygon.points.size(); ++i) {
              auto& point = fs_perc_polygon_msg_.polygon.points[i];
              if (abs(point.y) <= 2) {
                double range = std::sqrt(std::pow(point.x,2) + std::pow(point.y,2));
                double angle = 360;
                if ((point.x >= 0) && (point.y > 0)) {
                  double angle_rad = std::atan2(point.y, point.x);
                  angle = angle_rad * 180.0 / M_PI;
                  // std::cout << "x>=0, y>0, angle: " << angle << std::endl;
                } else if ((point.x >= 0) && (point.y < 0)) {
                  double angle_rad = std::atan2(abs(point.y), point.x);
                  angle = 360.0 - angle_rad * 180.0 / M_PI;
                  // std::cout << "x>=0, y<0, angle: " << angle << std::endl;
                } else if (point.y == 0) {
                  angle = 0;
                  // std::cout << "y = 0, angle: " << angle << std::endl;
                }
                int angle_id = std::ceil(angle);
                double x_lidar = fs_fusion_lidar_msg_.polygon.points[angle_id].x;
                double y_lidar = fs_fusion_lidar_msg_.polygon.points[angle_id].y;
                double range_lidar = std::sqrt(std::pow(x_lidar,2) + std::pow(y_lidar,2));
                std::vector<double> A(7, 0.0);

                for (int j = 0; j < 7; ++j) {
                  int tem_id = angle_id + 3 - j;
                  if (tem_id < 0) {
                    tem_id += 360;
                  } else if (tem_id >= 360) {
                    tem_id -= 360;
                  }
                  double tem_x_lidar = fs_fusion_lidar_msg_.polygon.points[tem_id].x;
                  double tem_y_lidar = fs_fusion_lidar_msg_.polygon.points[tem_id].y;
                  double Dis = std::pow((point.x-tem_x_lidar),2) + std::pow((point.y-tem_y_lidar),2);
                  A[j] = Dis;
                }
                 double mindis = *std::min_element(A.begin(), A.end());
                 if (range < 10) {
                   if (range < range_lidar && mindis > 0.25) {
                     fs_fusion_lidar_msg_.polygon.points[angle_id].x = point.x;
                     fs_fusion_lidar_msg_.polygon.points[angle_id].y = point.y;
                   }
                 } else if (range < 25) {
                   if (range < range_lidar && mindis > 2.25) {
                     fs_fusion_lidar_msg_.polygon.points[angle_id].x = point.x;
                     fs_fusion_lidar_msg_.polygon.points[angle_id].y = point.y;
                   }
                 } else if (range < 50) {
                   if (range < range_lidar && mindis > 6.25) {
                     fs_fusion_lidar_msg_.polygon.points[angle_id].x = point.x;
                     fs_fusion_lidar_msg_.polygon.points[angle_id].y = point.y;
                   }
                 } else {
                   if (range < range_lidar) {
                     fs_fusion_lidar_msg_.polygon.points[angle_id].x = point.x;
                     fs_fusion_lidar_msg_.polygon.points[angle_id].y = point.y;
                   }
                 }
              }
            }

            deserialized_msg_to_bag(writer, topic_and_type_lidar[bag_message_lidar->topic_name],
                          bag_message_lidar->topic_name, fs_polygon_msg_, bag_message_lidar->time_stamp);
            deserialized_msg_to_bag(writer, topic_and_type[bag_message->topic_name],
                          bag_message->topic_name, fs_perc_polygon_msg_, bag_message->time_stamp);    
            deserialized_msg_to_bag(writer, topic_and_type_lidar[bag_message_lidar->topic_name],
                          topic_name_fusion, fs_fusion_lidar_msg_, bag_message_lidar->time_stamp);
            break;                                       
          }                                                
        } 
      }
                                                      
    } else if (0 == topic_and_type[bag_message->topic_name].
                                                    compare("visualization_msgs/msg/Marker")) {
      rclcpp::Serialization<visualization_msgs::msg::Marker> serialization_ld_perception;
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization_ld_perception.deserialize_message(&extracted_serialized_msg,
                                                      &ld_marker_msgs_);
      common::Singleton<TypeConverAFOutput>::get()->write_marker(
                              bag_message->topic_name, ld_marker_msgs_);
      ld_marker_msgs_.header.frame_id = "FS_combine";
      FSCombiner::timestamp_align(reader_lidar_ld_perc, writer, bag_message, topic_and_type_lidar, 
                                  topic_and_type, ld_marker_msgs_, bag_message->topic_name);                                              
    }
  }
  reader.close();
  reader_lidar.close();
  reader_lidar_fs.close();
  image_decode_->clear_image_map();
  if (deflag_ || play_save_flag_) {
    std::cout << save_pcaket_path << " write over!" << std::endl;
  } else {
    std::cout << "no data is saved, please check the topic type!!" << std::endl;
  }
}
#endif

void FSCombiner::creat_folder(const std::string &folder_path) {
  std::string command;
  command = "mkdir -p " + folder_path;
  if (system(command.c_str())) {
    LOG(WARNING) << "[FSCombiner] Failed to create dir " << folder_path;
  }
}

void FSCombiner::write_packet(const std::string& path, const std::string& path_lidar) {
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
  std::cout << "camera config_file_path:" << config_file_path << std::endl;
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
  extract_ros2packet(path, path_lidar, folder_path + filename);
#endif
}

}  // namespace airi
}  // namespace crdc
