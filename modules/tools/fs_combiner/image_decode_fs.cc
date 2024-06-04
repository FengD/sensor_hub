#include "tools/fs_combiner/image_decode_fs.h"

namespace crdc {
namespace airi {
#ifdef WITH_ROS2

void ImageDecodeFS::DecodeInit() {
  params_.resize(3, 0);
  params_[0] = CV_IMWRITE_JPEG_QUALITY;
  params_[1] = 50;
}

template <typename T>
void ImageDecodeFS::decode_h264frame(std::string& image_name, T& decode_msg, bool& deflag) {
  if (!std::getenv("TYPE")) {
    LOG(FATAL) << "TYPE not setting! please check the script or env!!";
  }
  std::string extract_type = std::getenv("TYPE");
  int one_image_size = frame->width * frame->height, offset = 0;
  int num = one_image_size * 3 / 2;
  std::vector<uint8_t> data(num);
  std::memcpy(&data[offset], &frame->data[0][0], one_image_size);
  offset += one_image_size;
  for (int i = 0; i < frame->height / 2; ++i) {
    int j = 0;
    while (j < frame->width / 2) {
      std::memcpy(&data[offset], &frame->data[1][i * frame->linesize[1] + j], 1);
      offset += 1;
      std::memcpy(&data[offset], &frame->data[2][i * frame->linesize[2] + j], 1);
      offset += 1;
      j++;
    }
  }
  decode_msg.data.resize(num);
  std::memcpy(decode_msg.data.data(), data.data(), num);
  deflag = true;

  int height = static_cast<int>(decode_msg.height);
  int width = static_cast<int>(decode_msg.width);
  cv::Mat yuv_input(height*3/2, width, CV_8UC1,
  static_cast<void*>(const_cast<unsigned char*>(decode_msg.data.data())));
  cv::Mat res(height, width, CV_8UC3);
  cv::cvtColor(yuv_input, res, cv::COLOR_YUV2BGR_NV12);

  if (0 == extract_type.compare("jpg")) {
    if (access(path_.c_str(), 0) == -1) {
      if (system(("mkdir -p " + path_).c_str())) {
        LOG(WARNING) << "[jpgDecode] Failed to create dir " << path_;
      }
    }
    image_name += ".jpg";
    cv::imwrite(image_name, res);
  } else if (0 == extract_type.compare("png")) {
    if (access(path_.c_str(), 0) == -1) {
      if (system(("mkdir -p " + path_).c_str())) {
        LOG(WARNING) << "[pngDecode] Failed to create dir " << path_;
      }
    }
    image_name += ".png";
    cv::imwrite(image_name, res);
  } else if (0 == extract_type.compare("yuv")) {
    if (access(path_.c_str(), 0) == -1) {
      if (system(("mkdir -p " + path_).c_str())) {
        LOG(WARNING) << "[yuvDecode] Failed to create dir " << path_;
      }
    }
    image_name += ".yuv";
    int32_t yuv_size = height * width * 3 / 2;
    FILE *yuvfd = fopen(image_name.c_str(), "w+");
    fwrite(yuv_input.ptr<uint8_t>(), 1, yuv_size, yuvfd);
    fclose(yuvfd);
  }
}

template <typename T>
void ImageDecodeFS::decode_cvframe(std::string topic_name, cv::Mat& image_jpeg_,
                                 std::string& image_name, T& decode_msg, bool& deflag) {
  int height = image_jpeg_.rows;
  int width = image_jpeg_.cols;
  int area = height * width;
  std::size_t slashPos = image_name.find_last_of('/');
  std::string info = image_name.substr(slashPos + 1);
  cv::Mat image_jpeg_out = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat img_yuv(height * 1.5, width, CV_8UC1);
  if (!std::getenv("TYPE")) {
    LOG(FATAL) << "TYPE not setting! please check the script or env!!";
  }
  std::string extract_type = std::getenv("TYPE");

  if (!std::getenv("mask")) {
    LOG(FATAL) << "mask is not set! please check the script or env!!";
  }
  save_mask_ = std::getenv("mask");
  if (topic_name.compare("/hav_ld_mask") == 0) {
    cv::cvtColor(image_jpeg_, decode_ld_mask_8uc1_, CV_BGR2GRAY);
    // median filter
    ld_filter_mask = cv::Mat::zeros(decode_ld_mask_8uc1_.size(), CV_8UC1);
    for (int i = 1; i < 10; i++) {
      cv::Mat new_mask = cv::Mat::zeros(decode_ld_mask_8uc1_.size(), CV_8UC1);
      decode_ld_mask_8uc1_.copyTo(new_mask, decode_ld_mask_8uc1_ == i);
      cv::Mat img_median;
      medianBlur(new_mask, img_median, 3);
      img_median.copyTo(ld_filter_mask, img_median == i);
    }
    for (int i = 0; i < ld_filter_mask.rows; i++) {
      uchar* ptr = ld_filter_mask.ptr<uchar>(i);
      for (int j = 0; j < ld_filter_mask.cols; j++) {
        switch (ptr[j]) {
          case 1:  // SINGLE_SOLID_W   BLUE
            image_jpeg_out.at<cv::Vec3b>(i, j)[0] = 255;
          break;
          case 2:  // SINGLE_SOLID_Y   GREEN
            image_jpeg_out.at<cv::Vec3b>(i, j)[1] = 255;
          break;
          case 3:  // SINGLE_SOLID_B   RED
            image_jpeg_out.at<cv::Vec3b>(i, j)[2] = 255;
          break;
          case 4:  // SINGLE_DASH_ W  YELLOW
            image_jpeg_out.at<cv::Vec3b>(i, j)[1] = 255;
            image_jpeg_out.at<cv::Vec3b>(i, j)[2] = 255;
          break;
          case 5:  // SINGLE_DASH_Y    PINK
            image_jpeg_out.at<cv::Vec3b>(i, j)[0] = 255;
            image_jpeg_out.at<cv::Vec3b>(i, j)[2] = 255;
          case 6:  // DOUBLE_SOILD_Y   BLUE-GREEN
            image_jpeg_out.at<cv::Vec3b>(i, j)[0] = 255;
            image_jpeg_out.at<cv::Vec3b>(i, j)[1] = 255;
          case 7:  // DOUBLE_SOILD_W   ORANGE
            image_jpeg_out.at<cv::Vec3b>(i, j)[0] = 27;
            image_jpeg_out.at<cv::Vec3b>(i, j)[1] = 163;
            image_jpeg_out.at<cv::Vec3b>(i, j)[2] = 247;
          case 8:  // DOUBLE_DASH_Y   huise  GRAY
            image_jpeg_out.at<cv::Vec3b>(i, j)[0] = 162;
            image_jpeg_out.at<cv::Vec3b>(i, j)[1] = 156;
            image_jpeg_out.at<cv::Vec3b>(i, j)[2] = 166;
          case 9: // DOUBLE_DASH_W   VIOLET
            image_jpeg_out.at<cv::Vec3b>(i, j)[0] = 94;
            image_jpeg_out.at<cv::Vec3b>(i, j)[1] = 6;
            image_jpeg_out.at<cv::Vec3b>(i, j)[2] = 156;
          default:
          break;
        }
      }
    }
    ld_mask_map_[info] = image_jpeg_out;
  } else if (topic_name.compare("/hav_fs_mask") == 0) {
    cv::cvtColor(image_jpeg_, decode_fs_mask_8uc1_, CV_BGR2GRAY);
    for (int i = 0; i < decode_fs_mask_8uc1_.rows; i++) {
      uchar* ptr = decode_fs_mask_8uc1_.ptr<uchar>(i);
      for (int j = 0; j < decode_fs_mask_8uc1_.cols; j++) {
        if (ptr[j] == 1) {
          image_jpeg_out.at<cv::Vec3b>(i, j)[0] = 255;
        }
      }
    }
    fs_mask_map_[info] = image_jpeg_out;
  } else {
    image_jpeg_out = image_jpeg_;
    ld_image_map_[info] = image_jpeg_out;
    fs_image_map_[info] = image_jpeg_out;

    cv::Mat yv12_img;
    cv::cvtColor(image_jpeg_, yv12_img, CV_BGR2YUV_YV12);
    memcpy(img_yuv.data, yv12_img.data, area);
    char* v = (char*)yv12_img.data + area;
    char* u = v + area / 4;
    char* dst = (char*)img_yuv.data + area;
    for(int i = 0; i < area / 4; ++i) {
      dst[2 * i] = u[i];
      dst[2 * i + 1] = v[i];
    }
    decode_msg.data.resize(img_yuv.rows * img_yuv.cols);
    std::memcpy(decode_msg.data.data(), img_yuv.data, img_yuv.rows * img_yuv.cols);
  }
  if (!ld_image_map_.empty() && !ld_mask_map_.empty()) {
    auto ld_it = ld_mask_map_.begin();
    while (ld_it != ld_mask_map_.end()) {
      auto img_it = ld_image_map_.begin();
      while (img_it != ld_image_map_.end()) {
        rear_image_crop_ = img_it->second(cv::Rect(0, 471, 1920, 608));
        if (ld_it->first == img_it->first) {
          cv::addWeighted(rear_image_crop_, 1, ld_it->second, 0.5, 0.0, process_img_ld_);
          ld_image_map_.erase(img_it);
          ld_mask_map_.erase(ld_it);
          deflag = true;
          break;
        } else {
          ++img_it;
        }
      }
      if (deflag) {
        break;
      } else {
        ++ld_it;
      }
    }
  }
  if (!fs_image_map_.empty() && !fs_mask_map_.empty()) {
    auto fs_it = fs_mask_map_.begin();
    while (fs_it != fs_mask_map_.end()) {
      auto img_it = fs_image_map_.begin();
      while (img_it != fs_image_map_.end()) {
        rear_image_crop_ = img_it->second(cv::Rect(0, 471, 1920, 608));
        if (fs_it->first == img_it->first) {
          cv::addWeighted(rear_image_crop_, 1, fs_it->second, 0.5, 0.0, process_img_fs_);
          fs_image_map_.erase(img_it);
          fs_mask_map_.erase(fs_it);
          deflag = true;
          break;
        } else {
          ++img_it;
        }
      }
      if (deflag) {
        break;
      } else {
        ++fs_it;
      }
    }
  }

  if (0 != extract_type.compare("default") && 0 != extract_type.compare("skip")) {
    if (access(path_.c_str(), 0) == -1 &&
       ((topic_name.compare("/hav_ld_mask") != 0 && topic_name.compare("/hav_fs_mask") != 0)
        || save_mask_.compare("save") == 0)) {
      if (system(("mkdir -p " + path_).c_str())) {
        LOG(WARNING) << "[maskDecode] Failed to create dir " << path_;
      }
    }
    deflag = true;
  }

  if (0 == extract_type.compare("jpg")) {
    image_name += ".jpg";
    if (topic_name.compare("/hav_ld_mask") != 0 && topic_name.compare("/hav_fs_mask") != 0) {
      cv::imwrite(image_name, image_jpeg_out);
    } else if (topic_name.compare("/hav_ld_mask") == 0 && save_mask_.compare("save") == 0) {
      cv::imwrite(image_name, ld_filter_mask);
    } else if (topic_name.compare("/hav_fs_mask") == 0 && save_mask_.compare("save") == 0) {
      cv::imwrite(image_name, decode_fs_mask_8uc1_);
    }
  } else if (0 == extract_type.compare("png")) {
    image_name += ".png";
    if (topic_name.compare("/hav_ld_mask") != 0 && topic_name.compare("/hav_fs_mask") != 0) {
      cv::imwrite(image_name, image_jpeg_out);
    } else if (topic_name.compare("/hav_ld_mask") == 0 && save_mask_.compare("save") == 0) {
      cv::imwrite(image_name, ld_filter_mask);
    } else if (topic_name.compare("/hav_fs_mask") == 0 && save_mask_.compare("save") == 0) {
      cv::imwrite(image_name, decode_fs_mask_8uc1_);
    }
  } else if (0 == extract_type.compare("yuv")) {
    image_name += ".yuv";
    if (topic_name.compare("/hav_ld_mask") != 0 && topic_name.compare("/hav_fs_mask") != 0) {
      int32_t yuv_size = height * width * 3 / 2;
      FILE *yuvfd = fopen(image_name.c_str(), "w+");
      fwrite(img_yuv.ptr<uint8_t>(), 1, yuv_size, yuvfd);
      fclose(yuvfd);
    }
  }
}

void ImageDecodeFS::decode(rclcpp::SerializedMessage& extracted_serialized_msg,
                         std::string topic_name, sensor_msgs::msg::Image& decode_msg,
                         std::string path, bool& deflag, sensor_msgs::msg::CompressedImage& encodeframes) {
  path_ = path;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization_camera;
  serialization_camera.deserialize_message(&extracted_serialized_msg, &encodeframes);
  std::string image_name = path_ + "/" + std::to_string(
                              static_cast<uint64_t>(encodeframes.header.stamp.sec) * 1000000 +
                              static_cast<uint64_t>(encodeframes.header.stamp.nanosec) / 1000);
  decode_msg.header.stamp.sec = encodeframes.header.stamp.sec;
  decode_msg.header.stamp.nanosec = encodeframes.header.stamp.nanosec;
  if (encodeframes.format == "h264") {
    if (av_packet_from_data(packet, encodeframes.data.data(), encodeframes.data.size()) < 0) {
      fprintf(stderr, "av_packet_from_data failed.\n");
      exit(1);
    }

    if (avcodec_send_packet(context, packet) < 0) {
      fprintf(stderr, "avcodec_send_packet failed.\n");
      exit(1);
    }
    if (avcodec_receive_frame(context, frame) == 0) {
      decode_h264frame(image_name, decode_msg, deflag);
    }
  } else {
    std::vector<uint8_t> compressed_buffer(encodeframes.data.begin(), encodeframes.data.end());
    cv::Mat image_jpeg = cv::imdecode(compressed_buffer, CV_LOAD_IMAGE_UNCHANGED);
    decode_cvframe(topic_name, image_jpeg, image_name, decode_msg, deflag);
  }
}

void ImageDecodeFS::decode(rclcpp::SerializedMessage& extracted_serialized_msg,
                         std::string topic_name, sensor_msg::msg::Image& decode_msg,
                         std::string path, bool& deflag, sensor_msg::msg::Image& encodeframe) {
  path_ = path;
  rclcpp::Serialization<sensor_msg::msg::Image> serialization_camera;
  serialization_camera.deserialize_message(&extracted_serialized_msg, &encodeframe);
  std::string image_name = path_ + "/" + std::to_string(encodeframe.header.camera_timestamp);
  uint64_t frame_time = encodeframe.header.camera_timestamp;
  decode_msg.header.camera_timestamp = frame_time;
  decode_msg.header.timestamp_sec = static_cast<double>(frame_time) / 1e9;
  decode_msg.header.sequence_num = encodeframe.header.sequence_num;
  decode_msg.exposuretime = 1;
  decode_msg.type = "NV12";
  if (encodeframe.compression == sensor_msg::msg::Image::H264) {
    if (av_packet_from_data(packet, encodeframe.data.data(), encodeframe.data.size()) < 0) {
      fprintf(stderr, "av_packet_from_data failed.\n");
      exit(1);
    }

    if (avcodec_send_packet(context, packet) < 0) {
      fprintf(stderr, "avcodec_send_packet failed.\n");
      exit(1);
    }
    if (avcodec_receive_frame(context, frame) == 0) {
      decode_h264frame(image_name, decode_msg, deflag);
    }
  } else {
    std::vector<uint8_t> compressed_buffer(encodeframe.data.begin(), encodeframe.data.end());
    cv::Mat image_jpeg = cv::imdecode(compressed_buffer, CV_LOAD_IMAGE_UNCHANGED);
    cv::cvtColor(image_jpeg, image_jpeg, CV_BGR2RGB);
    decode_cvframe(topic_name, image_jpeg, image_name, decode_msg, deflag);
  }
}

void ImageDecodeFS::ffmpeg_init() {
  if ((codec = avcodec_find_decoder(AV_CODEC_ID_H264)) == NULL) {
    fprintf(stderr, "avcodec_find_decoder failed.\n");
    exit(1);
  }

  if ((context = avcodec_alloc_context3(codec)) == NULL) {
    fprintf(stderr, "avcodec_alloc_context3 failed.\n");
    exit(1);
  }
  context->codec_type = AVMEDIA_TYPE_VIDEO;
  context->pix_fmt = AV_PIX_FMT_YUV420P;
  if (avcodec_open2(context, codec, NULL) < 0) {
    fprintf(stderr, "avcodec_open2 failed.\n");
    avcodec_free_context(&context);
  }

  if ((packet = av_packet_alloc()) == NULL) {
    fprintf(stderr, "av_packet_alloc failed.\n");
    avcodec_close(context);
  }

  if ((frame = av_frame_alloc()) == NULL) {
    fprintf(stderr, "av_frame_alloc failed.\n");
    av_packet_free(&packet);
  }
}

void ImageDecodeFS::imagepublish(sensor_msg::msg::Image& image_msg_, cv::Mat& process_img_,
                            std::string channel_name, sensor_msg::msg::Image& encode_msg) {
  std::vector<uint8_t> compressed_buffer_;
  image_msg_.header.camera_timestamp = encode_msg.header.camera_timestamp;
  image_msg_.width = 1920;
  image_msg_.height = 608;
  compressed_buffer_.clear();
  if (!cv::imencode(".jpg", process_img_, compressed_buffer_, params_)) {
      std::cout<< "Failed to encode " << channel_name << " jpg format." << std::endl;
  }
  image_msg_.data.resize(compressed_buffer_.size());
  std::memcpy(image_msg_.data.data(), compressed_buffer_.data(), compressed_buffer_.size());
  common::Singleton<TypeConverAFOutput>::get()->write_image(
                                    channel_name, image_msg_);
}

void ImageDecodeFS::imagepublish(sensor_msgs::msg::CompressedImage& image_msg_, cv::Mat& process_img_,
                            std::string channel_name, sensor_msgs::msg::CompressedImage& encode_msg) {
  std::vector<uint8_t> compressed_buffer_;
  image_msg_.header.stamp = encode_msg.header.stamp;
  image_msg_.header.frame_id = encode_msg.header.frame_id;
  image_msg_.format = "jpeg";
  compressed_buffer_.clear();
  if (!cv::imencode(".jpg", process_img_, compressed_buffer_, params_)) {
      std::cout<< "Failed to encode " << channel_name << " jpg format." << std::endl;
  }
  image_msg_.data.resize(compressed_buffer_.size());
  std::memcpy(image_msg_.data.data(), compressed_buffer_.data(), compressed_buffer_.size());
  common::Singleton<TypeConverAFOutput>::get()->write_encodeimage(
                                            channel_name, image_msg_);
}

void ImageDecodeFS::get_img_ld(cv::Mat& img_ld) {
  img_ld = process_img_ld_;
  process_img_ld_.release();
}

void ImageDecodeFS::get_img_fs(cv::Mat& img_fs) {
  img_fs = process_img_fs_;
  process_img_fs_.release();
}

void ImageDecodeFS::clear_image_map() {
  fs_mask_map_.clear();
  ld_mask_map_.clear();
  fs_image_map_.clear();
  ld_image_map_.clear();
}
#endif

}  // namespace airi
}  // namespace crdc
