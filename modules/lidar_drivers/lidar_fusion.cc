// Copyright (C) 2022 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar fusion

#include "lidar_drivers/lidar_fusion.h"
#include <utility>
namespace crdc {
namespace airi {

LidarFusion::~LidarFusion() {
  clouds_queue_.break_all_wait();
  for (const auto& lidar : lidars_) {
    if (lidar->is_alive()) {
        lidar->join();
    }
    LOG(INFO) << "[LIDAR_MAIN] lidar[" << lidar->get_name() << "] joined successfully.";
  }
}

LidarFusion::LidarFusion(const LidarComponent& config) : common::Thread(true) {
  config_ = config;
  stop_ = false;
  std::string thread_name = config_.frame_id();
  if (thread_name.length() > 13) {
    thread_name = thread_name.substr(0, 12);
  }
  set_thread_name(thread_name);

  if (config_.has_priority()) {
    set_priority(config_.priority());
  }

  LOG(INFO) << "[" << get_thread_name() << "] " << "lidar fusion: " << config_.DebugString();
  if (config_.has_channel_name()) {
    compensator_.reset(new Compensator());
    compensator_->init(config_);
  }

  cloud_array_.resize(2);
  cloud_array_[0].stricted_strategy_ = true;
  cloud_array_[0].max_timestamp_gap_ = config_.stricted_max_time_gap();
  cloud_array_[1].stricted_strategy_ = false;
  cloud_array_[1].max_timestamp_gap_ = config_.latest_max_time_gap();
  for (const auto& cfg : config_.component_config()) {
    std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(cfg);
    lidars_.emplace_back(lidar);

    if (cfg.has_stricted_bundle() && cfg.stricted_bundle()) {
      cloud_array_[0].sensors_.insert(cfg.frame_id());
    } else {
      cloud_array_[1].sensors_.insert(cfg.frame_id());
    }
  }

  for (size_t i = 0; i < cloud_array_.size(); i++) {
    LOG(INFO) << "[" << get_thread_name() << "] " << "cloud array[" << i << "]:";
    for (auto iter = cloud_array_[i].sensors_.begin();
         iter != cloud_array_[i].sensors_.end(); ++iter) {
      LOG(INFO) << "[" << get_thread_name() << "] " << "sensor: " << *iter;
    }
  }
  if (config_.has_channel_name() || config_.has_calibrate_module()) {
    for (auto& lidar : lidars_) {
      lidar->set_callback(std::bind(&LidarFusion::process, this, std::placeholders::_1));
    }
  }
}

void LidarFusion::process(const std::shared_ptr<LidarPointCloud>& cloud) {
  std::lock_guard<std::mutex> lock(lock_);

  bool is_stricted_strategy = false;
#ifdef WITH_ROS2
  auto& sensor = cloud->cloud_msg_->header.frame_id;
#else
  auto& sensor = cloud->proto_cloud_->header().frame_id();
#endif
  for (size_t i = 0; i < cloud_array_.size(); i++) {
    if (cloud_array_[i].sensors_.find(sensor) != cloud_array_[i].sensors_.end()) {
      cloud_array_[i].clouds_[sensor] = cloud;
      if (cloud_array_[i].stricted_strategy_) {
        is_stricted_strategy = true;
      }
    }
  }

  if (is_stricted_strategy) {
#ifdef WITH_ROS2
    assemble(cloud->cloud_msg_->header.stamp.sec * 1000000 +
             cloud->cloud_msg_->header.stamp.nanosec / 1000);
#else
    assemble(cloud->proto_cloud_->header().lidar_timestamp());
#endif
  }
}

void LidarFusion::assemble(const uint64_t& timestamp) {
  for (size_t i = 0; i < cloud_array_.size(); i++) {
    for (auto iter = cloud_array_[i].clouds_.begin(); iter != cloud_array_[i].clouds_.end();) {
#ifdef WITH_ROS2
      uint64_t lidar_timestamp = iter->second->cloud_msg_->header.stamp.sec * 1000000 +
                                 iter->second->cloud_msg_->header.stamp.nanosec / 1000;
      uint64_t gap = timestamp > lidar_timestamp ?
                      timestamp - lidar_timestamp :
                      lidar_timestamp - timestamp;
#else
      uint64_t gap = timestamp > iter->second->proto_cloud_->header().lidar_timestamp() ?
                      timestamp - iter->second->proto_cloud_->header().lidar_timestamp() :
                      iter->second->proto_cloud_->header().lidar_timestamp() - timestamp;

#endif
      if (gap > cloud_array_[i].max_timestamp_gap_) {
        LOG(WARNING) << "[" << get_thread_name() << "] " << "remove cloud: "
              << iter->first << ", gap is too big: " << gap;
        cloud_array_[i].clouds_.erase(iter++);
      } else {
        iter++;
      }
    }
  }

  if (cloud_array_[0].clouds_.size() == cloud_array_[0].sensors_.size() &&
        cloud_array_[1].clouds_.size() == cloud_array_[1].sensors_.size()) {
    std::vector<std::shared_ptr<LidarPointCloud>> all_clouds;
    all_clouds.resize(config_.component_config_size());
    for (int i = 0; i< config_.component_config_size(); ++i) {
      auto& cfg = config_.component_config(i);
      bool miss_cloud = true;
      for (size_t j = 0; j < cloud_array_.size(); j++) {
        if (cloud_array_[j].clouds_.find(cfg.frame_id()) != cloud_array_[j].clouds_.end()) {
          miss_cloud = false;
          all_clouds[i] = cloud_array_[j].clouds_[cfg.frame_id()];
        }
      }
      if (miss_cloud) {
        LOG(ERROR) << "[" << get_thread_name() << "] " << "miss cloud: " << cfg.frame_id();
        return;
      }
    }
    clouds_queue_.enqueue(all_clouds);
    cloud_array_[0].clouds_.clear();
  }
}

void LidarFusion::remove_clouds_point(std::shared_ptr<PointClouds2>& clouds) {
  if (!config_.has_blind_zone() ||
        !config_.blind_zone().has_delete_point() ||
        !config_.blind_zone().delete_point()) {
    return;
  }

#ifdef WITH_ROS2
  for (size_t i = 0; i < clouds->clouds.size(); ++i) {
    float x_blind_zone = config_.blind_zone().max_x();
    char *data_buf = reinterpret_cast<char *>(clouds->clouds[i].data.data());
    check_blind_zone_points(i, x_blind_zone, data_buf, clouds);
  }
#else
  for (size_t i = 0; i < clouds->clouds_size(); ++i) {
    float x_blind_zone = config_.blind_zone().max_x();
    char *data_buf = const_cast<char *>(clouds->mutable_clouds(i)->mutable_data()->data());
    check_blind_zone_points(i, x_blind_zone, data_buf, clouds);
  }
#endif
}

void LidarFusion::check_blind_zone_points(const int& i,
                                          const float& x_blind_zone,
                                          char *data_buf,
                                          std::shared_ptr<PointClouds2>& clouds) {
  LidarPoint* pt = reinterpret_cast<LidarPoint*>(data_buf);
#ifdef WITH_ROS2
  auto cloud_size = clouds->clouds[i].width * clouds->clouds[i].height;
#else
  auto cloud_size = clouds->clouds(i).width() * clouds->clouds(i).height();
#endif
  for (size_t j = 0; j < cloud_size; ++j, ++pt) {
    if (pt->x_ >= config_.blind_zone().min_x() &&
        pt->x_ <= x_blind_zone &&
        pt->y_ >= config_.blind_zone().min_y() &&
        pt->y_ <= config_.blind_zone().max_y()) {
      if (config_.blind_zone().has_min_z() && config_.blind_zone().has_max_z() &&
          pt->z_ >= config_.blind_zone().min_z() &&  pt->z_ <= config_.blind_zone().max_z()) {
        continue;
      }
      reset_point(pt);
    }
  }
}

void LidarFusion::reset_point(LidarPoint* pt) {
  pt->x_ = 0.0;
  pt->y_ = 0.0;
  pt->z_ = 0.0;
  pt->intensity_ = 0.0;
}

void LidarFusion::stop() {
  LOG(INFO) << "[" << get_thread_name() << "] " << "lidar fusion stopped";
  for (auto& lidar : lidars_) {
    lidar->stop();
  }
  stop_ = true;
  clouds_queue_.break_all_wait();
}

void LidarFusion::parse_point_to_pcl_cloud(std::shared_ptr<PointCloud2> &pcd2_msg,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  cloud->clear();
#ifdef WITH_ROS2
  for (uint32_t h = 0; h < pcd2_msg->height; ++h) {
    auto data = pcd2_msg->data.data() + pcd2_msg->row_step * h;
    for (uint32_t w = 0; w < pcd2_msg->width; ++w) {
      auto p = data + 8;
      memcpy(&point_, p, sizeof(pcl::PointXYZI));
      cloud->push_back(point_);
      data += pcd2_msg->point_step;
    }
  }
#else
  for (uint32_t h = 0; h < pcd2_msg->height(); ++h) {
    auto data = pcd2_msg->data().data() + pcd2_msg->row_step() * h;
    for (uint32_t w = 0; w < pcd2_msg->width(); ++w) {
      // start with the offset x = 8, which is defiend in sensor_hubs lidar parser.cc
      auto p = data + 8;
      memcpy(&point_, p, sizeof(pcl::PointXYZI));
      cloud->push_back(point_);
      data += pcd2_msg->point_step();
    }
  }
#endif
}

#if (defined WITH_TDA4) || (defined CALIBRATE)
void LidarFusion::calibrate(const std::vector<std::shared_ptr<LidarPointCloud>> &clouds) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr fl_cloud, fr_cloud, rl_cloud, rr_cloud;
  fl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  fr_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  rl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  rr_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
#ifdef WITH_ROS2
  parse_point_to_pcl_cloud(clouds[0]->cloud_msg_, fl_cloud);
  parse_point_to_pcl_cloud(clouds[1]->cloud_msg_, fr_cloud);

  if (clouds.size() == 4) {
    parse_point_to_pcl_cloud(clouds[2]->cloud_msg_, rl_cloud);
    parse_point_to_pcl_cloud(clouds[3]->cloud_msg_, rr_cloud);
  }
#else
  parse_point_to_pcl_cloud(clouds[0]->proto_cloud_, fl_cloud);
  parse_point_to_pcl_cloud(clouds[1]->proto_cloud_, fr_cloud);

  if (clouds.size() == 4) {
    parse_point_to_pcl_cloud(clouds[2]->proto_cloud_, rl_cloud);
    parse_point_to_pcl_cloud(clouds[3]->proto_cloud_, rr_cloud);
  }
#endif
  int ret = lidars_calibration_->process(fl_cloud, fr_cloud, rl_cloud, rr_cloud);
#ifdef WITH_TDA4
  if (!ret) {
    lidars_calibration_->set_calib_end();
  }
#endif
}
void LidarFusion::vehicle_module_judge() {
  if (config_.has_calibrate_module()) {
#ifdef WITH_TDA4
    std::string calibration_config = std::string(std::getenv("CRDC_WS")) + "/params/" +
                                     "lidars_calibration/" + "hav3g/" +
                                     "/lidars_calibration.prototxt";
    lidars_calibration_ = std::make_shared<Calibrate>();
#else
    std::string calibration_config = std::string(std::getenv("CRDC_WS")) + "params/" +
                                     std::getenv("vehicle_type") + "/lidars_calibration.prototxt";
    lidars_calibration_ = std::make_shared<LidarsCalibration>();
#endif
    if (!crdc::airi::util::is_path_exists(calibration_config)) {
      LOG(FATAL) << "Calibration_config not exists.";
    }
    lidars_calibration_->init(calibration_config);

    while (!stop_) {
#ifdef WITH_TDA4
      if (lidars_calibration_->get_vehicle_mode() != 2 ||
          lidars_calibration_->get_calib_mode() != 1) {
        for (int i = 0; i < config_.component_config_size(); ++i) {
          auto &cfg = config_.component_config(i);
          if (!cfg.has_calibrate()) {
            lidars_[i]->set_flag(false);
          } else {
            lidars_[i]->set_flag(true);
            if (lidars_calibration_->get_calib_mode() == 2) {
              lidars_[i]->set_iftransform(true);
              lidars_[i]->get_calbrate_server(lidars_calibration_);
            }
          }
        }
        sleep(1);
        continue;
      } else {
        for (int i = 0; i < config_.component_config_size(); ++i) {
          lidars_[i]->set_flag(true);
          auto &cfg = config_.component_config(i);
          if (cfg.has_calibrate()) {
            lidars_[i]->set_iftransform(false);
          }
        }
      }
#endif

      std::vector<std::shared_ptr<LidarPointCloud>> clouds;
      if (!clouds_queue_.wait_for_dequeue(&clouds)) {
        LOG(WARNING) << "[" << get_thread_name() << "] "
                     << "wait queue failed";
        continue;
      }
      calibrate(clouds);
    }
  }
}
#endif

void LidarFusion::run() {
  for (auto &lidar : lidars_) {
    lidar->start();
  }

#if (defined WITH_TDA4) || (defined CALIBRATE)
  vehicle_module_judge();
#endif

  if (config_.has_channel_name()) {
    std::shared_ptr<CompensationInfo> compensation_info = std::make_shared<CompensationInfo>();
    while (!stop_) {
      std::vector<std::shared_ptr<LidarPointCloud>> clouds;
      uint64_t start_time = get_now_microsecond();
      if (!clouds_queue_.wait_for_dequeue(&clouds)) {
        LOG(WARNING) << "[" << get_thread_name() << "] "
                     << "wait queue failed";
        continue;
      }

      if (clouds_queue_.size() != 0) {
        LOG(WARNING) << "[" << get_thread_name() << "] "
                     << "too many data in queue: " << clouds_queue_.size() << ", skip this one";
        continue;
      }
      LOG(INFO) << "[" << get_thread_name() << "] [TIMER] "
                << "[wait_for_dequeue] elapsed_time(us): " << get_now_microsecond() - start_time;
      start_time = get_now_microsecond();

      if (clouds_compensated_ == nullptr) {
        clouds_compensated_ = std::make_shared<PointClouds2>();
#ifdef WITH_ROS2
        clouds_compensated_->header.frame_id = config_.frame_id();
        clouds_compensated_->clouds.resize(clouds.size());
        for (size_t i = 0; i < clouds.size(); ++i) {
          clouds_compensated_->clouds[i].data.reserve(clouds[i]->cloud_msg_->data.capacity());
        }
#else
        clouds_compensated_->mutable_header()->set_frame_id(config_.frame_id());
        for (size_t i = 0; i < clouds.size(); ++i) {
          clouds_compensated_->add_clouds()->mutable_data()->reserve(
                                        clouds[i]->proto_cloud_->data().capacity());
        }
#endif
      }

      if (!compensator_->motion_compensation(clouds, clouds_compensated_, compensation_info)) {
        LOG(ERROR) << "[" << get_thread_name() << "] "
                   << "failed to motion compensation";
        continue;
      }

      LOG(INFO) << "[" << get_thread_name() << "] [TIMER] "
            << "[lidar_compensation] elapsed_time(us): " << get_now_microsecond() - start_time;
      start_time = get_now_microsecond();

      remove_clouds_point(clouds_compensated_);

      LOG(INFO) << "[" << get_thread_name() << "] [TIMER] "
            << "[remove_clouds_point] elapsed_time(us): " << get_now_microsecond() - start_time;

      start_time = get_now_microsecond();
#ifdef WITH_ROS2
      clouds_compensated_->header.stamp.sec = compensation_info->end_utime_ / 1000000;
      clouds_compensated_->header.stamp.nanosec = compensation_info->end_utime_ % 1000000;
      common::Singleton<LidarROSOutput>::get()->write_fusion_clouds(
                                        config_.channel_name(), clouds_compensated_);
#else
      static uint32_t seq = 0;
      clouds_compensated_->mutable_header()->set_sequence_num(seq++);
      clouds_compensated_->mutable_header()->set_lidar_timestamp(compensation_info->end_utime_);
      common::Singleton<LidarCyberOutput>::get()->write_fusion_clouds(
                                        config_.channel_name(), clouds_compensated_);
#endif
      LOG(INFO) << "[" << get_thread_name() << "] [TIMER] "
            << "[send_fusion_cloud] elapsed_time(us): " << get_now_microsecond() - start_time;

      LOG(INFO) << "[" << get_thread_name() << "] [TIMER] "
            << "[lidar_total] elapsed_time(us): "
            << get_now_microsecond() - compensation_info->end_utime_;
    }
  }
}
}  // namespace airi
}  // namespace crdc
