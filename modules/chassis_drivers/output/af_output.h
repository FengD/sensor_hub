/**
 * @file af_output.h
 * @author yuanyuan.wang3 (yuanyuan.wang3@hirain.com)
 * @brief af output
 * @date 2023-04-26
 * @license Modified BSD Software License Agreement
 * @copyright Copyright (C) 2023 Hirain Technologies Inc.
 * 
 */

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msg/msg/packets.hpp"
#include "sensor_msg/msg/ins.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msg/msg/srr.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msg/msg/crane_info.hpp"

namespace crdc {
namespace airi {

class ChassisAFOutput {
 public:
  ChassisAFOutput() = default;
  virtual ~ChassisAFOutput() = default;

  /**
   * @brief Init the af output
   * @param the name of the node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[CHASSIS_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[CHASSIS_OUTPUT] Node name: " << node_name;
    node_ = std::make_shared<rclcpp::Node>(node_name);
    ins_data_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::Ins>(node_));
    imu_data_writer_ptr_.reset(new AFChannelWriter<sensor_msgs::msg::Imu>(node_));
    srr_data_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::Srr>(node_));
    vehicle_data_writer_ptr_.reset(new AFChannelWriter<geometry_msgs::msg::TwistStamped>(node_));
    mbstatus_data_writer_ptr_.reset(new AFChannelWriter<geometry_msgs::msg::PoseStamped>(node_));
    vhimu_data_writer_ptr_.reset(new AFChannelWriter<geometry_msgs::msg::PoseStamped>(node_));
    vhebs_data_writer_ptr_.reset(new AFChannelWriter<geometry_msgs::msg::PoseStamped>(node_));
    vhquay_data_writer_ptr_.reset(new AFChannelWriter<geometry_msgs::msg::PoseStamped>(node_));
    crane_data_writer_ptr_.reset(new AFChannelWriter<sensor_msg::msg::CraneInfo>(node_));
    campose_data_writer_ptr_.reset(new AFChannelWriter<geometry_msgs::msg::PoseStamped>(node_));
    return true;
  }

  /**
   * @brief Send the front camera pose data message by topic
   * @param topic name
   * @param proto fcampose data ptr
   * @return status
   */
  bool write_campose_data(const std::string& topic,
      const std::shared_ptr<geometry_msgs::msg::PoseStamped>& campose_msg) {
    campose_msg->header.frame_id = "ChassisDriver";
    return campose_data_writer_ptr_->write(topic, campose_msg);
  }

  /**
   * @brief Send the mbstatus data message by topic
   * @param topic name
   * @param proto   mbstatus data ptr
   * @return status
   */
  bool write_mbstatus_data(const std::string& topic,
      const std::shared_ptr<geometry_msgs::msg::PoseStamped>& mbstatus_msg) {
    mbstatus_msg->header.frame_id = "ChassisDriver";
    return mbstatus_data_writer_ptr_->write(topic, mbstatus_msg);
  }

  /**
   * @brief Send the vhimu data message by topic
   * @param topic name
   * @param proto   vhimu data ptr
   * @return status
   */
  bool write_vhimu_data(const std::string& topic,
      const std::shared_ptr<geometry_msgs::msg::PoseStamped>& vhimu_msg) {
    vhimu_msg->header.frame_id = "ChassisDriver";
    return vhimu_data_writer_ptr_->write(topic, vhimu_msg);
  }

  /**
   * @brief Send the vhebs data message by topic
   * @param topic name
   * @param proto   vhebs data ptr
   * @return status
   */
  bool write_vhebs_data(const std::string& topic,
      const std::shared_ptr<geometry_msgs::msg::PoseStamped>& vhebs_msg) {
    vhebs_msg->header.frame_id = "ChassisDriver";
    return vhebs_data_writer_ptr_->write(topic, vhebs_msg);
  }

  /**
   * @brief Send the vhquay data message by topic
   * @param topic name
   * @param proto   vhquay data ptr
   * @return status
   */
  bool write_vhquay_data(const std::string& topic,
      const std::shared_ptr<geometry_msgs::msg::PoseStamped>& vhquay_msg) {
    vhquay_msg->header.frame_id = "ChassisDriver";
    return vhquay_data_writer_ptr_->write(topic, vhquay_msg);
  }

  /**
   * @brief Send the crane data message by topic
   * @param topic name
   * @param proto   crane data ptr
   * @return status
   */
  bool write_crane_data(const std::string& topic,
      const std::shared_ptr<sensor_msg::msg::CraneInfo>& crane_msg) {
    crane_msg->header.frame_id = "ChassisDriver";
    return crane_data_writer_ptr_->write(topic, crane_msg);
  }

  /**
   * @brief Send the ins data message by topic
   * @param topic name
   * @param proto ins data ptr
   * @return status
   */
  bool write_ins_data(const std::string& topic,
      const std::shared_ptr<sensor_msg::msg::Ins>& ins_msg) {
    ins_msg->header.frame_id = "ChassisDriver";
    ins_msg->header.stamp.sec = get_now_microsecond() / 1000000;
    ins_msg->header.stamp.nanosec = get_now_microsecond() % 1000000 * 1000;
    return ins_data_writer_ptr_->write(topic, ins_msg);
  }

  /**
   * @brief Send the imu data message by topic
   * @param topic name
   * @param proto imu data ptr
   * @return status 
   */
  bool write_imu_data(const std::string& topic,
      const std::shared_ptr<sensor_msgs::msg::Imu>& imu_msg) {
    imu_msg->header.frame_id = "ChassisDriver";
    imu_msg->header.stamp.sec = get_now_microsecond() / 1000000;
    imu_msg->header.stamp.nanosec = get_now_microsecond() % 1000000 * 1000;
    return imu_data_writer_ptr_->write(topic, imu_msg);
  }

  /**
   * @brief Send the srr data message by topic
   * @param topic name
   * @param proto srr data ptr
   * @return status 
   */
  bool write_srr_data(const std::string& topic,
      const std::shared_ptr<sensor_msg::msg::Srr>& srr_msg) {
    srr_msg->header.frame_id = "ChassisDriver";
    srr_msg->header.stamp.sec = get_now_microsecond() / 1000000;
    srr_msg->header.stamp.nanosec = get_now_microsecond() % 1000000 * 1000;
    return srr_data_writer_ptr_->write(topic, srr_msg);
  }

  /**
   * @brief Send the vehicle data message by topic
   * @param topic name
   * @param proto vehicle data ptr
   * @return status 
   */
  bool write_vehicle_data(const std::string& topic,
      const std::shared_ptr<geometry_msgs::msg::TwistStamped>& vehicle_msg) {
    vehicle_msg->header.frame_id = "ChassisDriver";
    return vehicle_data_writer_ptr_->write(topic, vehicle_msg);
  }

  /**
   * @brief Send the packets message by topic
   * @param topic name
   * @param proto packets ptr
   * @return status
   */
  bool write_packet(const std::string& topic,
      const std::shared_ptr<sensor_msg::msg::Packets>& packets_msg) {
    return packets_writer_ptr_->write(topic, packets_msg);
  }

 private:
  friend class common::Singleton<ChassisAFOutput>;
  template <typename MessageT>
  class AFChannelWriter {
   public:
    explicit AFChannelWriter(std::shared_ptr<rclcpp::Node>& node) : node_(node) {}

    bool write(const std::string& topic, const std::shared_ptr<MessageT>& msg_ptr) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (writer_.find(topic) == writer_.end()) {
        writer_[topic] = node_->create_publisher<MessageT>(topic, rclcpp::QoS(10));
      }
      writer_[topic]->publish(*msg_ptr);
      return true;
    }

   private:
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Publisher<MessageT>>> writer_;
    std::shared_ptr<rclcpp::Node> node_;
    std::mutex mutex_;
  };

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::Ins>> ins_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msgs::msg::Imu>> imu_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::Srr>> srr_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<geometry_msgs::msg::TwistStamped>> vehicle_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::Packets>> packets_writer_ptr_;
  std::shared_ptr<AFChannelWriter<geometry_msgs::msg::PoseStamped>> mbstatus_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<geometry_msgs::msg::PoseStamped>> vhimu_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<geometry_msgs::msg::PoseStamped>> vhebs_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<geometry_msgs::msg::PoseStamped>> vhquay_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<sensor_msg::msg::CraneInfo>> crane_data_writer_ptr_;
  std::shared_ptr<AFChannelWriter<geometry_msgs::msg::PoseStamped>> campose_data_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc
