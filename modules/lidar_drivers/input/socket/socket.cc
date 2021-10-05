// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input socket

#include "lidar_drivers/input/socket/socket.h"
#include "lidar_drivers/output/cyber_output.h"

namespace crdc {
namespace airi {

SocketInput::~SocketInput() {
  for (auto i = 0; i < socket_num_; ++i) {
    close(poll_fds_[i].fd);
  }
}

bool SocketInput::init(const LidarInputConfig& config) {
  config_ = config;

  if (!config_.has_socket_config()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] has no SocketInput config";
    return false;
  }

  if (!config_.has_lidar_port()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] has no lidar port";
    return false;
  }

  int32_t socket_lidar_fd = set_udp_socket(config_.lidar_port());
  if (socket_lidar_fd < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to set lidar socket, port "
               << config_.lidar_port();
    return false;
  }

  ports_[0] = config_.lidar_port();
  poll_fds_[0].fd = socket_lidar_fd;
  socket_num_ = 1;

  if (config_.has_gps_port()) {
    int32_t socket_gps_fd = set_udp_socket(config_.gps_port());
    if (socket_gps_fd < 0) {
      LOG(ERROR) << "[" << config_.frame_id() << "] failed to set gps socket, port "
                 << config_.gps_port();
      return false;
    }

    ports_[socket_num_] = config_.gps_port();
    poll_fds_[socket_num_].fd = socket_gps_fd;
    socket_num_++;
  }

  if (!init_pool()) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to init raw data pool.";
    return false;
  }

  return true;
}

int32_t SocketInput::set_udp_socket(const uint16_t& port) {
  int32_t sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to create socket.";
    return DeviceStatus::CREATE_SOCKET_ERROR;
  }

  struct sockaddr_in my_addr;
  memset(reinterpret_cast<char*>(&my_addr), 0, sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(port);
  my_addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock_fd, (struct sockaddr *) &my_addr, sizeof(sockaddr)) < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to bind " << port;
    return DeviceStatus::BIND_SOCKET_ERROR;
  }

  if (fcntl(sock_fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] non block port " << port;
    return DeviceStatus::NON_BLOCK_ERROR;
  }

  return sock_fd;
}

int32_t SocketInput::check_socket_status() {
  for (auto i = 0; i < socket_num_; ++i) {
    poll_fds_[i].events = POLLIN;
  }
  int32_t retval = poll(poll_fds_, 1, POLL_TIMEOUT);
  if (retval < 0) {
    if (errno != EINTR) {
      LOG(WARNING) << "[" << config_.frame_id() << "] poll failed, error: "
                   << strerror(errno);
    }

    return DeviceStatus::POLL_FAILED_ERROR;
  } else if (retval == 0) {
    LOG(WARNING) << "[" << config_.frame_id() << "] poll time_out.";
    return DeviceStatus::POLL_TIMEOUT;
  }

  if ((poll_fds_[0].revents & POLLERR) || (poll_fds_[0].revents & POLLHUP)
          || (poll_fds_[0].revents & POLLNVAL)) {
    LOG(ERROR) << "[" << config_.frame_id() << "] poll reports lidar input error.";
    return DeviceStatus::POLL_INPUT_ERROR;
  }
  return DeviceStatus::SUCCESS;
}

int32_t SocketInput::get_lidar_data(Packet** packet) {
  int32_t ret = check_socket_status();
  if (ret != DeviceStatus::SUCCESS) {
    return ret;
  }

  for (auto i = 0; i < socket_num_; ++i) {
    if (poll_fds_[i].events & POLLIN) {
      Packet *raw_packet = get_raw_packet();
      if (!raw_packet) {
        LOG(ERROR) << "[" << config_.frame_id() << "] failed to get raw packet.";
        return DeviceStatus::GET_RAW_PACKET_ERROR;
      }

      raw_packet->mutable_data()->resize(config_.packet_size());
      char *data_buf = const_cast<char*>(raw_packet->mutable_data()->data());
      ssize_t receive_size = recvfrom(poll_fds_[i].fd, reinterpret_cast<void *>(data_buf),
                                      config_.packet_size(), 0, nullptr, nullptr);
      if (receive_size < 0) {
        LOG(WARNING) << "[" << config_.frame_id() << "] recvfrom error.";
        packet_cur_index_--;
        continue;
      }
      raw_packet->mutable_data()->resize(receive_size);
      raw_packet->set_size(receive_size);
      raw_packet->set_port(ports_[i]);
      raw_packet->set_time_system(get_now_microsecond());
      *packet = raw_packet;
      return DeviceStatus::SUCCESS;
    }
  }

  return DeviceStatus::NO_ERROR;
}

}  // namespace airi
}  // namespace crdc
