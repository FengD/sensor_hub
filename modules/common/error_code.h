// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: error code

#pragma once

enum DeviceStatus {
  PCAP_END = -29,
  FAILED_GET_CYBER_MESSAGE = -28,
  NON_BLOCKERR = -27,
  UNKOWN_HOST_ERROR = -26,
  CONNECT_ERROR = -25,
  ERR_BIND = -24,
  ERR_INTERFACE = -23,
  ERR_IO = -22,
  ERR_CAN_OPEN = -21,
  ERR_CAN_CLOSE = -20,
  ERR_CAN_READ = -19,
  ERR_CAN_WRITE = -18,
  MSG_SEND_ERROR = -17,
  MSG_RECEIVE_ERROR = -16,
  DEVICE_ERROR = -15,
  TIMEOUT_ERROR = -14,
  POLL_ERROR = -13,
  SOCKET_SETOPT_ERROR = -12,
  SOCKET_BIND_ERROR = -11,
  SOCKET_OPEN_ERROR = -10,
  IP_EMPTY_ERROR = -9,
  GET_RAW_PACKET_ERROR = -8,
  POLL_INPUT_ERROR = -7,
  POLL_TIMEOUT = -6,
  POLL_FAILED_ERROR = -5,
  NON_BLOCK_ERROR = -4,
  BIND_SOCKET_ERROR = -3,
  CREATE_SOCKET_ERROR = -2,
  CAMERA_TIMEOUT = -1,
  NO_ERROR = 0,
  SUCCESS = 1,

  // camera data abnormal diagnose
  CAMERA_DATA_GREEN = -30,
  CAMERA_WINDOW_DIRTY_BLOCKED = -31,
  CAMERA_STRONG_BACKLIGHT = -32,
  CAMERA_IMAGE_DARK = -33,
  CAMERA_IMAGE_BLUR = -34,

  // lidar data abnormal diagnose
  LIDAR_PACKET_LOSS = -50
};
