// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: error code

#pragma once

enum DeviceStatus {
  GET_RAW_PACKET_ERROR = -8,
  POLL_INPUT_ERROR = -7,
  POLL_TIMEOUT = -6,
  POLL_FAILED_ERROR = -5,
  NON_BLOCK_ERROR = -4,
  BIND_SOCKET_ERROR = -3,
  CREATE_SOCKET_ERROR = -2,
  CAMERA_TIMEOUT = -1,
  NO_ERROR = 0,
  SUCCESS = 1
};
