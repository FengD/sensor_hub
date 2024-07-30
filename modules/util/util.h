// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: This module is an util module,
//              which contains many useful functions
// Date: 2019-01-01
// Change Log:

#pragma once

#include <sys/wait.h>
#include <stdint.h>
#ifdef WITH_TDA4
#include <utils/properties.h>
#include <glog/logging.h>
#endif
#include <string>
#include <vector>

#ifdef WITH_TDA4
constexpr char PRODUCT_PROPERTY[] = "ro.product";
constexpr char VEHICLE_NUMBER_PROPERTY[] = "persist.product.vehiclenum";

std::string get_product_name();
std::string get_vehicle_number();
void set_env(const std::string &key, const std::string &value);
#endif

void calc_time_diff(const struct timespec *lo, const struct timespec *hi, struct timespec *diff);
void split(const std::string &s, const std::string &seperator, std::vector<std::string> *result);
void getAllFilesInFolder(const std::string& dir_in, std::vector<std::string> *files);
