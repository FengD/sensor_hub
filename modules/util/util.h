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

void calc_time_diff(const struct timespec *lo, const struct timespec *hi, struct timespec *diff);
void split(const std::string &s, const std::string &seperator, std::vector<std::string> *result);
void getAllFilesInFolder(const std::string& dir_in, std::vector<std::string> *files);
