// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: This module is an util module,
//              which contains many useful functions
// Date: 2019-01-01
// Change Log:

#include "util/util.h"
#include <string.h>
#include <time.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>

// time calculation
void calc_time_diff(const struct timespec *lo, const struct timespec *hi, struct timespec *diff) {
  diff->tv_sec = hi->tv_sec - lo->tv_sec;
  diff->tv_nsec = hi->tv_nsec - lo->tv_nsec;
  if (diff->tv_nsec < 0) {
    diff->tv_sec--;
    diff->tv_nsec += 1000000000;
  }
}

typedef std::string::size_type string_size;

void split(const std::string &s, const std::string &seperator, std::vector<std::string> *result) {
  result->clear();

  string_size i = 0;
  while (i != s.size()) {
    int32_t flag = 0;
    while (i != s.size() && flag == 0) {
      flag = 1;
      for (string_size x = 0; x < seperator.size(); ++x) {
        if (s[i] == seperator[x]) {
          ++i;
          flag = 0;
          break;
        }
      }
    }

    flag = 0;
    string_size j = i;
    while (j != s.size() && flag == 0) {
      for (string_size x = 0; x < seperator.size(); ++x) {
        if (s[j] == seperator[x]) {
          flag = 1;
          break;
        }
      }

      if (flag == 0) {
        ++j;
      }
    }
    if (i != j) {
      result->push_back(s.substr(i, j - i));
      i = j;
    }
  }
}

void getAllFilesInFolder(const std::string& dir_in, std::vector<std::string> *files) {
  if (dir_in.empty()) {
    return;
  }
  struct stat s;
  stat(dir_in.c_str(), &s);
  if (!S_ISDIR(s.st_mode)) {
    return;
  }
  DIR* open_dir = opendir(dir_in.c_str());
  if (NULL == open_dir) {
    std::exit(EXIT_FAILURE);
  }
  dirent* p = nullptr;
  while ((p = readdir(open_dir)) != nullptr) {
    struct stat st;
    if (p->d_name[0] != '.') {
      std::string name = dir_in + std::string("/") + std::string(p->d_name);
      stat(name.c_str(), &st);
      if (S_ISDIR(st.st_mode)) {
        getAllFilesInFolder(name, files);
      } else if (S_ISREG(st.st_mode)) {
        files->push_back(name);
      }
    }
  }
  closedir(open_dir);
}
