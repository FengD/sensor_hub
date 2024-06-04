/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#ifndef MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_REGISTRATIONS_H_
#define MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_REGISTRATIONS_H_

#include <pcl/registration/registration.h>

namespace crdc {
namespace airi {

template<typename T>
class Registration {
 public:
  static boost::shared_ptr<pcl::Registration<T, T>> ndt_omp(int nbThread, int maxIteration,
                                          float resolution, float stepSize, float epsilon);
};
}  // namespace airi
}  // namespace crdc

#endif  // MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_REGISTRATIONS_H_
