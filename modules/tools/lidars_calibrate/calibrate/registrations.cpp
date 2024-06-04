/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#include "include/registrations.h"
#include <iostream>
#include "include/pclomp/ndt_omp.h"
namespace crdc {
namespace airi {

template<typename T>
boost::shared_ptr<pcl::Registration<T, T>> Registration<T>::ndt_omp(int nbThread, int maxIteration,
                                                float resolution, float stepSize, float epsilon) {
  boost::shared_ptr<pclomp::NormalDistributionsTransform<T, T>>
          ndt(new pclomp::NormalDistributionsTransform<T, T>());
  ndt->setNumThreads(nbThread);
  ndt->setMaximumIterations(maxIteration);
  ndt->setResolution(resolution);
  ndt->setStepSize(stepSize);
  ndt->setTransformationEpsilon(epsilon);
  ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  return ndt;
}

template class Registration<pcl::PointXYZ>;
template class Registration<pcl::PointXYZI>;

}  // namespace airi
}  // namespace crdc
