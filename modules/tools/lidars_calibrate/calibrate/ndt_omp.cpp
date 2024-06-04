/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#include "include/pclomp/ndt_omp.h"
#include "include/pclomp/ndt_omp_impl.hpp"
namespace crdc {
namespace airi {

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;

}  // namespace airi
}  // namespace crdc
