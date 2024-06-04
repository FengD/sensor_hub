/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2019 Feng DING, Hirain
 *
 *  License: Modified BSD Software License Agreement
 *
 */

#include "include/pclomp/voxel_grid_covariance_omp.h"
#include "include/pclomp/voxel_grid_covariance_omp_impl.hpp"
namespace crdc {
namespace airi {
template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;

}  // namespace airi
}  // namespace crdc
