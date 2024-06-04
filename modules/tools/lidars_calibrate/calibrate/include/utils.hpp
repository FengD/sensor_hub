#ifndef MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_UTILS_HPP_
#define MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_UTILS_HPP_

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <assert.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <numeric>
#include <algorithm>

namespace crdc {
namespace airi {
typedef Eigen::Matrix<float, Eigen::Dynamic, 3> MatrixX3;

template <typename T>
static float sumX(const float& sum, const T& p) {
  return sum + p.x;
}

template <typename T>
static float sumY(const float& sum, const T& p) {
  return sum + p.y;
}

template <typename T>
static float sumXMultiplyY(const float& sum, const T& p) {
  return sum + p.x * p.y;
}

template <typename T>
static float sumXSquare(const float& sum, const T& p) {
  return sum + p.x * p.x;
}

template <typename T>
static void fitPlaneByRansac(const pcl::PointCloud<T>& input_cloud, const float& dist_th,
                             std::vector<int>& inliers, Eigen::VectorXf& coef) {
  typename pcl::SampleConsensusModelPlane<T>::Ptr model_plane(
      new pcl::SampleConsensusModelPlane<T>(input_cloud.makeShared()));
  pcl::RandomSampleConsensus<T> ransac_plane(model_plane);
  ransac_plane.setDistanceThreshold(dist_th);
  ransac_plane.computeModel();
  ransac_plane.getInliers(inliers);
  ransac_plane.getModelCoefficients(coef);
}
/**
 * @brief fit 3D plane
 * @tparam T point cloud type
 * @param cloud
 * @param plane_params [a,b,c,d] a*x+b*y+c*z+d=0
 */
template <typename T>
void fit3DPlane(T const& cloud, std::vector<float>& plane_params) {
  size_t cloud_size = cloud.size();
  if (cloud_size <= 5)
    return;

  plane_params.resize(4);
  MatrixX3 point_matrix;
  point_matrix.resize(cloud_size, 3);
  for (size_t i = 0; i < cloud_size; ++i) {
    point_matrix(i, 0) = cloud.points[i].x;
    point_matrix(i, 1) = cloud.points[i].y;
    point_matrix(i, 2) = cloud.points[i].z;
  }
  Eigen::Matrix<float, 1, 3> mean_matrix = point_matrix.colwise().mean();
  MatrixX3 mean_temp;
  mean_temp.setOnes(cloud_size, 3);
  mean_temp.col(0) = mean_temp.col(0) * mean_matrix(0, 0);
  mean_temp.col(1) = mean_temp.col(1) * mean_matrix(0, 1);
  mean_temp.col(2) = mean_temp.col(2) * mean_matrix(0, 2);
  point_matrix = point_matrix - mean_temp;
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(point_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto V_matrix = svd.matrixV();
  plane_params[0] = V_matrix(0, 2);
  plane_params[1] = V_matrix(1, 2);
  plane_params[2] = V_matrix(2, 2);
  plane_params[3] = -(plane_params[0] * mean_matrix(0, 0) + plane_params[1] * mean_matrix(0, 1) +
                      plane_params[2] * mean_matrix(0, 2));
}

template <typename T>
static void fitLineByRansac(const pcl::PointCloud<T>& input_cloud, const float& dist_th,
                            std::vector<int>& inliers, Eigen::VectorXf& coef) {
  typename pcl::SampleConsensusModelLine<T>::Ptr model_line(
      new pcl::SampleConsensusModelLine<T>(input_cloud.makeShared()));
  pcl::RandomSampleConsensus<T> ransac_line(model_line);
  ransac_line.setDistanceThreshold(dist_th);
  ransac_line.computeModel();
  ransac_line.getInliers(inliers);
  ransac_line.getModelCoefficients(coef);
}

// line equation: y = k * x + b
template <typename T>
static void fit2dLineByLeastSquare(const pcl::PointCloud<T>& input_cloud, float& k, float& b) {
  size_t cloud_size = input_cloud.size();
  if (cloud_size <= 5)
    return;

  float v1, v2, v3, v4;
  v1 = std::accumulate(input_cloud.begin(), input_cloud.end(), 0.0, sumX<T>);
  v2 = std::accumulate(input_cloud.begin(), input_cloud.end(), 0.0, sumY<T>);
  v3 = std::accumulate(input_cloud.begin(), input_cloud.end(), 0.0, sumXMultiplyY<T>);
  v4 = std::accumulate(input_cloud.begin(), input_cloud.end(), 0.0, sumXSquare<T>);
  float temp1 = input_cloud.size() * v4 - v1 * v1;
  float temp2 = input_cloud.size() * v3 - v1 * v2;
  if (fabs(temp1 - 0.0) > 0.0001) {
    k = temp2 / temp1;
    b = (v4 * v2 - v1 * v3) / temp1;
  } else {
    k = 0.0;
    b = 0.0;
  }
}

template <typename PointT>
static bool sortByAxisY(const PointT& pt1, const PointT& pt2) {
  return pt1.y > pt2.y;
}

template <typename T>
static void fit2dLineBy2Points(const pcl::PointCloud<T>& input_cloud, float& k, float& b) {
  size_t cloud_size = input_cloud.size();
  if (cloud_size <= 5)
    return;

  pcl::PointCloud<T> temp_cloud = input_cloud;
  std::sort(temp_cloud.points.begin(), temp_cloud.points.end(), sortByAxisY<T>);
  T pt1 = temp_cloud.points[0];
  T pt2 = temp_cloud.points[1];
  T last_pt = temp_cloud.points[temp_cloud.points.size() - 1];
  float delta_x = pt1.x - pt2.x;
  float delta_y = pt1.y - pt2.y;
  if (fabs(delta_x - 0.0) > 0.0001) {
    k = delta_y / delta_x;
    b = pt1.y - k * pt1.x;
  }
}

template <typename T>
static void cloudTransformer(const pcl::PointCloud<T>& input_cloud, const float& delta_x,
                             const float& delta_y, const float& delta_z, const float& roll,
                             const float& pitch, const float& yaw,
                             pcl::PointCloud<T>& output_cloud) {
  Eigen::Translation3f translation(delta_x, delta_y, delta_z);
  Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f tf_matrix = (translation * rot_z * rot_y * rot_x).matrix();
  pcl::transformPointCloud(input_cloud, output_cloud, tf_matrix);
}

template <typename T>
static void transformCloud(const pcl::PointCloud<T>& input_cloud,
                           const std::vector<float>& transform_params,
                           pcl::PointCloud<T>& output_cloud) {
  Eigen::Translation3f translation(transform_params[0], transform_params[1], transform_params[2]);
  Eigen::AngleAxisf rot_x(transform_params[3], Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y(transform_params[4], Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(transform_params[5], Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f tf_matrix = (translation * rot_z * rot_y * rot_x).matrix();
  pcl::transformPointCloud(input_cloud, output_cloud, tf_matrix);
}

static Eigen::Vector3f rotationMatrixToEulerAngles(const Eigen::Matrix3f& rot_mat) {
  float sy = sqrt(rot_mat(0, 0) * rot_mat(0, 0) + rot_mat(1, 0) * rot_mat(1, 0));
  bool singular = sy < 1e-6;
  Eigen::Vector3f rpy;
  if (!singular) {
    rpy(0, 0) = atan2(rot_mat(2, 1), rot_mat(2, 2));
    rpy(1, 0) = atan2(-rot_mat(2, 0), sy);
    rpy(2, 0) = atan2(rot_mat(1, 0), rot_mat(0, 0));
  } else {
    rpy(0, 0) = atan2(-rot_mat(1, 2), rot_mat(1, 1));
    rpy(1, 0) = atan2(-rot_mat(2, 0), sy);
    rpy(2, 0) = 0;
  }
  return rpy;
}

template <typename T>
static void addVector(std::vector<T>& vl, std::vector<T>& vr) {
  size_t size = vl.size();
  assert(size == vr.size());
  for (size_t i = 0; i < size; ++i) {
    vl[i] += vr[i];
  }
}

template <typename T>
static void initT(std::vector<T>& v) {
  std::vector<T> tmp_v(v.size(), 0);
  v.swap(tmp_v);
}

template <typename T>
static void divideVector(std::vector<T>& vl, std::vector<T>& vr, int n) {
  size_t size = vl.size();
  for (size_t i = 0; i < size; ++i) {
    vl[i] = vr[i] / n;
  }
}

}  // namespace airi
}  // namespace crdc

#endif  // MODULES_TOOLS_LIDARS_CALIBRATE_CALIBRATE_INCLUDE_UTILS_HPP_
