#include <find_edge/FeatureCloud.h>
#include <fstream>
#include <limits>
#include <Eigen/Core>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;

FeatureCloud::FeatureCloud()
    : search_method_xyz_(new SearchMethod), normal_radius_(0.12f),
      feature_radius_(0.12f) {}

FeatureCloud::~FeatureCloud() {}

void FeatureCloud::setInputCloud(PointCloud::Ptr xyz) {
  xyz_ = xyz;
  processInput();
}

void FeatureCloud::loadInputCloud(const std::string &pcd_file) {
  xyz_ = PointCloud::Ptr(new PointCloud);
  pcl::io::loadPCDFile(pcd_file, *xyz_);
  processInput();
}

FeatureCloud::PointCloud::Ptr FeatureCloud::getPointCloud() const { return (xyz_); }

FeatureCloud::SurfaceNormals::Ptr FeatureCloud::getSurfaceNormals() const {
  return (normals_);
}

FeatureCloud::LocalFeatures::Ptr FeatureCloud::getLocalFeatures() const {
  return (features_);
}

void FeatureCloud::processInput() {
  computeSurfaceNormals();
  computeLocalFeatures();
}

void FeatureCloud::computeSurfaceNormals() {
  normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setInputCloud(xyz_);
  norm_est.setSearchMethod(search_method_xyz_);
  norm_est.setRadiusSearch(normal_radius_);
  norm_est.compute(*normals_);
}

void FeatureCloud::computeLocalFeatures() {
  features_ = LocalFeatures::Ptr(new LocalFeatures);

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>
      fpfh_est;
  fpfh_est.setInputCloud(xyz_);
  fpfh_est.setInputNormals(normals_);
  fpfh_est.setSearchMethod(search_method_xyz_);
  fpfh_est.setRadiusSearch(feature_radius_);
  fpfh_est.compute(*features_);
}
