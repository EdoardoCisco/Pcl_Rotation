#ifndef __FEATURECLOUD_H__
#define __FEATURECLOUD_H__

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
using namespace pcl;

class FeatureCloud {

protected:
  void processInput(); // chiama funzioni sotto
  void computeSurfaceNormals(); // per trovare la normale della superfice e
  void computeLocalFeatures(); // per computare/settare vari tipi presenti nella classe

public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // Cloud che viene caricato
  typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
  typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;
  FeatureCloud();
  ~FeatureCloud();
  void setInputCloud(PointCloud::Ptr xyz); // Per settate Cloud del brick o .pcd
  void loadInputCloud(const string &pcd_file); // per caricare i file di confronto usato da StoreClass [pcdFileStore.h]
  PointCloud::Ptr getPointCloud() const;
  SurfaceNormals::Ptr getSurfaceNormals() const;
  LocalFeatures::Ptr getLocalFeatures() const;
  
  private:
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;
    float normal_radius_;
    float feature_radius_;

};

#endif
