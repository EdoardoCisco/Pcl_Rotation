#ifndef __TEMPLATEALIGNMENT_H__
#define __TEMPLATEALIGNMENT_H__

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
#include <vector>
#include <find_edge/FeatureCloud.h>

using namespace std;
using namespace pcl;

class FeatureCloud;

class TemplateAlignment {
private:
  vector<FeatureCloud> templates_; // vettore di .pcd
  FeatureCloud target_; // cloud del brick
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,
                                       pcl::FPFHSignature33>
      sac_ia_;
  float min_sample_distance_;
  float max_correspondence_distance_;
  int nr_iterations_;

public:
  struct Result {
    float fitness_score; // quanto e' simile la rotazione se blocco esattamente dritto allora fit 0
    Eigen::Matrix4f final_transformation; // matrice di rotazione del brick rispetto a un .pcd
    PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment();
  ~TemplateAlignment();
  void setTargetCloud(FeatureCloud &target_cloud); // imposto il cloud del Brick
  void addTemplateCloud(FeatureCloud &template_cloud); // imposto il vettore per il Brick corrispondente
  void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result); // chiamata da alignAll
  void alignAll(vector<TemplateAlignment::Result,
                       Eigen::aligned_allocator<Result>> &results); // chiamata da find best computa rotazioni
  int findBestAlignment(TemplateAlignment::Result &result); // ritorna indice a quale .pcd sono le rotazioni
};

#endif
