#include <find_edge/TemplateAlignment.h>
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

using namespace std;

TemplateAlignment::TemplateAlignment()
    : min_sample_distance_(0.01f), max_correspondence_distance_(0.01f * 0.01f),
      nr_iterations_(500) {
  sac_ia_.setMinSampleDistance(min_sample_distance_);
  sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
  sac_ia_.setMaximumIterations(nr_iterations_);
}

TemplateAlignment::~TemplateAlignment(){};

void TemplateAlignment::setTargetCloud(FeatureCloud &target_cloud) {
  target_ = target_cloud;
  sac_ia_.setInputTarget(target_cloud.getPointCloud());
  sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
}

void TemplateAlignment::addTemplateCloud(FeatureCloud &template_cloud) {
  templates_.push_back(template_cloud);
}

void TemplateAlignment::align(FeatureCloud &template_cloud,
                              TemplateAlignment::Result &result) {
  sac_ia_.setInputSource(template_cloud.getPointCloud());
  sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

  pcl::PointCloud<pcl::PointXYZ> registration_output;
  sac_ia_.align(registration_output);

  result.fitness_score =
      (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
  result.final_transformation = sac_ia_.getFinalTransformation();
}

void TemplateAlignment::alignAll(
    std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result>>
        &results) {
  results.resize(templates_.size());
  for (std::size_t i = 0; i < templates_.size(); ++i) {
    align(templates_[i], results[i]);
  }
}

int TemplateAlignment::findBestAlignment(TemplateAlignment::Result &result) {
  vector<Result, Eigen::aligned_allocator<Result>> results;
  alignAll(results);
  float lowest_score = std::numeric_limits<float>::infinity();
  int best_template = 0;
  for (std::size_t i = 0; i < results.size(); ++i) {
    const Result &r = results[i];
    if (r.fitness_score < lowest_score) {
      lowest_score = r.fitness_score;
      best_template = (int)i;
    }
  }
  result = results[best_template];
  return (best_template);
}
