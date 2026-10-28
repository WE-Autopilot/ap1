#include "ap1_localization/ndt_matcher.hpp"

#include <pcl/filters/voxel_grid.h>

namespace ap1_localization
{

NdtMatcher::NdtMatcher(const NdtParams & params)
: params_(params)
{
  pclomp::NdtParams ndt_params;
  ndt_params.trans_epsilon = params_.trans_epsilon;
  ndt_params.step_size = params_.step_size;
  ndt_params.resolution = static_cast<float>(params_.resolution);
  ndt_params.max_iterations = params_.max_iterations;
  ndt_params.num_threads = std::max(params_.num_threads, 1);
  ndt_params.search_method = pclomp::KDTREE;  // Autoware's default
  ndt_.setParams(ndt_params);
}

void NdtMatcher::setMap(const Cloud::Ptr & map)
{
  ndt_.setInputTarget(map);
  ndt_.createVoxelKdtree();
}

NdtMatcher::Cloud::Ptr NdtMatcher::preprocess(const Cloud & scan) const
{
  auto cropped = std::make_shared<Cloud>();
  cropped->reserve(scan.size());
  const double min_sq = params_.min_range * params_.min_range;
  const double max_sq = params_.max_range * params_.max_range;
  for (const auto & p : scan) {
    const double d_sq = p.x * p.x + p.y * p.y + p.z * p.z;
    if (d_sq >= min_sq && d_sq <= max_sq) {
      cropped->push_back(p);
    }
  }

  auto filtered = std::make_shared<Cloud>();
  pcl::VoxelGrid<Point> voxel;
  const auto leaf = static_cast<float>(params_.scan_leaf_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.setInputCloud(cropped);
  voxel.filter(*filtered);
  return filtered;
}

NdtResult NdtMatcher::align(const Cloud::Ptr & scan, const Eigen::Matrix4f & guess, Cloud & aligned)
{
  ndt_.align(aligned, guess, scan);
  const auto r = ndt_.getResult();

  NdtResult result;
  result.pose = r.pose;
  result.iterations = r.iteration_num;
  result.hit_max_iterations = r.iteration_num >= params_.max_iterations;
  result.transform_probability = r.transform_probability;
  result.nearest_voxel_transformation_likelihood = r.nearest_voxel_transformation_likelihood;
  return result;
}

}  // namespace ap1_localization
