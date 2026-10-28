// NDT scan-to-map matcher backed by Autoware's multigrid NDT
// (pclomp::MultiGridNormalDistributionsTransform from autoware_ndt_scan_matcher,
// the same algorithm the Autoware ndt_scan_matcher node runs, multithreaded
// with OpenMP). No ROS here, so it can be unit tested on its own.
#pragma once

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ap1_localization
{

// Defaults match autoware_ndt_scan_matcher's config/ndt_scan_matcher.param.yaml.
struct NdtParams
{
  // NDT voxel size of the map (m). Larger = wider convergence basin, less precise.
  double resolution = 2.0;
  // Max step of the Newton update (m).
  double step_size = 0.1;
  // Stop iterating once the transform changes less than this.
  double trans_epsilon = 0.01;
  int max_iterations = 30;
  int num_threads = 4;
  // Scan preprocessing: voxel downsampling and range crop (m).
  double scan_leaf_size = 1.0;
  double min_range = 2.0;   // drops returns from the ego vehicle itself
  double max_range = 80.0;
};

struct NdtResult
{
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();  // base_link in map
  int iterations = 0;
  bool hit_max_iterations = false;  // usually means it did not settle
  // Autoware's two match-quality scores; higher is better for both.
  // Autoware accepts a match by default when NVTL >= 2.3.
  double transform_probability = 0.0;
  double nearest_voxel_transformation_likelihood = 0.0;
};

class NdtMatcher
{
public:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  explicit NdtMatcher(const NdtParams & params);

  // Builds the NDT voxel grid. Expensive, so call once per map.
  void setMap(const Cloud::Ptr & map);
  bool hasMap() const { return ndt_.hasTarget(); }

  // Range crop + voxel downsample. Input must already be in the base frame.
  Cloud::Ptr preprocess(const Cloud & scan) const;

  // Aligns a preprocessed scan to the map starting from `guess`.
  // `aligned` receives the scan transformed by the resulting pose.
  NdtResult align(const Cloud::Ptr & scan, const Eigen::Matrix4f & guess, Cloud & aligned);

private:
  NdtParams params_;
  pclomp::MultiGridNormalDistributionsTransform<Point, Point> ndt_;
};

}  // namespace ap1_localization
