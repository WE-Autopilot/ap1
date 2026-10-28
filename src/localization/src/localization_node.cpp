// Sample NDT localization node.
//
// Loads a prebuilt point-cloud map (.pcd), waits for an initial pose, then
// matches every lidar scan against the map to estimate base_link in map.
//
//   in : points       sensor_msgs/PointCloud2               (lidar scan)
//        initialpose  geometry_msgs/PoseWithCovarianceStamped (RViz "2D Pose Estimate")
//   out: ndt_pose     geometry_msgs/PoseStamped
//        aligned_points / map_points  sensor_msgs/PointCloud2 (for RViz)
//        TF map -> base_link (if publish_tf)
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "ap1_localization/ndt_matcher.hpp"

namespace ap1_localization
{

namespace
{

Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  const Eigen::Quaternionf q(
    static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  m.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  m(0, 3) = static_cast<float>(pose.position.x);
  m(1, 3) = static_cast<float>(pose.position.y);
  m(2, 3) = static_cast<float>(pose.position.z);
  return m;
}

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f & m)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = m(0, 3);
  pose.position.y = m(1, 3);
  pose.position.z = m(2, 3);
  const Eigen::Quaternionf q(Eigen::Matrix3f(m.block<3, 3>(0, 0)));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

Eigen::Matrix4f transformToMatrix(const geometry_msgs::msg::Transform & t)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = t.translation.x;
  pose.position.y = t.translation.y;
  pose.position.z = t.translation.z;
  pose.orientation = t.rotation;
  return poseToMatrix(pose);
}

}  // namespace

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : Node("ndt_localization"),
    matcher_(loadParams()),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(*this)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    min_nvtl_ = declare_parameter<double>("min_nvtl", 2.3);
    const auto map_path = declare_parameter<std::string>("map_path", "");
    // Optional [x, y, z, yaw] so it can start without RViz. Empty = wait for initialpose.
    const auto initial_pose = declare_parameter<std::vector<double>>(
      "initial_pose", std::vector<double>{});

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("aligned_points", 10);
    // Transient local so RViz receives the map even if it starts after this node.
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_points", rclcpp::QoS(1).transient_local());

    loadMap(map_path);

    if (initial_pose.size() == 4) {
      const Eigen::AngleAxisf yaw(static_cast<float>(initial_pose[3]), Eigen::Vector3f::UnitZ());
      Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
      m.block<3, 3>(0, 0) = yaw.toRotationMatrix();
      m(0, 3) = static_cast<float>(initial_pose[0]);
      m(1, 3) = static_cast<float>(initial_pose[1]);
      m(2, 3) = static_cast<float>(initial_pose[2]);
      setInitialPose(m);
    } else if (!initial_pose.empty()) {
      RCLCPP_WARN(get_logger(), "initial_pose must be [x, y, z, yaw]; ignoring it");
    }

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        setInitialPose(poseToMatrix(msg->pose.pose));
      });
    // Sensor QoS with depth 1: if a match is slower than the lidar rate,
    // stale scans are dropped instead of queuing up latency.
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onScan(msg);});
  }

private:
  NdtParams loadParams()
  {
    NdtParams p;
    p.resolution = declare_parameter<double>("ndt.resolution", p.resolution);
    p.step_size = declare_parameter<double>("ndt.step_size", p.step_size);
    p.trans_epsilon = declare_parameter<double>("ndt.trans_epsilon", p.trans_epsilon);
    p.max_iterations = declare_parameter<int>("ndt.max_iterations", p.max_iterations);
    p.num_threads = declare_parameter<int>("ndt.num_threads", p.num_threads);
    p.scan_leaf_size = declare_parameter<double>("scan.leaf_size", p.scan_leaf_size);
    p.min_range = declare_parameter<double>("scan.min_range", p.min_range);
    p.max_range = declare_parameter<double>("scan.max_range", p.max_range);
    return p;
  }

  void loadMap(const std::string & path)
  {
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter map_path is empty; nothing to localize against");
      return;
    }
    auto map = std::make_shared<NdtMatcher::Cloud>();
    if (pcl::io::loadPCDFile(path, *map) != 0 || map->empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to load map '%s'", path.c_str());
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    matcher_.setMap(map);
    const auto ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(get_logger(), "Loaded map '%s' (%zu points, NDT grid built in %.0f ms)",
      path.c_str(), map->size(), ms);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    map_pub_->publish(msg);
  }

  void setInitialPose(const Eigen::Matrix4f & pose)
  {
    pose_ = pose;
    prev_pose_ = pose;  // zero velocity until two scans have been matched
    initialized_ = true;
    first_match_ = true;
    RCLCPP_INFO(get_logger(), "Initial pose set to (%.2f, %.2f, %.2f)",
      pose(0, 3), pose(1, 3), pose(2, 3));
  }

  void onScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    if (!matcher_.hasMap()) {
      return;
    }
    if (!initialized_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for an initial pose on 'initialpose' (RViz: 2D Pose Estimate)");
      return;
    }

    // NDT estimates base_link, so express the scan in base_link first.
    NdtMatcher::Cloud raw;
    pcl::fromROSMsg(*msg, raw);
    if (msg->header.frame_id != base_frame_) {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          base_frame_, msg->header.frame_id, tf2::TimePointZero);
        pcl::transformPointCloud(raw, raw, transformToMatrix(tf.transform));
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No TF %s -> %s: %s", msg->header.frame_id.c_str(), base_frame_.c_str(), e.what());
        return;
      }
    }
    auto scan = matcher_.preprocess(raw);
    if (scan->size() < 50) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Only %zu points after filtering; skipping scan", scan->size());
      return;
    }

    // Constant-velocity guess: assume the car moves as much as it did last scan.
    const Eigen::Matrix4f delta = prev_pose_.inverse() * pose_;
    const Eigen::Matrix4f guess = pose_ * delta;

    const auto t0 = std::chrono::steady_clock::now();
    NdtMatcher::Cloud aligned;
    const auto result = matcher_.align(scan, guess, aligned);
    const auto ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    // Same acceptance test as Autoware's default (converged_param_type: NVTL).
    const double nvtl = result.nearest_voxel_transformation_likelihood;
    if (nvtl < min_nvtl_) {
      // Keep the old pose rather than jump to a bad match.
      RCLCPP_WARN(get_logger(),
        "NDT rejected: NVTL %.3f < %.3f (TP=%.3f iters=%d, %.0f ms)",
        nvtl, min_nvtl_, result.transform_probability, result.iterations, ms);
      return;
    }
    if (result.hit_max_iterations) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "NDT used all %d iterations; the initial guess may be far off", result.iterations);
    }
    // The first match after an initial pose is mostly correcting that pose's
    // error, not real motion, so it must not seed the velocity estimate.
    prev_pose_ = first_match_ ? result.pose : pose_;
    pose_ = result.pose;
    first_match_ = false;
    RCLCPP_DEBUG(get_logger(), "NDT ok: NVTL=%.3f TP=%.3f iters=%d, %.0f ms",
      nvtl, result.transform_probability, result.iterations, ms);

    publish(msg->header.stamp, aligned);
  }

  void publish(const builtin_interfaces::msg::Time & stamp, const NdtMatcher::Cloud & aligned)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = matrixToPose(pose_);
    pose_pub_->publish(pose_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = pose_msg.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = pose_msg.pose.position.x;
      tf.transform.translation.y = pose_msg.pose.position.y;
      tf.transform.translation.z = pose_msg.pose.position.z;
      tf.transform.rotation = pose_msg.pose.orientation;
      tf_broadcaster_.sendTransform(tf);
    }

    if (aligned_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 cloud;
      pcl::toROSMsg(aligned, cloud);
      cloud.header = pose_msg.header;
      aligned_pub_->publish(cloud);
    }
  }

  NdtMatcher matcher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string map_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  double min_nvtl_ = 2.3;

  bool initialized_ = false;
  bool first_match_ = true;
  Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev_pose_ = Eigen::Matrix4f::Identity();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
};

}  // namespace ap1_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ap1_localization::LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
