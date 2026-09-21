/**
 * Definition for the Route and Map frames.
 * See documentation on frames for more details.
 */

#ifndef AP1_PLANNING_FRAMES_HPP
#define AP1_PLANNING_FRAMES_HPP

#include <vector>

#include "rclcpp/time.hpp"

#include "ap1/planning/math_utils.hpp"

#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"

namespace ap1::planning::frames {
struct RouteF
{
    std::vector<ap1::math::vec2f> route;
    std::vector<float> speed_profile;
};

struct MapF
{
    float speed;    // m/s
    float odometer; // distance (m)
    rclcpp::Time time;
    ap1_msgs::msg::LaneBoundaries lane;
    ap1_msgs::msg::EntityStateArray entities;
};

/**
 * Convert RouteF to TargetPathStamped
 */
void unwrap_route_f(const RouteF& route, ap1_msgs::msg::TargetPathStamped &path, ap1_msgs::msg::SpeedProfileStamped &speed_profile);
} // namespace ap1::planning::frames

#endif // AP1_PLANNING_FRAMES_HPP
