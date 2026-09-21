#include "ap1/planning/frames.hpp"
#include "ap1/planning/math_utils.hpp"

#include "geometry_msgs/msg/point.hpp"

#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"

using geometry_msgs::msg::Point;

using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::SpeedProfileStamped;

void ap1::planning::frames::unwrap_route_f(const RouteF& route, TargetPathStamped &path, SpeedProfileStamped& speed_profile) {
    // ROUTE WAYPOINTS
    path.path.clear();
    for (const ap1::math::vec2f& waypoint : route.route) {
        Point p;
        p.x = waypoint.x;
        p.y = waypoint.y;
        path.path.push_back(p);
    }

    // SPEED PROFILE
    speed_profile.speeds.clear();
    for (float speed : route.speed_profile) {
        speed_profile.speeds.push_back(speed);
    }
}
