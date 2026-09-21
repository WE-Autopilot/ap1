#include <cstddef>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "ap1/planning/waypoint_utils.hpp"

namespace ap1::planning {

int locate_closest_waypoint(
    const ap1::math::vec2f& target_location,
    const std::vector<ap1::math::vec2f>& navigable_waypoints
) {
    // Check if navigable_waypoints is empty
    if (navigable_waypoints.empty()) {
        return -1;
    }

    // Initialize with the first waypoint
    float min_distance = distance(target_location, navigable_waypoints[0]);
    size_t closest_index = 0;

    // Loop through all waypoints to find the closest one
    for (size_t i = 1; i < navigable_waypoints.size(); ++i) {
        float current_distance = distance(target_location, navigable_waypoints[i]);
        if (current_distance < min_distance) {
            min_distance = current_distance;
            closest_index = i;
        }
    }

    return closest_index;
}

std::vector<ap1::math::vec2f> generate_waypoint_sequence(
    const std::vector<ap1::math::vec2f>& waypoints,
    const int to,
    const std::vector<ap1::math::vec2f>& fallback_path
)
{
    // Check if waypoints is empty OR to == -1 (no valid target)
    if (waypoints.empty() || to == -1) {
        RCLCPP_WARN(rclcpp::get_logger("WAYPOINT_UTILS"), "Using fallback path due to empty waypoints or no valid target.");
        return fallback_path;
    }

    // Create a new vector to hold the sequence
    std::vector<ap1::math::vec2f> sequence;

    // Copy waypoints from index 0 to index 'to' (inclusive)
    // This creates the path from the car's position to the target
    for (int i = 0; i <= to && i < static_cast<int>(waypoints.size()); ++i) {
        sequence.push_back(waypoints[i]);
    }

    return sequence;
}

/**
 * @brief Finds the next waypoint in the path.
 * Does this by jumping forwards until the path has an x coord > 0 (bc of car-centered origin system
 * this is identical).
 *
 * @return size_t
 */
long find_next_waypoint_idx(const std::vector<ap1::math::vec2f>& centerline)
{
    // find the closest waypoint to us (ahead or behind)
    long closest_waypoint_idx = locate_closest_waypoint(ap1::math::vec2f{0.f, 0.f}, centerline);
    if (closest_waypoint_idx == -1) {
        return -1;
    }

    size_t closest_waypoint_idx_st = static_cast<size_t>(closest_waypoint_idx);

    for (size_t i = 0; i < MAX_WAYPOINT_ITER_COUNT && closest_waypoint_idx_st + i < centerline.size(); i++) {
        const auto &waypoint = centerline.at(closest_waypoint_idx_st + i);

        // if the waypoint is ahead, return it.
        // THIS ASSUMES THE CAR IS ORIENTED +X FWD, +Y LEFT
        if (waypoint.x > 0) {
            return closest_waypoint_idx;
        } else closest_waypoint_idx++;
    }

    // no waypoints ahead of car
    return -1;
}

} // namespace ap1::planning
