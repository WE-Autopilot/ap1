#ifndef AP1_PLANNING_WAYPOINT_UTILS_HPP
#define AP1_PLANNING_WAYPOINT_UTILS_HPP

#include "ap1/planning/math_utils.hpp"

#include <vector>

#define MAX_WAYPOINT_ITER_COUNT 20 // how many waypoints to try ahead before giving up. See find_next_waypoint().

namespace ap1::planning
{

/**
 * @brief Locate the closest waypoint to a target location
 *
 * This function finds which navigable waypoint is nearest to the desired
 * target location. This is similar to how Google Maps finds the nearest
 * road to a map pin you drop.
 *
 * @param target_location The desired destination (where the user wants to go)
 * @param navigable_waypoints List of waypoints representing the navigable path
 *                           (e.g., lane centers, road waypoints)
 * @return int Index of the closest waypoint in the navigable_waypoints vector
 *                Returns -1 (as size_t max) if navigable_waypoints is empty
 *
 * Example:
 *   - User selects location eg: (10, 5)
 *   - Lane waypoints are at: [(8, 0), (9, 0), (10, 0), (11, 0)]
 *   - Function returns 2 (index of waypoint at (10, 0), closest to target)
 */
int locate_closest_waypoint(const ap1::math::vec2f& target_location,
                            const std::vector<ap1::math::vec2f>& navigable_waypoints);

/**
 * @brief Generate a sequence of waypoints from current position to destination
 *
 * This function creates the path the vehicle should follow. If lane waypoints
 * are available, it uses them (keeping the car on the road). If not, it falls
 * back to direct navigation (like driving in a parking lot with no lanes).
 *
 * @param waypoints List of navigable waypoints. Index 0 is the waypoint closest
 *                  to and directly in front of the car. Subsequent waypoints
 *                  extend forward along the navigable path.
 * @param to Index of the target waypoint (from locate_closest_waypoint).
 *           If -1, indicates no valid waypoint was found.
 * @param fallback_path Path to use when no waypoints are available or to == -1.
 *                      Typically a direct line to the target location.
 * @return vector<vec2f> Sequence of waypoints for the vehicle to follow
 *
 * Behavior:
 *   - If waypoints is empty OR to == -1: Returns fallback_path
 *   - Otherwise: Returns waypoints[0] through waypoints[to]
 *
 * Example with lanes:
 *   - waypoints = [(1,0), (2,0), (3,0), (4,0), (5,0)]
 *   - to = 3
 *   - Returns [(1,0), (2,0), (3,0), (4,0)] (car follows lane)
 *
 * Example without lanes:
 *   - waypoints = [] (empty)
 *   - fallback_path = [(5, 2)] (direct to target)
 *   - Returns [(5, 2)] (car drives directly to target)
 */
std::vector<ap1::math::vec2f> generate_waypoint_sequence(const std::vector<ap1::math::vec2f>& waypoints, const int to,
                                              const std::vector<ap1::math::vec2f>& fallback_path);

/**
 * @brief Finds the next waypoint in the path.
 * Does this by jumping forwards until the path has an x coord > 0 (bc of car-centered origin system
 * this is identical).
 * 
 * Returns -1 on failure.
 *
 * @return long
 */
long find_next_waypoint_idx(const std::vector<ap1::math::vec2f>& centerline);

} // namespace ap1::planning

#endif // AP1_PLANNING_WAYPOINT_UTILS_HPP
