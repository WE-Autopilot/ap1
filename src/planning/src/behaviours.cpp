#include "ap1/planning/behaviours.hpp"

#include <algorithm>
#include <stdexcept>

#include "ap1/planning/frames.hpp"
#include "ap1/planning/fsm.hpp"
#include "ap1/planning/math_utils.hpp"
#include "ap1/planning/planner_node.hpp"
#include "ap1/planning/waypoint_utils.hpp"

#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"

using namespace ap1::planning;
using namespace ap1::math;

using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;

// Behavior Handlers
frames::RouteF handle_driving(const frames::MapF& map);
frames::RouteF handle_stopping(const frames::MapF& map);
frames::RouteF handle_stopped(const frames::MapF& map);

// Map
constexpr std::array<std::pair<fsm::VehicleState, ap1::planning::behaviors::BehaviorFn>, 6>
    behavior_table{{
        {fsm::VehicleState::Driving, handle_driving},
        {fsm::VehicleState::Stopping, handle_stopping},
        {fsm::VehicleState::Stopped, handle_stopped},
        {fsm::VehicleState::DrivingThrough, handle_driving} // same as driving
    }};

// Helpers
// ASSUMES ALL ENTITIES ARE STOP SIGNS
const EntityState* get_next_sign(const EntityStateArray& entities)
{
    // filter for only those ahead and the closest
    const EntityState* closest = nullptr;
    for (const EntityState& entity : entities.entities)
    {
        bool is_ahead = entity.x > 0;

        // if it's behind us, skip
        if (!is_ahead)
            continue;

        // if we don't already have one
        if (closest == nullptr)
        {
            closest = &entity;
            continue;
        }

        // if it's closer
        if (magnitude(entity.x, entity.y) < magnitude(closest->x, closest->y))
        {
            closest = &entity;
            continue;
        }
    }

    return closest;
}

std::vector<vec2f> calculate_centerline(const LaneBoundaries lane)
{
    std::vector<vec2f> centerline;

    if (lane.left.size() != lane.right.size())
    {
        throw std::runtime_error("Left and right lane boundaries have different sizes!");
    }

    for (size_t i = 0; i < lane.left.size(); ++i)
    {
        centerline.emplace_back((lane.left[i].x + lane.right[i].x) / 2.0,
                                (lane.left[i].y + lane.right[i].y) / 2.0);
    }

    return centerline;
}

// Behavior Handlers
frames::RouteF ap1::planning::behaviors::run_behaviour(const fsm::VehicleState current_state,
                                                       const frames::MapF& map)
{
    for (const auto& behavior_pair : behavior_table)
    {
        const fsm::VehicleState state = behavior_pair.first;

        if (state == current_state)
            return behavior_pair.second(map);
    }

    throw std::runtime_error("State machine failed to find transition!");
}

/**
 * Handle driving.
 * The current behavior is centerline following with constant speed.
 */
frames::RouteF handle_driving(const frames::MapF& map)
{
    std::cout << "handle driving" << std::endl;
    // Check if there is a usable lane in map
    bool lane_is_usable = !map.lane.left.empty() && !map.lane.right.empty();
    if (!lane_is_usable)
    {                                  // if there's no lane
        return frames::RouteF{{}, {}}; // return empty
    }

    // If there is a usable lane, calculate its centerline
    std::vector<vec2f> centerline = calculate_centerline(map.lane); // throws on failure btw

    // Find the starting waypoint on this centerline
    long start_idx = find_next_waypoint_idx(centerline); // first waypoint ahead of car in lane
    if (start_idx == -1)
    {
        // if we failed to find one don't move
        return frames::RouteF{{{0, 0}}, {0}};
    }
    auto start =
        centerline.begin() + std::min(static_cast<unsigned long>(start_idx), centerline.size());

    // Find the ending waypoint on this centerline
    auto end = centerline.begin() +
               std::min(static_cast<unsigned long>(start_idx) + MAX_PLAN_AHEAD_WAYPOINTS,
                        centerline.size()); // replace with navigation later

    // Slice the centerline vector to get the route
    std::vector<vec2f> route(start, end);

    // return the route
    return frames::RouteF{
        route, {TARGET_SPEED} // should be received from planning somehow
    };
}

/**
 * Handle stopping.
 * Current behavior is continue driving except:
 * - clip path at stop sign
 * - set speed profile correctly
 *
 * Current behavior is to set speed based on distance to sign.
 * distance_to_sign / target_desceleration = m/s at current point
 */
frames::RouteF handle_stopping(const frames::MapF& map)
{
    // Get the stop sign
    const EntityState* next_sign = get_next_sign(map.entities);
    if (next_sign == nullptr)
    {
        throw std::runtime_error("Stopping with no upcoming sign?");
    }

    // Generate the route as if driving normally
    frames::RouteF route = handle_driving(map);

    // Clip the path at the stop sign
    long closest_centerline_idx =
        locate_closest_waypoint({next_sign->x, next_sign->y}, route.route);
    if (closest_centerline_idx == -1)
        throw std::runtime_error("Closest waypoint to sign was not found. Failing.");
    std::vector<vec2f> new_path(route.route.begin(),
                                route.route.begin() + closest_centerline_idx + 1);
    route.route = std::move(new_path);

    // Speed Profile
    float target_speed = 0.f;
    float distance_to_stop = next_sign->x; // distance to sign ykwim kinda cooked if our coords aren't right but whatever
    if (distance_to_stop < 0.1) target_speed = 0.f;
    else if (distance_to_stop < 1) target_speed = std::min(route.speed_profile.at(0), 0.2f);
    else if (distance_to_stop < 4) target_speed = std::min(route.speed_profile.at(0), 0.7f);
    else target_speed = map.speed;
    route.speed_profile = {target_speed};

    return route;
}

/**
 * Handle being stopped.
 * Current behavior is to sit still with v = 0.
 */
frames::RouteF handle_stopped(const frames::MapF&)
{
    return {{vec2f{0, 0}}, {0}};
}
