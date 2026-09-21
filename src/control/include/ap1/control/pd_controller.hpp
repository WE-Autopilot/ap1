/**
 * PD Controller Implementation
 * Date: Nov. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_PD_CONTROLLER_HPP
#define AP1_PD_CONTROLLER_HPP

#include <cmath>
#include <iostream>
#include <sys/types.h>

#include "ap1/control/icontroller.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "vectors.hpp"

#define EPSILON 1e-6
#define MAX_WAYPOINT_SEARCH 5 // how far into the planned path to search for a path far from the current position

using ap1_msgs::msg::TargetPathStamped;

/**
 * @brief Figure's out the first, next up target position.
 * @return vec2f Returns a vec containing the first waypoint that is further than EPSILON from the origin.
 */
inline vec2f get_target_position(const TargetPathStamped::ConstSharedPtr path) {
    uint wpt_idx = 0;

    while (wpt_idx < MAX_WAYPOINT_SEARCH && wpt_idx < path->path.size()) {
        const vec2f next{(float) path->path.at(wpt_idx).x, (float) path->path.at(wpt_idx).y};
        const vec2f origin{0, 0};

        // if the waypoint and the origin are far enough apart
        if (distance(next, origin) > EPSILON) {
            // return it
            return next;
        }

        // otherwise increment and try again
        wpt_idx++;
    }

    // OPERATION FAILED!
    // at this point in development I'm just trying to get it working
    // this leaves me with 2 very bad options
    // option 1: I return 0 here and let jesus take the wheel
    // option 2: I can throw a runtime error and just take all of control down
    // both options are kind of shit I can't lie
    // really this should propagate upwards to some kind of system that takes care of it, decelerating the car or keeping it stopped depending
    // on if we're starting up or moving or whatever
    // but considering we don't have that I'm just going to take option 1.
    return {0, 0};
    // option 2 for fun:
    // throw std::runtime_error("MAX_WAYPOINT_SEARCH exceeded! Couldn't find a waypoint far enough."); 
}

namespace ap1::control
{
class PDController : public IController
{
    // Stength of the proportional and derivative terms
    float kp_, kd_;
    // Last velocity vector
    vec3f last_vel_;

  public:
    PDController(float kp = 2.0, float kd = 0.5) : kp_(kp), kd_(kd) {};

    // Computes acceleration based on current velocity, target position, and target speed. Returns
    // acceleration vector.
    virtual vec3f compute_acceleration(
        const vec3f& vel,
        const TargetPathStamped::ConstSharedPtr path,
        const float target_speed
    ) override {
        if (!path) {
            printf("Unc is null twin");
            return {0, 0, 0};
        }

        const auto target_pos = get_target_position(path);

        // Get the very next target position
        // Convert target position to vec3f and calculate distance
        const float distance = target_pos.length();

        // targetDir for normalizing target position into unit direction
        vec2f targetDir;
        if (distance > EPSILON)
            targetDir = target_pos.unit_vector();
        else
        {
            targetDir = vec2f(0.0, 0.0);
        }

        // Scale direction vector by target speed for velocity
        vec3f targetVel = targetDir * target_speed;

        // Store current velocity for derivative approximation
        last_vel_ = vel;

        // Calculate velocity change (numerical derivative)
        vec3f drv = vel - last_vel_;

        // PD controller: proportional term tracks target velocity, derivative term adds damping
        return kp_ * (targetVel - vel) - kd_ * drv;
    };
};
} // namespace ap1::control

#endif // AP1_PD_CONTROLLER_HPP
