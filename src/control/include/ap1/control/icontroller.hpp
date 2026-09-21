/**
 * Abstract class (interface) defining what a controller is.
 * Date: Nov. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROLLER_HPP
#define AP1_CONTROLLER_HPP

#include "vectors.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"

using ap1_msgs::msg::TargetPathStamped;

namespace ap1::control
{
class IController
{
  public:
    ~IController() = default;
    virtual vec3f compute_acceleration(const vec3f& vel, const TargetPathStamped::ConstSharedPtr path, const float target_speed) = 0;
};
} // namespace ap1::control

#endif // AP1_CONTROLLER_HPP
