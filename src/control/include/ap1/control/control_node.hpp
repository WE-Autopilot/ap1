/**
 * Created: Oct. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROL_NODE_HPP
#define AP1_CONTROL_NODE_HPP

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"

#include "ap1/control/ackermann_controller.hpp"
#include "ap1/control/icontroller.hpp"

namespace ap1::control
{
class ControlNode : public rclcpp::Node
{
  private:
    // Fields

    // Control Loop
    const double rate_hz_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Controller
    std::unique_ptr<IController> controller_;
    AckermannController ackermann_controller_;

    // Memory
    ap1_msgs::msg::SpeedProfileStamped::SharedPtr speed_profile_;
    ap1_msgs::msg::TargetPathStamped::SharedPtr target_path_;
    ap1_msgs::msg::FloatStamped::SharedPtr vehicle_speed_;
    ap1_msgs::msg::FloatStamped::SharedPtr vehicle_turn_angle;

    // Subs
    rclcpp::Subscription<ap1_msgs::msg::TargetPathStamped>::SharedPtr target_path_sub_;
    rclcpp::Subscription<ap1_msgs::msg::SpeedProfileStamped>::SharedPtr speed_profile_sub_;
    rclcpp::Subscription<ap1_msgs::msg::FloatStamped>::SharedPtr vehicle_speed_sub_;
    rclcpp::Subscription<ap1_msgs::msg::FloatStamped>::SharedPtr vehicle_turn_angle_sub_;

    // Pubs
    rclcpp::Publisher<ap1_msgs::msg::FloatStamped>::SharedPtr turning_angle_pub_;
    rclcpp::Publisher<ap1_msgs::msg::FloatStamped>::SharedPtr motor_power_pub_;
    rclcpp::Publisher<ap1_msgs::msg::FloatStamped>::SharedPtr brake_pub_;

    // Methods
    void on_speed_profile(const ap1_msgs::msg::SpeedProfileStamped::SharedPtr speed_profile);
    void on_path(const ap1_msgs::msg::TargetPathStamped::SharedPtr target_path);
    void on_speed(const ap1_msgs::msg::FloatStamped::SharedPtr speed);
    void on_turn_angle(const ap1_msgs::msg::FloatStamped::SharedPtr turn_angle);
    void control_loop_callback();

  public:
    ControlNode(const std::string& cfg_path, float rate_hz = 60);
};
} // namespace ap1::control

#endif // AP1_CONTROL_NODE_HPP
