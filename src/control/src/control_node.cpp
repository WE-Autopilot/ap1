/**
 * Created: Nov. 23, 2025
 * Author(s): Aly Ashour
 */

#include <cmath>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/float_stamped.hpp"

#include "ap1/control/control_node.hpp"
#include "ap1/control/icontroller.hpp"
#include "ap1/control/pd_controller.hpp"
#include "vectors.hpp"

using namespace ap1_msgs::msg;
using namespace ap1::control;

void ControlNode::on_speed_profile(const SpeedProfileStamped::SharedPtr speed_profile)
{
    speed_profile_ = speed_profile;
}

void ControlNode::on_path(const TargetPathStamped::SharedPtr target_path)
{
    target_path_ = target_path;
}

void ControlNode::on_speed(const FloatStamped::SharedPtr speed)
{
    vehicle_speed_ = speed;
}

void ControlNode::on_turn_angle(const FloatStamped::SharedPtr turn_angle)
{
    vehicle_turn_angle = turn_angle;
}

// TODO: refactor. This is just a mess of spaghetti code vro.
void ControlNode::control_loop_callback()
{
    // check that we have the car's velocity yet
    if (!this->vehicle_speed_) { // TODO: really we should safety stop all of ap1 on a NAN
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Velocity is null or nan. Skipping update.");
        return;
    }

    // the car's current velocity - the car only ever moves forward G
    const vec3f velocity(this->vehicle_speed_->value, 0, 0); // +x is always forward on the car

    const bool PATH_IS_STALE = false, SPEED_PROFILE_IS_STALE = false; // TEMP

    // if path has no waypoints OR path is too old
    if (!target_path_ || target_path_->path.size() < 1 || PATH_IS_STALE) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Target path is invalid. Null, not enough waypoints, or old, skipping.");
        return;       
    }

    // if speed profile has no waypoints or speed profile is too old
    if (!speed_profile_ || speed_profile_->speeds.size() < 1 || SPEED_PROFILE_IS_STALE) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Speed profile is invalid. Null, not enough waypoints, or old, skipping.");
        return;
    }

    // ask the controller to calculate the acceleration needed
    const vec3f acc = controller_->compute_acceleration(velocity, target_path_, speed_profile_->speeds.at(0));

    // ALY'S FAVOURITE DEBUG CMD 1
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 700, "ACC: %.2f, %.2f, %.2f", acc.x, acc.y, acc.z);

    // compute acc and throttle using ackermann controller
    AckermannController::Command cmd = ackermann_controller_.compute_command(acc, velocity);

    // ALY'S FAVOURITE DEBUG CMD 2
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 700, "CMD: {throttle: %.2f, steering: %.2f, brake: %.2f}", cmd.throttle, cmd.steering, cmd.brake);

    // pack the turn angle into a message
    FloatStamped turn_msg;
    turn_msg.header.stamp = this->now();
    turn_msg.value = cmd.steering; // rads

    // pack the power into a message
    FloatStamped pwr_msg;
    pwr_msg.header.stamp = this->now();
    pwr_msg.value = cmd.throttle; // [-1, 1]

    // pack the brake into a message
    FloatStamped brake_msg;
    brake_msg.header.stamp = this->now();
    brake_msg.value = cmd.brake;

    // send all messages out
    turning_angle_pub_->publish(turn_msg);
    motor_power_pub_->publish(pwr_msg);
    brake_pub_->publish(brake_msg);
}

ControlNode::ControlNode(const std::string& cfg_path, float rate_hz)
    : Node("control_node"), rate_hz_(rate_hz), controller_(new PDController()),
      ackermann_controller_(AckermannController(AckermannController::load_config(cfg_path)))
{
    // Subs
    speed_profile_sub_ = this->create_subscription<SpeedProfileStamped>(
        "/ap1/planning/speed_profile", 1,
        std::bind(&ControlNode::on_speed_profile, this, std::placeholders::_1));
    target_path_sub_ = this->create_subscription<TargetPathStamped>(
        "/ap1/planning/target_path", 1,
        std::bind(&ControlNode::on_path, this, std::placeholders::_1));
    vehicle_speed_sub_ = this->create_subscription<FloatStamped>(
        "/ap1/actuation/speed", 1,
        std::bind(&ControlNode::on_speed, this, std::placeholders::_1));
    vehicle_turn_angle_sub_ = this->create_subscription<FloatStamped>(
        "/ap1/actuation/turn_angle", 1,
        std::bind(&ControlNode::on_turn_angle, this, std::placeholders::_1));

    // Pubs
    turning_angle_pub_ = this->create_publisher<FloatStamped>("/ap1/control/turn_angle", 1);
    motor_power_pub_ = this->create_publisher<FloatStamped>("/ap1/control/motor_power", 1);
    brake_pub_ = this->create_publisher<FloatStamped>("/ap1/control/brake", 1);

    // Create Control Loop
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
                                     std::bind(&ControlNode::control_loop_callback, this));

    // Log completion
    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
}
