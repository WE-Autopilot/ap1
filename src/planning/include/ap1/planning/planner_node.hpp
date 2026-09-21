#ifndef AP1_PLANNING_NODE_HPP
#define AP1_PLANNING_NODE_HPP

#include <cmath>
#include <optional>

#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "ap1_msgs/msg/float_stamped.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/event_generator.hpp"

#define MAX_PLAN_AHEAD_WAYPOINTS 5

using rclcpp::TimerBase;
using geometry_msgs::msg::Point;

using ap1_msgs::msg::FloatStamped;
using ap1_msgs::msg::LaneBoundaries;
using ap1_msgs::msg::EntityStateArray;
using ap1_msgs::msg::TargetPathStamped;
using ap1_msgs::msg::SpeedProfileStamped;

namespace ap1::planning
{

/**
 * @brief Planner Node class. See latest PCI for interface documentation.
 */
class PlannerNode : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new Planner Node object
     *
     * @param rate_hz Primary loop update frequency.
     */
    PlannerNode(double rate_hz, std::string transitions_path);
  private:
    const double rate_hz_;
    float speed = 0; // car speed
    float target_speed_ = 0; // target speed

    fsm::FSM fsm;
    EventGenerator event_generator;

    // Memory
    FloatStamped::SharedPtr odometer_;
    EntityStateArray::SharedPtr entities_;
    LaneBoundaries::SharedPtr current_lane_;

    // Timer
    TimerBase::SharedPtr timer_;

    // Subscriptions
    rclcpp::Subscription<LaneBoundaries>::SharedPtr lane_sub_; // mapping
    rclcpp::Subscription<FloatStamped>::SharedPtr odometer_sub_; // mapping
    rclcpp::Subscription<EntityStateArray>::SharedPtr entities_sub_; // mapping

    rclcpp::Subscription<FloatStamped>::SharedPtr speed_sub_; // actuation

    rclcpp::Subscription<Point>::SharedPtr target_location_sub_; // console
    rclcpp::Subscription<FloatStamped>::SharedPtr target_speed_sub_; // console

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_; // console
    rclcpp::Publisher<TargetPathStamped>::SharedPtr target_path_pub_; // control
    rclcpp::Publisher<SpeedProfileStamped>::SharedPtr speed_profile_pub_; // control

    // # Callbacks
    void on_speed(const FloatStamped::SharedPtr speed);
    void on_odometer(const FloatStamped::SharedPtr od);
    void on_lanes(const LaneBoundaries::SharedPtr lanes);
    void on_target_speed(const FloatStamped::SharedPtr speed);
    void on_entities(const EntityStateArray::SharedPtr entities);
    void on_target_location(const geometry_msgs::msg::Point::SharedPtr loc);

    // Age Check
    static constexpr double DATA_TTL_SEC = 0.5;

    /**
     * @brief Planning loop callback runs rate_hz times per second.
     * This callback is responsible for sending commands & updates to control.
     * See P&C design for all Planning Loops.
     * See PCI for planning-control interface.
     */
    void planning_loop_callback();
};
} // namespace ap1::planning

#endif // AP1_PLANNING_NODE_HPP
