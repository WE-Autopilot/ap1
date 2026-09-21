/**
 * AP1's beautiful, beautiful planning node.
 * CAVEATS:
 * - Loads map ONLY ONCE at the beginning of the runtime.
 * - assumes left-right lane lines have same number of waypoints
 * - assumes left-right lane lines are directly accross from eachother.
 *
 * We should assemble a smooth curve out of waypoints and determine centerline based on that
 * instead.
 */

#include <cmath>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

#include "ap1/planning/behaviours.hpp"
#include "ap1/planning/event_generator.hpp"
#include "ap1/planning/frames.hpp"
#include "ap1/planning/fsm.hpp"
#include "ap1/planning/planner_node.hpp"

#include "ap1_msgs/msg/float_stamped.hpp"
#include "ap1_msgs/msg/lane_boundaries.hpp"
#include "ap1_msgs/msg/speed_profile_stamped.hpp"
#include "ap1_msgs/msg/target_path_stamped.hpp"

using geometry_msgs::msg::Point;

using ap1_msgs::msg::LaneBoundaries;
using ap1_msgs::msg::SpeedProfileStamped;
using ap1_msgs::msg::TargetPathStamped;

namespace ap1::planning
{
PlannerNode::PlannerNode(double rate_hz, std::string transitions_path)
    : Node("planner_node"), rate_hz_(rate_hz),
      fsm(this->now(), fsm::VehicleState::Driving, transitions_path), event_generator()
{
    // Declare path parameters with defaults.
    this->declare_parameter("topics.lanes",         
            "/ap1/mapping/lanes");
    //this->declare_parameter("topics.target_location",
    //      "/ap1/control/target_location);
    this->declare_parameter("topics.target_speed",  
            "/ap1/control/target_speed");
    this->declare_parameter("topics.odometer",      
            "/ap1/mapping/odometer");
    this->declare_parameter("topics.entities",      
            "/ap1/mapping/entities");
    this->declare_parameter("topics.speed",         
            "/ap1/actuation/speed");
    this->declare_parameter("topics.state",         
            "/ap1/planning/state");
    this->declare_parameter("topics.target_path",   
            "/ap1/planning/target_path");
    this->declare_parameter("topics.speed_profile", 
            "/ap1/planning/speed_profile");

    // # Subscribe to all inputs
    this->lane_sub_ = create_subscription<LaneBoundaries>(
        this->get_parameter("topics.lanes").as_string(), 1,
        [this](LaneBoundaries::SharedPtr msg) { this->on_lanes(msg); });

    // this->target_location_sub_ = create_subscription<Point>(
    //     this->get_parameter("topics.target_location").as_string(), 1,
    //     [this](Point::SharedPtr msg) { this->on_target_location(msg); });

    this->target_speed_sub_ = create_subscription<FloatStamped>(
        this->get_parameter("topics.target_speed").as_string(), 1,
        [this](FloatStamped::SharedPtr msg) { this->on_target_speed(msg); });

    this->odometer_sub_ = create_subscription<FloatStamped>(
        this->get_parameter("topics.odometer").as_string(), 1,
        [this](FloatStamped::SharedPtr msg) { this->on_odometer(msg); });

    this->entities_sub_ = create_subscription<EntityStateArray>(
        this->get_parameter("topics.entities").as_string(), 1,
        [this](EntityStateArray::SharedPtr msg) { this->on_entities(msg); });

    this->speed_sub_ = create_subscription<FloatStamped>(
        this->get_parameter("topics.speed").as_string(), 1,
        [this](FloatStamped::SharedPtr msg) { this->on_speed(msg); });

    // # Publishers
    this->state_pub_ = create_publisher<std_msgs::msg::String>(
        this->get_parameter("topics.state").as_string(), 1);
    this->target_path_pub_ = create_publisher<TargetPathStamped>(
        this->get_parameter("topics.target_path").as_string(), 1);
    this->speed_profile_pub_ =
        create_publisher<SpeedProfileStamped>(
        this->get_parameter("topics.speed_profile").as_string(), 1);

    // # Create Planning Loop @ rate_hz
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0f / rate_hz),
        [this]() { this->planning_loop_callback(); });

    RCLCPP_INFO(this->get_logger(), "Path Planner Node initialized.");

    // Publish state info once for startup.
    std_msgs::msg::String msg;
    msg.data = fsm::to_string(this->fsm.current_state).value();
    this->state_pub_->publish(msg);
}

// # Methods

void PlannerNode::planning_loop_callback()
{
    // check that we have all the necessary fields
    if (this->odometer_ == nullptr || this->current_lane_ == nullptr || this->entities_ == nullptr)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "1 or more necessary field is null. Skipping loop."
        );
        return;
    }

    // Check that field data is not too old. 
    const rclcpp::Time now = this->get_clock()->now();
    const auto odometer_age = (now - this->odometer_->header.stamp).seconds();
    const auto current_lane_age = (now - this->current_lane_->header.stamp)
        .seconds();
    const auto entities_age = (now - this->entities_->header.stamp).seconds();

    if (odometer_age > DATA_TTL_SEC || current_lane_age > DATA_TTL_SEC 
            || entities_age > DATA_TTL_SEC)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Stale data detected (odom: %.2fs, lane: %.2fs, entities: %.2fs). "
            "Skipping loop.",
            odometer_age, current_lane_age, entities_age
        );
        return;
    }

    // assemble frame
    const frames::MapF map_f{this->speed, this->odometer_->value, this->now(), *this->current_lane_, *this->entities_};

    // process events
    auto events = this->event_generator.update(map_f, this->fsm.context, this->now());

    // handle state
    fsm::VehicleState previous_state = this->fsm.current_state;
    if (!events.empty())
    {
        // update next state
        this->fsm.current_state =
            fsm::next_state(this->fsm.current_state, events, this->fsm.transitions);
    }
    if (this->fsm.current_state != previous_state)
    { // if the state has changed
        fsm::on_state_change(previous_state, this->fsm.current_state, map_f, this->fsm.context);

        // publish state
        std::string s(fsm::to_string(this->fsm.current_state).value());
        std_msgs::msg::String msg;
        msg.data = s;
        this->state_pub_->publish(msg);
    }

    // plan the route
    frames::RouteF route_f = behaviors::run_behaviour(this->fsm.current_state, map_f);

    // unwrap to route and speed profile messages
    TargetPathStamped path;
    SpeedProfileStamped speed_profile;
    frames::unwrap_route_f(route_f, path, speed_profile);
    const auto stamp = this->now();
    path.header.stamp = stamp;
    path.header.frame_id = this->current_lane_->header.frame_id;
    speed_profile.header.stamp = stamp;
    speed_profile.header.frame_id = this->current_lane_->header.frame_id;

    // publish
    target_path_pub_->publish(path);
    speed_profile_pub_->publish(speed_profile);
}

// # Callbacks
void PlannerNode::on_speed(const FloatStamped::SharedPtr speed)
{
    this->speed = speed->value;
}

void PlannerNode::on_lanes(const LaneBoundaries::SharedPtr lane)
{
    this->current_lane_ = lane;
}

void PlannerNode::on_target_speed(const FloatStamped::SharedPtr msg)
{
    this->target_speed_ = msg->value;

    std::string s = "Command: set speed to " + std::to_string(this->target_speed_);
    RCLCPP_INFO_STREAM(this->get_logger(), s);
}

void PlannerNode::on_odometer(const FloatStamped::SharedPtr msg)
{
    this->odometer_ = msg;
}

void PlannerNode::on_entities(const EntityStateArray::SharedPtr msg)
{
    this->entities_ = msg;
}

} // namespace ap1::planning
