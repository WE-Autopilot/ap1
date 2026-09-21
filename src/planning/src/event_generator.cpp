#include "ap1/planning/event_generator.hpp"

#include <iostream>
#include <vector>
#include <optional>

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/state_context.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "ap1/planning/frames.hpp"
#include "ap1/planning/planner_node.hpp"

#include "ap1_msgs/msg/entity_state.hpp"
#include "ap1_msgs/msg/entity_state_array.hpp"

using ap1::planning::fsm::Event;
using ap1::planning::frames::MapF;

using ap1_msgs::msg::EntityState;
using ap1_msgs::msg::EntityStateArray;

// Helpers
bool sign_is_close(const EntityStateArray& entities) {
    for (const EntityState& entity : entities.entities) {
        // skip signs behind us
        if (entity.x < 0) continue;

        // if the sign is close by
        if (entity.x < STOPPING_TRANSITION_DISTANCE) {
            return true;
        }
    }

    return false;
}

// Beef

/**
 * Create a new EventGenerator
 */
ap1::planning::EventGenerator::EventGenerator() {}


/**
 * Get the event generator to figure out all the events in the frame.
 * There are 4 events to check, see ap1::planning::fsm::Event for all options.
 */
std::vector<Event> ap1::planning::EventGenerator::update(
    const MapF& frame,
    fsm::StateContext &ctx,
    rclcpp::Time now
)
{
    // Determine the time that has been taken to stop so far.
    const rclcpp::Duration stop_duration = ctx.stop_entry_time.has_value()
        ? now - ctx.stop_entry_time.value()  // Given the recorded entry time.
        : rclcpp::Duration::from_seconds(0);

    // Determine the distance that has been driven through.
    const double drive_through_distance = ctx.drive_through_start_distance
            .has_value()
        ? frame.odometer - ctx.drive_through_start_distance.value()
            // Given the entry distance.
        : 0.0;

    // Check which events have been triggered.
    const std::vector<std::pair<bool, Event>> checks = {
        { 
            sign_is_close(frame.entities),
            Event::SignDetected
        },
        { 
            drive_through_distance > DRIVE_THROUGH_DISTANCE,
            Event::DriveThruDistanceCovered
        },
        {
            frame.speed <= EPSILON,
            Event::HasStopped
        },
        {
            stop_duration > rclcpp::Duration::from_seconds(MIN_STOP_DURATION),
            Event::StopTimeElapsed
        },
    };

    // Add the triggered events to the event vector and return it.
    std::vector<Event> events{};
    for (const auto& [condition, event] : checks) {
        if (condition) events.push_back(event);
    }
    return events;
}
