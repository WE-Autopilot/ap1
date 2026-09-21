/**
 * Event generator is responsible for determining events that occur in the world through mapping frames.
 * These are then consumed
 */

#ifndef AP1_PLANNING_EVENT_GEN_HPP
#define AP1_PLANNING_EVENT_GEN_HPP

#include <optional>

#include "ap1/planning/state_context.hpp"
#include "rclcpp/time.hpp"

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/frames.hpp"

#define MIN_STOP_DURATION 3 // s
#define DRIVE_THROUGH_DISTANCE 1 // m
#define STOPPING_TRANSITION_DISTANCE 3 // m
#define EPSILON 0.01

using ap1::planning::frames::MapF;

namespace ap1::planning {
class EventGenerator {
public:
    EventGenerator();

    std::vector<ap1::planning::fsm::Event> update(
        const MapF& map,
        fsm::StateContext &ctx,
        rclcpp::Time now
    );    
};
}

#endif // AP1_PLANNING_EVENT_GEN_HPP