#ifndef AP1_PLANNING_STATE_CONTEXT_HPP
#define AP1_PLANNING_STATE_CONTEXT_HPP

#include <optional>

#include "rclcpp/time.hpp"

namespace ap1::planning::fsm {
struct StateContext {
    // Stop related
    std::optional<rclcpp::Time> stop_entry_time = std::nullopt;

    // Drive-through related
    std::optional<float> drive_through_start_distance = std::nullopt;

    // Time
    rclcpp::Time now;

    StateContext(rclcpp::Time now): now(now) {}
};
}

#endif // AP1_PLANNING_STATE_CONTEXT_HPP