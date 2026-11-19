
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/state_context.hpp"

using ap1::planning::fsm::Event;
using ap1::planning::fsm::StateContext;
using ap1::planning::fsm::Transition;
using ap1::planning::fsm::VehicleState;

using namespace ap1::planning;

fsm::FSM::FSM(rclcpp::Time now, VehicleState default_state, std::vector<Transition> transitions)
    : context(now), current_state(default_state), transitions(std::move(transitions))
{
}

/**
 * @brief Figures out the next state.
 * NOTE: consumes the vector of events IN ORDER from MOST IMPORTANT TO LEAST.
 */
VehicleState ap1::planning::fsm::next_state(VehicleState current, std::vector<Event> events,
                                            const std::vector<Transition>& transitions)
{
    for (const auto& transition : transitions)
    {
        for (const auto event : events)
        {
            if (transition.from == current && transition.event == event)
            {
                return transition.to;
            }
        }
    }

    // otherwise just stay in the current state
    return current;
}

void fsm::on_state_change(const VehicleState from, const VehicleState to, const frames::MapF& frame,
                          StateContext& context)
{
    /* CONTEXT CHANGES MUST COME IN PAIRS TO CORRECTLY CLEAR EVENTS! */

    // if we've just stopped
    if (to == VehicleState::Stopped)
    {
        // start tracking the stopped time
        context.stop_entry_time = frame.time;
    }

    // if we're done stopping
    if (from == VehicleState::Stopped)
    {
        // stop tracking time
        context.stop_entry_time = std::nullopt;
    }

    // if we've just started driving away
    if (to == VehicleState::DrivingThrough)
    {
        // start tracking the distance
        context.drive_through_start_distance = frame.odometer;
    }

    // if we're done driving through
    if (from == VehicleState::DrivingThrough)
    {
        // stop tracking distance
        context.drive_through_start_distance = std::nullopt;
    }
}

std::vector<Transition> fsm::parse_transitions(const std::vector<std::string>& entries)
{
    if (entries.empty())
    {
        throw std::runtime_error("The transitions parameter must contain at least one transition");
    }

    std::vector<Transition> transitions;
    for (const auto& entry : entries)
    {
        std::istringstream fields(entry);
        std::string from_name, event_name, to_name, extra;
        if (!(fields >> from_name >> event_name >> to_name) || (fields >> extra))
        {
            throw std::runtime_error("Expected '<from> <event> <to>' in transition: " + entry);
        }

        const auto from = to_state(from_name);
        const auto event = to_event(event_name);
        const auto to = to_state(to_name);
        if (!from || !event || !to)
        {
            throw std::runtime_error("Unknown state or event in transition: " + entry);
        }

        transitions.push_back({*from, *event, *to});
    }

    return transitions;
}
