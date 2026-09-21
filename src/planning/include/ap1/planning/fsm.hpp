#ifndef AP1_PLANNING_FSM_HPP
#define AP1_PLANNING_FSM_HPP

#include <array>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ap1/planning/frames.hpp"
#include "ap1/planning/state_context.hpp"

namespace ap1::planning::fsm
{

enum class VehicleState
{
    Driving,
    Stopping,
    Stopped,
    DrivingThrough
};

enum class Event
{
    SignDetected,    // if a sign **THAT APPLIES** has been detected. Not just any sign.
    HasStopped,      // v = 0
    StopTimeElapsed, // timer
    DriveThruDistanceCovered
};

struct Transition
{
    VehicleState from;
    Event event;
    VehicleState to;
};

struct FSM
{
    StateContext context;                // context vars
    VehicleState current_state;          // the state of the vehicle
    std::vector<Transition> transitions; // transitions between states

    FSM(rclcpp::Time now, VehicleState default_state, std::string transitions_path);
};

// Lookup tables
constexpr std::array<std::pair<std::string_view, VehicleState>, 4> state_table{
    {{"Driving", VehicleState::Driving},
     {"Stopping", VehicleState::Stopping},
     {"Stopped", VehicleState::Stopped},
     {"DrivingThrough", VehicleState::DrivingThrough}}};

constexpr std::array<std::pair<std::string_view, Event>, 4> event_table{
    {{"SignDetected", Event::SignDetected},
     {"HasStopped", Event::HasStopped},
     {"StopTimeElapsed", Event::StopTimeElapsed},
     {"DriveThruDistanceCovered", Event::DriveThruDistanceCovered}}};

/**
 * @brief Loads a transition table from a yaml file.
 *
 * @param path
 * @return std::vector<Transition>
 */
std::vector<Transition> load_transitions_from_file(std::string path);

/**
 * @brief Transitions to the next state based on a transition table and current state.
 *
 * If no transition is found for the current state and event, the current state is returned back.
 */
VehicleState next_state(VehicleState current, std::vector<Event> events,
                        const std::vector<Transition>& transitions);

/**
 * @brief On state transition callback.
 * MUTATES: context
 *
 * Used to update context.
 */
void on_state_change(const VehicleState from, const VehicleState to, const frames::MapF& frame,
                    StateContext& context);

// STRING ENCODING/DECODING
constexpr std::optional<std::string_view> to_string(VehicleState state)
{
    for (auto& [name, s] : fsm::state_table)
    {
        if (s == state)
            return name;
    }

    return std::nullopt;
}

constexpr std::optional<std::string_view> to_string(Event event)
{
    for (auto& [name, e] : fsm::event_table)
    {
        if (e == event)
            return name;
    }

    return std::nullopt;
}

constexpr std::optional<VehicleState> to_state(const std::string& n)
{
    for (auto& [name, s] : fsm::state_table)
    {
        if (n == name)
        {
            return s;
        }
    }

    return std::nullopt;
}

constexpr std::optional<Event> to_event(const std::string& n)
{
    for (auto& [name, e] : fsm::event_table)
    {
        if (n == name)
        {
            return e;
        }
    }

    return std::nullopt;
}
} // namespace ap1::planning::fsm
#endif // AP1_PLANNING_FSM_HPP