
#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "ap1/planning/fsm.hpp"
#include "ap1/planning/state_context.hpp"

using ap1::planning::fsm::Event;
using ap1::planning::fsm::Transition;
using ap1::planning::fsm::VehicleState;
using ap1::planning::fsm::StateContext;

using namespace ap1::planning;

// Helper funcs
VehicleState state_from_string(const std::string &s);
Event event_from_string(const std::string &s);

// Beef and potatoes
fsm::FSM::FSM(rclcpp::Time now, VehicleState default_state, std::string transitions_path)
: context(now), current_state(default_state), transitions(load_transitions_from_file(transitions_path)) {}

/**
 * @brief Figures out the next state.
 * NOTE: consumes the vector of events IN ORDER from MOST IMPORTANT TO LEAST.
 */
VehicleState ap1::planning::fsm::next_state(VehicleState current, std::vector<Event> events, const std::vector<Transition> &transitions) {
    for (const auto& transition : transitions) {
        for (const auto event : events) {
            if (transition.from == current && transition.event == event) {
                return transition.to;
            }
        }
    }

    // otherwise just stay in the current state
    return current;
}

void fsm::on_state_change(
    const VehicleState from,
    const VehicleState to,
    const frames::MapF &frame,
    StateContext &context
) {
    /* CONTEXT CHANGES MUST COME IN PAIRS TO CORRECTLY CLEAR EVENTS! */

    // if we've just stopped
    if (to == VehicleState::Stopped) {
        // start tracking the stopped time
        context.stop_entry_time = frame.time;
    }
    
    // if we're done stopping
    if (from == VehicleState::Stopped) {
        // stop tracking time
        context.stop_entry_time = std::nullopt;
    }

    // if we've just started driving away
    if (to == VehicleState::DrivingThrough) {
        // start tracking the distance
        context.drive_through_start_distance = frame.odometer;
    }

    // if we're done driving through
    if (from == VehicleState::DrivingThrough) {
        // stop tracking distance
        context.drive_through_start_distance = std::nullopt;
    }
}

std::vector<Transition> fsm::load_transitions_from_file(std::string path) {
    // 1. Check extension
    std::filesystem::path fs_path(path);
    if (fs_path.extension() != ".yaml") {
        throw std::runtime_error("Invalid file extension. Expected .yaml");
    }

    // Load that bomboclaat file
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception &e) {
        throw std::runtime_error(
            std::string("Failed to load YAML file: ") + e.what()
        );
    }

    // Make sure we have a root
    if (!root["transitions"]) {
        throw std::runtime_error("Missing 'transitions' in config.");
    }

    // Build transitions vector
    std::vector<Transition> transitions;

    for (const auto& node : root["transitions"]) {
        if (!node.IsMap()) {
            throw std::runtime_error("Each transition must be a map");
        }

        if (!node["from"] || !node["event"] || !node["to"]) {
            throw std::runtime_error("Transntion is missing fields. Must have 'from', 'event', to'");
        }

        std::string from_str = node["from"].as<std::string>();
        std::string event_str = node["event"].as<std::string>();
        std::string to_str = node["to"].as<std::string>();

        // validate against enum options
        VehicleState from = to_state(from_str).value();
        Event event = to_event(event_str).value();
        VehicleState to = to_state(to_str).value();
    
        transitions.push_back({from, event, to});
    }

    if (transitions.empty()) {
        throw std::runtime_error("No transitions defined in file");
    }

    return transitions;
}

