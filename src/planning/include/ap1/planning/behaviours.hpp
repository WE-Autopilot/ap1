/**
 * This contains callbacks for all of plannin's state behaviors.
 */
#ifndef AP1_PLANNING_BEHAVIORS_HPP
#define AP1_PLANNING_BEHAVIORS_HPP

#include "ap1/planning/frames.hpp"
#include "ap1/planning/fsm.hpp"

#define TARGET_SPEED 4.0f // ms^-1
#define MAX_BRAKING 2.0f // ms^-2

using namespace ap1::planning;

namespace ap1::planning::behaviors {
// BehaviorFn type
using BehaviorFn = frames::RouteF(*)(const frames::MapF&); // consumes a map frame and produces a route frame

// Primary usage twin
frames::RouteF run_behaviour(const fsm::VehicleState current_state, const frames::MapF &map);
}
#endif // AP1_PLANNING_BEHAVIOURS_HPP