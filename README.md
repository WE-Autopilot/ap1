# AP1 | 2025-6

This is WE Autopilot's master workspace for development of the self-driving car system called AP1.

For setup see SETUP.md.

## Repo Structure

- `src/bringup`: launch files for the full system
- `src/msgs`: `ap1_msgs`, messages between subsystems
- `src/perception`: Perception
- `src/mapping`, `src/localization`: Mapping & Localization
- `src/planning`, `src/control`: Planning & Control
- `src/console`: operator console UI
- `simulation/`: CARLA bag recording and sensor filtering (see `simulation/README.md`)
- `scripts/`: helper scripts, including `map_carla.sh` (see `src/mapping/README.md`)
