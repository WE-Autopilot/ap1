# AP1 | 2025-6

This is WE Autopilot's master workspace for development of the self-driving car system called AP1.

## Setup

- **Native (all subsystems):** see [SETUP.md](SETUP.md).
- **Docker (perception for now):** see [docker/README.md](docker/README.md). Containers are built and run with `./ap1d`.

## Repo Structure

```text
ap1d               Docker wrapper: build, run, and test module containers
ap1_setup.sh       Native setup script (see SETUP.md)
ap1.repos          External repos imported with vcs (src/msgs, src/external/)
docker/            Dockerfiles, bake file, and container tests
modules/           Docker Compose files used by ap1d
scripts/           Smoke tests and utilities (see scripts/README.md)
src/
  bringup/         ap1_bringup: launch files for the full system
  msgs/            ap1_msgs: messages between subsystems (imported from ap1.repos)
  perception/      ap1_perception
  mapping/         ap1_mapping       (Mapping & Localization)
  localization/    ap1_localization  (Mapping & Localization)
  planning/        ap1_planning      (Planning & Control)
  control/         ap1_control       (Planning & Control)
  console/         ap1_console: operator console UI
  external/        Third-party dependencies (imported from ap1.repos, gitignored)
```
