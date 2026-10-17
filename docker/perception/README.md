# AP1 Perception container

Builds a ROS 2 Jazzy image with `ap1_perception` and its dependencies
(`ap1_msgs`, rosdep system packages, and the `uv` Python environment from
`src/perception/uv.lock`), then launches the perception nodes.

## One-command build and launch

From the workspace root:

```bash
./scripts/launch_perception_container.sh
```

Equivalent Compose invocation:

```bash
docker compose -f docker/perception/compose.yaml up --build
```

This builds `ap1-perception:local` and runs:

```bash
ros2 launch ap1_perception perception.launch.py
```

## Environment

| Variable | Default | Purpose |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | DDS domain shared with other AP1 participants |
| `AP1_DOCKER_NETWORK_MODE` | `host` on Linux, `bridge` elsewhere | Docker network mode |
| `AP1_PERCEPTION_VENV` | `/opt/ap1/src/perception/.venv` | uv venv root (resolved dynamically; no host `$HOME` paths) |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS middleware |

## Interactive shell

```bash
docker compose -f docker/perception/compose.yaml run --rm perception bash
```

Inside the container, ROS and the workspace overlay are already sourced by the
entrypoint. Useful checks:

```bash
ros2 pkg list | grep ap1_
ros2 launch ap1_perception yolo.launch.py
```

## Notes

- Image build requires network access to pull `osrf/ros:jazzy-desktop`, clone
  `ap1_msgs`, and download Python wheels via `uv`.
- The `osrf/ros:jazzy-desktop` base is `linux/amd64`. On Apple Silicon, Docker
  Desktop runs it under emulation (expect a platform warning).
- The image is large (tens of GB) because `uv.lock` resolves the full Torch /
  CUDA wheel set used by perception.
- On Docker Desktop (macOS/Windows), host networking is unavailable; keep
  `ROS_DOMAIN_ID` consistent for processes that must discover each other.
- Camera / RealSense hardware is optional at runtime. Nodes wait for image
  topics when no sensor is attached.
