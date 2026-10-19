# AP1 Perception
---
Contains the perception team's codebase, revolving around the package 'ap1_perception', our ROS2 package.

## Container launch

From the AP1 workspace root:

```bash
./scripts/launch_perception_container.sh
```

This builds the ROS 2 Jazzy perception image (uv + rosdep deps) and starts
`ros2 launch ap1_perception perception.launch.py`. See
[docker/perception/README.md](../../docker/perception/README.md).
