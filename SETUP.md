# AP1 Developer Setup Guide

## Prerequisites

- Ubuntu (with ROS2 Jazzy installed at `/opt/ros/jazzy`)
- `uv` installed ([install guide](https://docs.astral.sh/uv/getting-started/installation/))
- `zsh` or `bash`
- `colcon` build tool

---

## 1. Clone the Workspace

```bash
mkdir -p ~/Documents/ap1/src
cd ~/Documents/ap1/src

vcs import < ap1.repos
```

---

## 2. Set Up the Perception Python Environment

The perception package uses `uv` to manage its Python dependencies (ultralytics, onnxruntime, PyQt5, torch, etc.).

```bash
cd ~/path_to_ap1/ap1/src/perception
uv sync
```

This creates a `.venv` inside the perception folder with all pinned dependencies from `uv.lock`.

---

## 3. Expose the Perception venv to ROS2

ROS2 uses the system Python (`/usr/bin/python3`) to launch nodes — it does **not** respect the active venv automatically. You need to add the venv's site-packages to `PYTHONPATH`.

```bash
echo 'export PYTHONPATH=/home/$USER/rest_of_path/ap1/src/perception/.venv/lib/python3.12/site-packages:$PYTHONPATH' >> ~/.zshrc
source ~/.zshrc
```

> ⚠️ If your username or workspace path differs, adjust accordingly.

---

## 4. Build the Workspace

> **Important:** Always run `colcon build` from the **workspace root** (`~/Documents/ap1`), NOT from inside `src/`. Building from inside `src/` puts `build/`, `install/`, and `log/` in the wrong place and ROS2 won't find your packages.

```bash
cd ~/Documents/ap1

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **Tip:** For C++ IDE support (clangd autocomplete in `ap1_control`/`ap1_planning`, i.e., this is mainly for planning & control team):
> ```bash
> colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
> cp build/compile_commands.json .
> ```

---

## 5. Source the Workspace

After a successful build, source the overlay. **Do this in every new terminal before running any ROS2 commands.**

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/ap1/install/setup.bash
```

> 💡 To avoid doing this manually every time, add both lines to your `~/.zshrc` or `~/.bashrc`:
> ```bash
> echo 'source /opt/ros/jazzy/setup.bash' >> ~/.zshrc
> echo 'source ~/Documents/ap1/install/setup.bash' >> ~/.zshrc
> ```

---

## 6. Launch the System

### Full System (all nodes)
```bash
ros2 launch ap1_bringup full_system.launch.py
```

### Planning, Control & Sim only
```bash
ros2 launch ap1_bringup pncbackend.launch.py
```

### Mapping & Localization pipeline only
```bash
ros2 launch mapping_localization_python mapping_pipeline.launch.py

# With Kitware SLAM enabled:
ros2 launch mapping_localization_python mapping_pipeline.launch.py use_kitware_slam:=true

# With synthetic perception data (no real sensors needed):
ros2 launch mapping_localization_python mapping_pipeline.launch.py use_synthetic_perception:=true
```

### Console UI only
```bash
ros2 run ap1_console console
```

---

## 7. Verify Everything is Running

```bash
# List all active nodes
ros2 node list

# Expected nodes on fullsystem launch:
# /ap1_control
# /ap1_planning
# /ap1_yolo
# /ap1_ufld_ground
# /ap1_console
# /perception_pipeline
# /stored_point_registry
# /slam_bridge
# /base_to_lidar_tf

# Check a topic is publishing
ros2 topic echo /ap1/localization/slam_pose
```

---

## Quick Reference — Every New Terminal

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/ap1/install/setup.bash
ros2 launch ap1_bringup full_system.launch.py
```

---

## Troubleshooting

| Error | Cause | Fix |
|---|---|---|
| `Package 'ap1_bringup' not found` | `colcon build` was run from inside `src/` | `rm -rf src/build src/install src/log` then rebuild from workspace root |
| `No module named 'ultralytics'` | Perception venv not on `PYTHONPATH` | Add venv `site-packages` to `PYTHONPATH` (see Step 3) |
| `No module named 'onnxruntime'` | Same as above | Same fix |
| `No module named 'PyQt6'` | Console requires PyQt6 | `pip install PyQt6` into the active venv |
| All nodes shut down immediately | A `CriticalNode` crashed (usually `yolo_node`) | Fix the crashing node first — it takes down the whole system on exit |
| `WARN: Velocity is null` / `necessary field is null` | Normal at startup | Nodes are waiting for sensor data — not an error |