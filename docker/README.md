# AP1 Docker

Containerized build and dev environment for AP1 modules, modelled on
[WATonomous' wato_monorepo](https://github.com/WATonomous/wato_monorepo).
Perception is the first module on it. Other modules still use `ap1_setup.sh`.

Every image comes in two variants built from the same files:

| Variant | Base image | onnxruntime | For |
|---|---|---|---|
| `cpu` | `ubuntu:24.04` | `onnxruntime` | anyone, no GPU needed (image ~2.9 GB) |
| `gpu` | `nvidia/cuda:13.0.3-cudnn-runtime-ubuntu24.04` | `onnxruntime-gpu` | NVIDIA GPU machines (image ~8.3 GB) |

The code is identical in both. Only the installed packages differ, and the
YOLO node's `device` parameter picks the hardware.

## Setup

1. Docker with Buildx and Compose.
   - Arch: `sudo pacman -S docker docker-buildx docker-compose`
   - Ubuntu: [Docker's apt repo](https://docs.docker.com/engine/install/ubuntu/)
     (includes `docker-buildx-plugin` and `docker-compose-plugin`)
   - Add yourself to the `docker` group, then log out and back in.
2. GPU machines only: NVIDIA driver 580 or newer (needed for CUDA 13) and the
   [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html),
   then `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`.
3. Put the model where containers can see it (`models/` is gitignored and
   mounted read-only):
   ```bash
   mkdir -p models && cp src/perception/ap1_perception/yolo/yolo11n.onnx models/
   ```

## Everyday use

```bash
./ap1d up perception --dev     # build the dev image and start the dev container
./ap1d shell perception        # shell inside it
```

Inside the container, `src/perception` from your checkout is mounted at
`/ws/src/perception`, so edits on the host show up immediately:

```bash
colcon build --symlink-install     # builds into /ws/install (kept across restarts)
source install/setup.bash
ros2 launch ap1_perception yolo.launch.py \
    model_path:=/opt/ap1/models/yolo11n.onnx device:=cpu   # or device:=cuda
```

Add `--gpu` to any command for the GPU variant, or set it once with
`export AP1_VARIANT=gpu` in your shell rc.

| Command | What it does |
|---|---|
| `./ap1d up perception` | Run the YOLO node from the deploy image (CPU, or `--gpu` for CUDA) |
| `./ap1d logs perception` | Follow its logs |
| `./ap1d test perception` | Smoke test the deploy image (imports, cv_bridge, YOLO inference) |
| `./ap1d e2e perception` | End-to-end test: starts the YOLO node, feeds it a stop-sign photo (downloaded to a temp folder and deleted afterwards) with camera info and a point cloud over ROS, and checks the published entity |
| `./ap1d build perception [--dev]` | Build an image without starting it |
| `./ap1d down` | Stop all AP1 containers |

`device:=cuda` fails at startup if the GPU isn't usable, rather than silently
running on CPU. `device:=auto` uses the GPU when available.

Containers use host networking and IPC, so they talk to ROS nodes on the host
and in other containers as long as `ROS_DOMAIN_ID` (and `RMW_IMPLEMENTATION`,
if you set one) match. Recordings in `bags/` appear at `/ws/bags`.

ROS discovery stays on your machine by default (`ap1d` sets
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), so people developing on the same
network, like campus Wi-Fi, never see each other's topics. To talk to another
machine (the car, a teammate's laptop), run `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
on both before starting, and use the same `ROS_DOMAIN_ID`. For native (non-Docker)
work on shared networks, put `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in
your shell rc too.

`./ap1d` runs every container as your host user. Fast DDS shared memory only
works between processes of the same user, so native nodes must run as the
user who ran `./ap1d` (don't use `sudo ./ap1d`). If you previously ran
containers as root, remove their leftover shared-memory files once with
`sudo rm /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` while no ROS nodes are running.

## Adding a dependency

1. Declare it in `src/perception/package.xml` with its
   [rosdep key](https://github.com/ros/rosdistro/tree/master/rosdep). Prefer apt
   keys (`python3-...`). Keys ending in `-pip` install from PyPI.
2. Pin any pip package in `config/pip-constraints.txt`. rosdep can't pin
   versions, so this file is what keeps builds reproducible.
3. Something rosdep can't express at all goes in the `dependencies` stage of
   `perception.Dockerfile`. This is a last resort.

If a dependency differs between CPU and GPU, use a condition on `AP1_GPU`, like
the onnxruntime entries in `package.xml`.

Never pip-install `numpy`: ROS's `cv_bridge` is built against Ubuntu's numpy
(1.26). The constraints file blocks upgrades, and the smoke test checks it.

## Layout

```
ap1d                          wrapper for bake + compose (run from anywhere)
.dockerignore                 keeps models, bags, build output out of images
modules/                      how modules run (one service per module)
  docker-compose.yaml         deploy services: mounts, networking, launch commands
  docker-compose.dev.yaml     dev services: extend deploy, mount source, idle
  docker-compose.gpu.yaml     GPU access, only applied with --gpu
docker/                       how images are built
  base.Dockerfile             ROS Jazzy + AP1 tooling on any Ubuntu 24.04 image
  template.Dockerfile         shared pipeline: rosdep scan -> install -> build -> deploy/develop
  perception.Dockerfile       perception's sources and extra dependencies
  docker-bake.hcl             build graph: variants x targets (used by ap1d and CI)
  config/                     entrypoint, shell env, pip version pins, pyproject mask
  tests/perception_smoke.py   smoke test, run by ./ap1d test and CI
  tests/perception_e2e.py     end-to-end ROS test, run by ./ap1d e2e
```

## How the build works

`docker-bake.hcl` chains three Dockerfiles per variant:

1. `base.Dockerfile` installs ROS on the variant's starting image. It's
   rebuilt only when that file changes.
2. `perception.Dockerfile` defines `source` (`src/msgs` + `src/perception`)
   and `dependencies` (the base plus anything rosdep can't express).
3. `template.Dockerfile` takes those two stages:
   - `rosdep_scan` resolves every `package.xml` into apt and pip package lists.
   - `dependencies` installs the lists. Docker caches this layer until the
     lists change, so editing code never reinstalls packages.
   - `build` runs `colcon build` into `/opt/ap1`.
   - `deploy` copies only `/opt/ap1` on top of the dependencies, with no sources
     or build tree. This image runs on the car.
   - `develop` is deploy plus dev tools and a user matching your UID/GID, so
     files created in mounted folders stay yours.

New modules add a `<module>.Dockerfile`, targets in `docker-bake.hcl`, and a
service in each `modules/` compose file. They reuse the base and the template.

## CI

`.github/workflows/perception-images.yml` builds both variants on every PR
that touches `docker/`, `src/perception/` or `src/msgs/`, and runs the smoke
test in each.
On `main` it pushes `ghcr.io/we-autopilot/ap1/perception:{cpu,gpu}` plus a
per-commit tag. The runners have no GPU, so the GPU image's inference runs on
CPU there. CI only checks that its onnxruntime has CUDA support.

## Not covered yet

- **UFLD node.** It needs torch, which isn't in the image. Only the YOLO node
  is supported.
- **Live RealSense camera.** The container has no USB device access yet. Replay
  bags instead (see `SETUP.md` section 7; run it inside the dev container).
- **Jetson / arm64 images.** These would be a new entry in `VARIANTS` in
  `docker-bake.hcl` once the Orin's JetPack version is settled.
