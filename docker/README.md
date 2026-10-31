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
   - Windows: use WSL 2 and follow the Ubuntu steps inside it. See
     [Windows (WSL 2)](#windows-wsl-2) first.
   - macOS: [Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/)
     (includes Buildx and Compose). See [macOS](#macos) for what works.
   - Linux and WSL: add yourself to the `docker` group, then log out and back in.
2. GPU machines only (Linux and WSL): NVIDIA driver 580 or newer (needed for CUDA 13) and the
   [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html),
   then `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`.
3. The YOLO model is mounted straight from `src/perception/ap1_perception/yolo/yolo11n.onnx`,
   which is in Git LFS. If that file is a small text pointer instead of ~10 MB,
   run `git lfs pull`.

### Windows (WSL 2)

Run everything, including the repo and Docker, inside an Ubuntu WSL distro. It
then behaves like an Ubuntu machine.

1. In an admin PowerShell: `wsl --install -d Ubuntu-24.04`, reboot, and open
   Ubuntu from the Start menu.
2. Clone the repo inside WSL (e.g. `~/code/ap1`), not under `/mnt/c`. Mounts
   from the Windows drive are slow and lose file ownership, and a Windows-side
   clone can check out shell scripts (like the container entrypoint) with CRLF
   line endings, which breaks them.
3. Install Docker Engine inside WSL with the Ubuntu steps above. It needs
   systemd, which new WSL installs turn on. Check that `/etc/wsl.conf` contains
   ```ini
   [boot]
   systemd=true
   ```
   If not, add it and run `wsl --shutdown` in PowerShell.

   Prefer this over Docker Desktop: Docker Desktop's engine runs outside your
   Ubuntu distro, so `network_mode: host` and `ipc: host` don't connect containers
   to native ROS nodes in WSL the way they do on Linux. If Docker Desktop is
   installed, turn off its WSL integration for this distro (Settings > Resources >
   WSL Integration) so `docker` uses the engine inside WSL.
4. GPU: install the NVIDIA driver (580 or newer) on Windows only. WSL uses it
   automatically, so never install a Linux NVIDIA driver inside WSL. Then
   install the NVIDIA Container Toolkit inside WSL (setup step 2). `nvidia-smi`
   in WSL should list your GPU.
5. Talking to other machines (`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, see below):
   WSL's default NAT networking keeps ROS discovery off your LAN. Switch to
   mirrored networking (Windows 11 22H2 or newer) by putting this in
   `%UserProfile%\.wslconfig` and running `wsl --shutdown`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
   Windows Firewall may also need to allow the traffic.

### macOS

Docker Desktop runs containers in a Linux VM, on both Intel and Apple Silicon
(Apple Silicon builds arm64 images natively).

- Every command below works with the CPU variant. There's no GPU variant:
  `docker-compose.gpu.yaml` and `device:=cuda` need an NVIDIA GPU.
- Keep the repo under your home folder. Docker Desktop shares `/Users` with
  containers by default.
- Host networking means the VM's network, not your Mac's. Containers reach each
  other fine (dev container, smoke and end-to-end tests), but ROS discovery doesn't reach
  other machines, so `SUBNET` won't find the car or teammates. Use a Linux
  machine for that.
- CI only builds x86_64 images, so arm64 builds aren't tested there.

## Everyday use

Images are built with `docker buildx bake` and run with `docker compose`. Run
everything from the repo root, in a shell where compose knows the module files:

```bash
export COMPOSE_FILE=modules/docker-compose.yaml:modules/docker-compose.dev.yaml
```

Then build the dev image, start the dev container and open a shell in it:

```bash
docker buildx bake -f docker/docker-bake.hcl --load perception-dev-cpu
docker compose up -d perception-dev
docker compose exec perception-dev bash
```

Inside the container, `src/msgs` and `src/perception` from your checkout are
mounted at `/ws/src`, so edits on the host show up immediately. The dev image
has no prebuilt copy of them: build both in the container:

```bash
colcon build --symlink-install     # builds into /ws/install (kept across restarts)
source install/setup.bash
ros2 launch ap1_perception yolo.launch.py \
    model_path:=/opt/ap1/models/yolo11n.onnx device:=cpu   # or device:=cuda
```

The deploy image runs the YOLO node on its own:

```bash
docker buildx bake -f docker/docker-bake.hcl --load perception-cpu
docker compose up -d perception        # start the YOLO node
docker compose logs -f perception      # follow its logs
docker compose --profile '*' down      # stop all AP1 containers (keeps dev build volumes)
```

Compose never builds images. After changing code or Dockerfiles, rebuild with
bake, then `docker compose up -d` again recreates the container from the new
image. If an image isn't built locally, compose pulls it from the registry.

### Tests

Both tests run in a second container from the deploy image, with
`docker/tests` mounted in.

Smoke test (imports, cv_bridge, YOLO inference):

```bash
docker compose run --rm --no-deps -v "$PWD/docker/tests:/tests:ro" perception \
    python3 /tests/perception_smoke.py --model /opt/ap1/models/yolo11n.onnx --device cpu
```

End-to-end test: with the YOLO node running (`docker compose up -d perception`),
it feeds the node a stop-sign photo (downloaded to a temp folder and deleted
afterwards) with camera info and a point cloud over ROS, and checks the
published entity. Run it with the same variant as the running node.

```bash
docker compose run --rm --no-deps -v "$PWD/docker/tests:/tests:ro" perception \
    python3 /tests/perception_e2e.py
docker compose logs perception        # what the node logged during the test
```

### GPU

For the GPU variant, build the `-gpu` targets (`perception-gpu`,
`perception-dev-gpu`) and set this before running compose:

```bash
export AP1_VARIANT=gpu AP1_DEVICE=cuda
export COMPOSE_FILE=$COMPOSE_FILE:modules/docker-compose.gpu.yaml
```

`device:=cuda` fails at startup if the GPU isn't usable, rather than silently
running on CPU. `device:=auto` uses the GPU when available. The smoke test
takes `--device cuda` the same way.

### Settings

Compose and bake read these from your shell. All are optional:

| Variable | Default | What it sets |
|---|---|---|
| `AP1_VARIANT` | `cpu` | image variant compose runs: `cpu` or `gpu` |
| `AP1_DEVICE` | `cpu` | YOLO inference device in the deploy service: `cpu` or `cuda` |
| `AP1_UID`, `AP1_GID` | `1000` | user the containers run as, and the dev image's user (at build time) |
| `AP1_REGISTRY` | `ghcr.io/we-autopilot/ap1` | image name prefix |
| `AP1_TAG_SUFFIX` | empty | appended to the deploy image tag, e.g. `-<git sha>` to run an exact CI-built image |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | how far ROS discovery reaches |

### Networking and users

Containers use host networking and IPC, so they talk to ROS nodes on the host
and in other containers as long as `ROS_DOMAIN_ID` (and `RMW_IMPLEMENTATION`,
if you set one) match. Recordings in `bags/` appear at `/ws/bags`.

ROS discovery stays on your machine by default (the compose file defaults
`ROS_AUTOMATIC_DISCOVERY_RANGE` to `LOCALHOST`), so people developing on the
same network, like campus Wi-Fi, never see each other's topics. To talk to another
machine (the car, a teammate's laptop), run `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
on both before starting, and use the same `ROS_DOMAIN_ID`. The value is
case-sensitive, and ROS silently falls back to localhost-only on an invalid one
(e.g. `subnet`). For native (non-Docker) work on shared networks, put
`export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in your shell rc too.

Containers run as `AP1_UID:AP1_GID`, which must be your host user, so files
created in mounted folders stay yours. The default, 1000, is the first user on
most Linux machines. If `id -u` or `id -g` prints something else, put
`export AP1_UID=$(id -u) AP1_GID=$(id -g)` in your shell rc and rebuild the dev
image. Fast DDS shared memory only works between processes of the same user, so
native nodes must run as that user too (don't use `sudo`). If you previously
ran containers as root, remove their leftover shared-memory files once with
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
.dockerignore                 keeps models, bags, build output out of images
bags/                         recordings, mounted at /ws/bags (contents gitignored)
modules/                      how modules run (one service per module)
  docker-compose.yaml         deploy services: mounts, networking, launch commands
  docker-compose.dev.yaml     dev services: extend deploy, mount source, idle
  docker-compose.gpu.yaml     GPU access, added for the gpu variant
docker/                       how images are built
  base.Dockerfile             ROS Jazzy base, plus the shared pipeline:
                              rosdep scan -> install -> build -> deploy/develop
  perception.Dockerfile       perception's sources and extra dependencies
  docker-bake.hcl             build graph: variants x targets (used locally and by CI)
  config/                     entrypoint, shell env, pip version pins, pyproject mask
  tests/perception_smoke.py   smoke test, run locally and by CI
  tests/perception_e2e.py     end-to-end ROS test
```

## How the build works

`docker-bake.hcl` chains two Dockerfiles per variant:

1. The `base` stage of `base.Dockerfile` installs ROS on the variant's
   starting image. It's rebuilt only when its instructions change.
2. `perception.Dockerfile` defines `source` (`src/msgs` + `src/perception`)
   and `dependencies` (the base plus anything rosdep can't express).
3. The remaining stages of `base.Dockerfile` take those two stages, which bake
   passes in as the named contexts `module-source` and `module-deps`:
   - `rosdep_scan` resolves every `package.xml` into apt and pip package lists.
   - `dependencies` installs the lists. Docker caches this layer until the
     lists change, so editing code never reinstalls packages.
   - `build` runs `colcon build` into `/opt/ap1`.
   - `deploy` copies only `/opt/ap1` on top of the dependencies, with no sources
     or build tree. This image runs on the car.
   - `develop` is `dependencies` plus dev tools, with no prebuilt workspace:
     you build the mounted source in `/ws`. Its `ubuntu` user takes your
     UID/GID, so files created in mounted folders stay yours.

Dev build output lives in named volumes, one set per variant
(`ap1_perception-build-cpu`, `...-gpu`, and the same for `install` and `log`),
so switching `AP1_VARIANT` never reuses a build compiled against the other
image. `docker compose --profile '*' down` keeps them; delete them with
`docker volume rm` to start clean.

Deploy containers restart after a crash or reboot until stopped with
`docker compose --profile '*' down`. Dev containers don't.

BuildKit only builds the stages a target needs, so building `base-<variant>`
(target `base`) never touches the module stages, and the module targets
(`deploy`, `develop`) start from the module contexts instead of `base`.

New modules add a `<module>.Dockerfile`, targets in `docker-bake.hcl`, and a
service in each `modules/` compose file. They reuse `base.Dockerfile` as is.

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
