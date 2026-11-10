# AP1 Docker

Containerized build and dev environment for AP1 modules, modelled on
[WATonomous' wato_monorepo](https://github.com/WATonomous/wato_monorepo).
Perception and control run on it. Other modules still use `ap1_setup.sh`.

Perception images come in two variants built from the same files:

| Variant | Base image | onnxruntime | For |
|---|---|---|---|
| `cpu` | `ubuntu:24.04` | `onnxruntime` | anyone, no GPU needed (image ~2.9 GB) |
| `gpu` | `nvidia/cuda:13.0.3-cudnn-runtime-ubuntu24.04` | `onnxruntime-gpu` | NVIDIA GPU machines (image ~8.3 GB) |

The code is identical in both. Only the installed packages differ, and the
YOLO node's `device` parameter picks the hardware. Control has no GPU code, so
it only has a CPU image (see [Control](#control)).

## Setup

1. Docker with Buildx and Compose (2.24 or newer).
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
3. [Git LFS](https://git-lfs.com), then `git lfs pull` in the repo. Models live in
   `src/perception/models/` (LFS) and the folder is mounted read-only at
   `/opt/ap1/models`, so a model can be swapped without rebuilding. If
   `yolo26n.onnx` is a small text pointer instead of ~10 MB, run `git lfs pull`.
4. Create a `.env` file in the repo root (it's gitignored) with this content:
   ```bash
   # Required: the compose files, so commands need no -f flags
   COMPOSE_FILE=modules/docker-compose.yaml:modules/docker-compose.dev.yaml

   # Optional, see Settings
   # ROS_DOMAIN_ID=0
   # ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
   ```
   Run `docker compose` from the repo root, since that's the only place it reads `.env` from.

### Windows (WSL 2)

Run everything, including the repo and Docker, inside an Ubuntu WSL distro. It
then behaves like an Ubuntu machine.

1. (if you do not have an Ubuntu 24.04 install) In an admin PowerShell: `wsl --install -d Ubuntu-24.04`,
   reboot, and open Ubuntu from the Start menu.
2. Clone the repo inside WSL (e.g. `~/code/ap1`), not under `/mnt/c`.
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

- Every command below works with the CPU variant (`-cpu` targets and services).
  There's no GPU variant on macOS: `-gpu` services and `device:=cuda` need an
  NVIDIA GPU.
- Keep the repo under your home folder. Docker Desktop shares `/Users` with
  containers by default.
- Host networking means the VM's network, not your Mac's. Containers reach each
  other fine (dev containers, tests), but ROS discovery doesn't reach
  other machines, so `SUBNET` won't find the car or teammates. Use a Linux
  machine for that.

## Everyday use

Images are built with `docker buildx bake` and run with `docker compose` from
the repo root (your `.env` file from [Setup](#setup) step 4 points
`docker compose` at `modules/`, and service names match the `bake` targets):

```bash
docker buildx bake -f docker/docker-bake.hcl --load perception-dev-cpu
docker compose up -d perception-dev-cpu
docker compose exec perception-dev-cpu bash
```

Inside the container, `src/msgs` and `src/perception` from your checkout are
mounted at `/ws/src`, so edits on the host show up immediately. The dev image
has no prebuilt copy of them: build both in the container, from `/ws`:

```bash
cd /ws
colcon build --symlink-install     # builds into /ws/install (kept across restarts)
source install/setup.bash
```

Always run `colcon build` from `/ws`. Run from a package folder like
`/ws/src/perception`, it writes `build/`, `install/` and `log/` into your
checkout on the host, and later builds from `/ws` then find the package twice.

Try launching the YOLO node from the `ap1_perception` ROS package:

```bash
ros2 launch ap1_perception yolo.launch.py
```

In the dev container the node loads the model installed with the package from
your mounted source, and logs the path at startup. Pass
`model_path:=/path/to/model.onnx` to try another one; the deploy services pass
the mounted `/opt/ap1/models/yolo26n.onnx`.

The deploy image runs the YOLO node on its own.
In another terminal, try:

```bash
docker buildx bake -f docker/docker-bake.hcl --load perception-cpu # build the perception cpu image
docker compose up -d perception-cpu        # start the perception cpu service, which starts the YOLO node
docker compose logs -f perception-cpu      # follow its logs
docker compose --profile '*' down          # stop all AP1 containers (keeps dev build volumes)
```

After changing code or Dockerfiles, rebuild with bake,
then `docker compose up -d` will recreate the container from the new
image. If an image isn't built locally, compose pulls it from the registry.

### Tests

Tests live in each package's `test/` folder and run with `colcon test`.
`test/deploy/` holds the subset that also ships in the deploy image, to check
the exact image that runs on the car:

- `test_perception_environment.py`: dependencies import and work together
  (cv_bridge with apt's numpy, CUDA onnxruntime on GPU images), YOLO inference.
- `test_yolo_node.py`: launches `yolo_node`, feeds it a stop-sign photo with
  camera info and a point cloud over ROS, and checks the published entity. The
  photo is downloaded to a temporary folder and deleted once loaded, so this
  test needs internet.

The other tests in `test/` are unit tests that only run in dev containers.

The tests launch their own nodes in a ROS domain no other test run uses, so
they never talk to running containers or to a teammate's nodes. They run on
the GPU when the image is a GPU image and a GPU is visible, and on CPU otherwise.

In a dev container, from `/ws` after `colcon build`:

```bash
colcon test --packages-select ap1_perception
colcon test-result --verbose
```

Against a deploy image, through the deploy service so the model is mounted:

```bash
docker compose run --rm perception-cpu pytest /opt/ap1/share/ap1_perception/test/deploy
```

Use `docker compose run`, not `exec`: `exec` skips the image's entrypoint, so ROS
isn't sourced. The same commands work in the all-modules containers
(`all-dev-cpu` and `all-dev-gpu`, `all-perception`).

### GPU

For the GPU variant, use the `-gpu` targets and service names (`perception-gpu`,
`perception-dev-gpu`) in place of `-cpu`:

```bash
docker buildx bake -f docker/docker-bake.hcl --load perception-dev-gpu
docker compose up -d perception-dev-gpu
docker compose exec perception-dev-gpu bash
```

The GPU image sets `AP1_GPU=1`, which makes `yolo.launch.py` default to
`device:=cuda` (failing at startup if the GPU isn't usable, rather than silently
running on CPU). `device:=auto` uses the GPU when available.

### Control

Control is CPU only, so its targets and services have no `-cpu`/`-gpu` suffix:
`control` (deploy) and `control-dev`. The dev container mounts `src/msgs` and
`src/control`:

```bash
docker buildx bake -f docker/docker-bake.hcl --load control-dev
docker compose up -d control-dev
docker compose exec control-dev bash
```

Inside, build from `/ws` and run the node. `ros2 run` takes the package name
(`ap1_control`) and the executable name (`control_node`), then the path to the
config CSV:

```bash
cd /ws
colcon build --symlink-install
source install/setup.bash
ros2 run ap1_control control_node /ws/src/control/config/control_node_cfg.csv
```

The deploy service runs the node with the CSV installed in the image
(`/opt/ap1/share/ap1_control/config/control_node_cfg.csv`), so rebuild the
`control` image after changing the config:

```bash
docker buildx bake -f docker/docker-bake.hcl --load control
docker compose up -d control
docker compose logs -f control
```

It logs `Velocity is null or nan. Skipping update.` every 2 seconds until
something publishes `/ap1/actuation/speed`. That's expected.

### All modules

The `all` images hold every Docker module (perception and control for now) in
one image, built from the same commit with one shared `ap1_msgs`.

For development, `all-dev-cpu` and `all-dev-gpu` are single containers with the
sources of `src/msgs`, `src/perception` and `src/control` mounted at `/ws/src`.
Build and run everything from one shell:

```bash
docker buildx bake -f docker/docker-bake.hcl --load all-dev-cpu
docker compose up -d all-dev-cpu
docker compose exec all-dev-cpu bash
```

Inside, `colcon build` from `/ws` builds all three packages, then any `ros2`
command works as in the module dev containers.

For deployment, `all-deploy` is a GPU image (`ap1/all:gpu`), like the car. The
`all-deploy` profile runs one container per module from it, `all-perception` and
`all-control`, with the same commands and mounts as the module services. Each
restarts and logs on its own:

```bash
docker buildx bake -f docker/docker-bake.hcl --load all-deploy
docker compose --profile all-deploy up -d
docker compose logs -f all-perception
```

Don't run the `all-deploy` profile alongside the module deploy services
(`perception-gpu`, `control`): every node would run twice.

### Settings

Compose reads these from your shell or from your `.env` file. Bake reads only
from your shell, so set `AP1_REGISTRY` there if you change it. All are optional:

| Variable | Default | What it sets |
|---|---|---|
| `AP1_REGISTRY` | `ghcr.io/we-autopilot/ap1` | image name prefix |
| `AP1_TAG_SUFFIX` | empty | appended to the deploy image tag, e.g. `-<git sha>` to run an exact CI-built image |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | how far ROS discovery reaches |
| `ROS_DOMAIN_ID` | `0` | ROS domain. Must match any native nodes you talk to |

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

Both `deploy` and `develop` containers run as Ubuntu 24.04's built-in `ubuntu`
user (`1000:1000`), so files created in mounted folders belong to the standard
Linux host user (`1000:1000`) and Fast DDS shared memory works across containers
and host tools. If you previously ran containers as root, remove their leftover
shared-memory files once with
`sudo rm /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` while no ROS nodes are running.

## Adding a dependency

1. Declare it in the module's `package.xml` (e.g. `src/perception/package.xml`) with its
   [rosdep key](https://github.com/ros/rosdistro/tree/master/rosdep). Prefer apt
   keys (`python3-...`). Keys ending in `-pip` install from PyPI.
2. Pin any pip package in `config/pip-constraints.txt`. rosdep can't pin
   versions, so this file is what keeps builds reproducible.
3. Something rosdep can't express at all goes in the `dependencies` stage of
   `<module>.Dockerfile`. This is a last resort. Install it with `RUN`; don't
   `COPY` files from the repo there (see [How the build works](#how-the-build-works)).

If a dependency differs between CPU and GPU, use a condition on `AP1_GPU`, like
the onnxruntime entries in `package.xml`.

Never pip-install `numpy`: ROS's `cv_bridge` is built against Ubuntu's numpy
(1.26). The constraints file blocks upgrades, and the deploy tests check it.

## Layout

```
.env                          yours, not tracked: points docker compose at modules/ (Setup step 4)
.dockerignore                 keeps bags and build output out of images
bags/                         recordings, mounted at /ws/bags (contents gitignored)
modules/                      how modules run (CPU and GPU services per module, CPU only for control)
  docker-compose.yaml         deploy services: mounts, networking, launch commands
  docker-compose.dev.yaml     dev services: extend deploy, mount source, idle
docker/                       how images are built
  base.Dockerfile             ROS Jazzy base, plus the shared pipeline:
                              rosdep scan -> install -> build -> deploy/develop
  perception.Dockerfile       perception's sources and extra dependencies
  control.Dockerfile          control's sources and extra dependencies
  all.Dockerfile              merges every module's sources for the all-modules images
  docker-bake.hcl             build graph: variants x targets (used locally and by CI)
  config/                     entrypoint, shell env, pip version pins, pyproject mask
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
     you build the mounted source in `/ws`.

Only `source` stages may `COPY` from the repo. The `base` stage and the module
`dependencies` stages are passed between bake targets as contexts, and a
`COPY` from the build context in one of them breaks the cache of everything
built on it: every CI build would reinstall all packages. That's why the
entrypoint and shell setup are copied in `base.Dockerfile`'s `dependencies`
stage, after the install.

Dev build output lives in named volumes, one set per variant
(`ap1_perception-build-cpu`, `...-gpu`, and the same for `install` and `log`),
so CPU and GPU dev containers never reuse a build compiled against the other
image. Control has one set (`ap1_control-build` and so on), and the all-modules
dev containers one per variant (`ap1_all-build-cpu`, ...).
`docker compose --profile '*' down` keeps them; delete them with
`docker volume rm` to start clean.

Deploy containers restart after a crash or reboot until stopped with
`docker compose --profile '*' down`. Dev containers don't.

BuildKit only builds the stages a target needs, so building `base-<variant>`
(target `base`) never touches the module stages, and the module targets
(`deploy`, `develop`) start from the module contexts instead of `base`.

The all-modules images go through the same pipeline, with the module stages
combined first:

- `all-source-<variant>` (`all.Dockerfile`) copies every module's `source`
  stage into one tree, so `rosdep_scan` and `build` see all packages.
- `all-deps-<variant>` stacks the modules' `dependencies` stages: each module's
  stage is built with the previous module's as its `base`. They can't be merged
  with `COPY` like sources, since a `dependencies` stage can install packages.

Both are built per variant, so `rosdep_scan` resolves the `AP1_GPU` conditions in
`package.xml` for the right image.

New modules add a `<module>.Dockerfile`, targets in `docker-bake.hcl`, and a
service in each `modules/` compose file. They reuse `base.Dockerfile` as is. To
include a module in the all-modules images, also add a `COPY` line in
`all.Dockerfile`, its `source` target to `all-source`'s contexts, a step to the
`all-deps` stack, an `all-<module>` deploy service, and its source mount in the
`all-dev-*` services.

## CI

Two workflows build the deploy images on every PR that touches their module,
`docker/` or `src/msgs/`:

- `.github/workflows/perception-images.yml` builds both perception variants
  and runs the deploy tests in each. The runners have no GPU, so the GPU
  image's inference runs on CPU there; CI only checks that its onnxruntime has
  CUDA support.
- `.github/workflows/control-images.yml` builds the control image and checks
  that `control_node` starts with its installed config and stays up.

On `main` they push `ghcr.io/we-autopilot/ap1/{perception,control}` images plus
a per-commit tag. They also run on pushes to `development`, without pushing
images, so PRs into `development` start from a warm build cache.

The build cache lives in the GitHub Actions cache, one scope per image
(`perception-cpu`, `perception-gpu`, `control`; see `cache_from` and `cache_to`
in `docker-bake.hcl`). A PR can read its own cache and its base branch's. A
build only reinstalls packages when a `package.xml` or `pip-constraints.txt`
changes, as long as the rule in [How the build works](#how-the-build-works)
holds. The all-modules images aren't built in CI.

## Not covered yet

- **UFLD node.** It needs torch, which isn't in the image. Only the YOLO node
  is supported.
- **Live RealSense camera.** The container has no USB device access yet. Replay
  bags instead (see `SETUP.md` section 7; run it inside the dev container).
- **Jetson / arm64 images.** These would be a new entry in `VARIANTS` in
  `docker-bake.hcl` once the Orin's JetPack version is settled.
