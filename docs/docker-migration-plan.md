# AP1 Docker migration plan

Status: proposal. Follows the design review of `perception/docker-setup`.

Decisions this plan is built on:

- Docker is the AP1 environment. The native path (`ap1_setup.sh`, `SETUP.md`,
  uv) is being retired, not maintained in parallel.
- Middleware stays Fast DDS (no rmw_zenoh). The root/non-root shared-memory
  problem is solved with `FASTDDS_BUILTIN_TRANSPORTS`, not with host-UID
  containers.
- Bake stays. The HCL file is not the complexity; the three-Dockerfile chain is.
- The car's compute is assumed to be an NVIDIA Orin (arm64). Confirm this in
  Phase 0; it decides whether Phase 8 is required.

Each phase lists the files it touches and what "done" means. Phases 1-4 can
land as separate PRs on `perception/docker-setup`; 5-8 are independent of each
other once 1-4 are in.

---

## 0. End state

```text
ap1d                           thin wrapper: build | up | shell | test | pull | down | clean
.devcontainer/devcontainer.json  VS Code / IDE entry point (uses modules/docker-compose.dev.yaml)
docker/
  base.Dockerfile              ROS Jazzy on the variant's image; `runtime` and `devel` stages
  module.Dockerfile            ONE pipeline for every module (ARG MODULE)
  docker-bake.hcl              matrix = MODULES x VARIANTS; constant size
  config/                      entrypoint, env, pip-constraints.txt, fastdds profile (if needed)
  modules/<module>/            optional per-module extras: deps.sh, pip-extras.txt
  tests/<module>/              test scripts + vendored fixtures (LFS)
modules/
  docker-compose.yaml          deploy services (one per module), root, host net/ipc
  docker-compose.dev.yaml      dev services: extend deploy, mount src/<module> and src/msgs
  docker-compose.test.yaml     test runners (smoke, e2e) and replay sources
  docker-compose.gpu.yaml      GPU access, applied with --gpu
src/
  msgs/ perception/ sensors/ control/ planning/ localization/ mapping/ console/ bringup/
```

Images per module and variant, all from the same `module.Dockerfile`:

| Tag | Stage | Contents |
|---|---|---|
| `<module>:<variant>-<sha>` | `deploy` | runtime base + rosdep deps + `/opt/ap1` install tree. Immutable. |
| `<module>:<variant>` | `deploy` | moving tag, retagged from the sha tag after tests pass on `main` |
| `<module>:<variant>-test-<sha>` | `test` | deploy + test scripts + fixtures. CI and `ap1d test` run this. |
| `<module>:<variant>-dev` | `develop` | deps + dev tools; built locally only, never pushed |
| `base:<variant>` | `devel` | pushed from `main`; PRs pull it instead of rebuilding ROS |

Runtime rules (apply to every service in `modules/docker-compose.yaml`):

- Containers run as root. No `user:`, no `ROS_HOME` override, no `userdel ubuntu`.
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` while any ROS node still runs natively on a
  host. Once every node is in a root container with `ipc: host`, measure and
  decide whether to flip back to `DEFAULT` (SHM) for large point clouds.
- `network_mode: host`, `ipc: host`, `init: true`, `restart: unless-stopped` on
  deploy services (not on dev/test services).
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` default, `SUBNET` on the car.

---

## Phase 0. Decisions and guardrails

Scope: documentation only, no behaviour change.

1. Record in `docker/README.md` (short "Decisions" section): Fast DDS + UDPv4,
   root containers, CUDA major (13), Jazzy/24.04, Bake as the build definition,
   Docker as the only supported environment with a sunset date for the native
   path.
2. Confirm the car's compute (Orin? JetPack version? driver's CUDA version via
   `nvidia-smi`). This decides Phase 8 and whether CUDA 13 is viable on the car
   today (JetPack 7.2 brings Orin to Ubuntu 24.04 / CUDA 13.2; JetPack 6 is
   22.04 / CUDA 12.6 and would force a different plan).
3. Inventory native-only behaviour that must survive in containers: LIO-SAM
   patching in `ap1_setup.sh`, PyQt6 console (needs X11), RealSense USB.

Done: README section merged; Orin/JetPack/driver versions written down.

---

## Phase 1. Runtime simplification (small, safe, do first)

Fixes the parts that cause support tickets, with no change to the build graph.

Files: `modules/docker-compose.yaml`, `modules/docker-compose.dev.yaml`,
`docker/template.Dockerfile` (develop stage), `ap1d`, `docker/README.md`.

1. `modules/docker-compose.yaml`
   - Remove `user:` and `ROS_HOME` from `x-ros`. Add
     `FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}`.
   - Add `restart: unless-stopped` to deploy services only.
   - Pin images with a tag variable: `image: ${AP1_REGISTRY}/perception:${AP1_VARIANT}${AP1_TAG_SUFFIX:-}`
     where `ap1d` sets `AP1_TAG_SUFFIX=-<sha>` for `pull`/car use and empty for
     local builds (see Phase 3).
2. `docker/template.Dockerfile`
   - `develop` becomes `FROM dependencies` (not `deploy`), so the dev container
     has no second copy of the workspace in `/opt/ap1`. The dev workflow is
     "mount `src/`, `colcon build` in `/ws`", which is what the README already
     says.
   - Keep the host-UID user in `develop` only for file ownership of bind mounts;
     stop deleting the `ubuntu` user (use `usermod -u`/`groupmod -g` on it
     instead, which is what devcontainers do). Phase 2 replaces this with
     `.devcontainer.json`'s `updateRemoteUserUID` anyway.
3. `modules/docker-compose.dev.yaml`
   - Mount `../src/msgs:/ws/src/msgs` as well as `src/perception`, so message
     changes are built in the overlay, not baked into the image.
   - Name volumes with the variant in them (`perception-build-${AP1_VARIANT}`),
     so switching `--gpu`/`--cpu` does not reuse a build tree compiled against
     the other image.
4. `ap1d`
   - Add `clean [module]`: `compose down -v` for the module's named volumes, with
     a one-line explanation of when to use it (image rebuilt, deps changed).
   - Remove `AP1_UID/AP1_GID` exports for deploy; keep them for the dev build only.
5. `ap1_env.sh`: unchanged. The overlay source order already matches the new layout.

Done: `./ap1d up perception` runs as root and exchanges data with a native node
run by a normal user on the same machine; `./ap1d up perception --dev` +
`colcon build` in `/ws` works with no duplicate `ap1_perception` in
`ros2 pkg prefix`; `./ap1d clean perception` exists.

---

## Phase 2. Collapse the build graph

Replace `perception.Dockerfile` + `template.Dockerfile` + per-module bake
targets with one `module.Dockerfile` and a two-axis matrix. The Docker Bake
reference itself recommends regular multi-stage builds over `target:` contexts
when the Dockerfiles can be merged; ours can.

Files: `docker/module.Dockerfile` (new), `docker/base.Dockerfile`,
`docker/docker-bake.hcl`, `docker/modules/perception/` (new, optional),
`.dockerignore`, delete `docker/perception.Dockerfile` and
`docker/template.Dockerfile`.

### 2.1 `base.Dockerfile`: two stages

```dockerfile
ARG GENERIC_IMAGE=ubuntu:24.04
FROM ${GENERIC_IMAGE} AS runtime
# ros-<distro>-ros-base, python3-pip, ca-certificates, tzdata, ap1_env.sh, entrypoint. No compilers.

FROM runtime AS devel
# build-essential, git, colcon, rosdep (+ rosdep init/update), vcstool
```

`deploy` images derive from `base-runtime`; `source`/`rosdep_scan` derive from
`base-devel` (they need rosdep and its cache). The `build` and `develop` stages
derive from `dependencies` (runtime + deps) and install the toolchain
themselves in one cached layer, so compilers never reach `deploy`. This removes
roughly 400 MB from the car image and is the only reason `base` stays a
separate Dockerfile (it is pushed and reused across modules).

### 2.2 `module.Dockerfile`

```dockerfile
ARG MODULE
FROM base-devel AS source
WORKDIR ${AP1_WS}/src
COPY src/msgs msgs
COPY src/${MODULE} ${MODULE}
# modules that vendor external repos (localization: LIO-SAM) do `vcs import` here,
# before the COPY of local sources, so the clone layer is cached

FROM source AS rosdep_scan
# no `rosdep update` here: the cache is in base-devel
RUN rosdep keys --from-paths src --ignore-src \
      --dependency-types build buildtool build_export buildtool_export exec \
    | sort -u > /tmp/keys && ... (same awk as today) -> /tmp/apt_packages /tmp/pip_packages

FROM base-runtime AS dependencies
COPY --from=rosdep_scan /tmp/apt_packages /tmp/pip_packages /tmp/
COPY docker/config/pip-constraints.txt /tmp/
COPY docker/modules/${MODULE}/ /tmp/module/        # may be empty; deps.sh and pip-extras.txt are optional
RUN apt-get update && xargs -r -a /tmp/apt_packages apt-get install -y --no-install-recommends && \
    xargs -r -a /tmp/pip_packages pip install --no-cache-dir -c /tmp/pip-constraints.txt && \
    if [ -f /tmp/module/pip-extras.txt ]; then pip install --no-cache-dir -c /tmp/pip-constraints.txt -r /tmp/module/pip-extras.txt; fi && \
    if [ -x /tmp/module/deps.sh ]; then /tmp/module/deps.sh; fi && \
    rm -rf /var/lib/apt/lists/* /tmp/module /tmp/*_packages /tmp/pip-constraints.txt

FROM dependencies AS build
# toolchain lives here and in develop only; one cached layer, never reaches deploy
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
COPY --from=source ${AP1_WS}/src ${AP1_WS}/src
RUN colcon build --merge-install --install-base ${AP1_INSTALL} ...

FROM build AS test
# unit tests run here in CI: colcon test + colcon test-result. Sources and build tree present.
COPY docker/tests/${MODULE}/ /tests/

FROM dependencies AS deploy
COPY --from=build ${AP1_INSTALL} ${AP1_INSTALL}

FROM dependencies AS develop
# same toolchain line as build, plus gdb/tmux/vim/sudo and a non-root user whose
# UID is fixed at runtime (devcontainer updateRemoteUserUID, or ap1d via usermod)
```

Notes:

- `rosdep_scan` still re-runs on every source change (it is `FROM source`), but
  it is now pure CPU and offline; the expensive `dependencies` layer is still
  keyed only on the two package lists.
- `--dependency-types` drops `test` so `ament_flake8`, `pytest`, etc. stay out of
  deploy images.
- Models: stop excluding `**/*.onnx` in `.dockerignore`. `yolo11n.onnx` is 10 MB
  and already in LFS; let `setup.py`'s `package_data` ship it so
  `model_path` can default to the bundled model. Keep `bags/` excluded.
  Revisit if a model exceeds ~100 MB (then: OCI artifact in GHCR pulled at
  build, or a `models` volume filled by `ap1d pull`).

### 2.3 `docker-bake.hcl`: constant size

```hcl
variable "MODULES"  { default = ["perception"] }           # Phase 5 adds "sensors", Phase 7 the rest
variable "VARIANTS" { default = [
  { name = "cpu", image = "ubuntu:24.04", gpu = "0" },
  { name = "gpu", image = "nvidia/cuda:13.0.3-cudnn-runtime-ubuntu24.04", gpu = "1" },
]}
variable "REGISTRY" { default = "ghcr.io/we-autopilot/ap1" }
variable "GIT_SHA"  { default = "" }
variable "CACHE"    { default = "" }   # "registry" in CI, "" locally

function "cache" { params = [name, mode]
  result = CACHE == "registry" ? ["type=registry,ref=${REGISTRY}/cache:${name}${mode == "to" ? ",mode=max" : ""}"] : [] }

# base: variants x stages -> base-runtime-cpu, base-devel-cpu, base-runtime-gpu, base-devel-gpu
target "base" {
  matrix = { v = VARIANTS, s = ["runtime", "devel"] }
  name       = "base-${s}-${v.name}"
  dockerfile = "docker/base.Dockerfile"
  target     = s
  args       = { GENERIC_IMAGE = v.image, AP1_GPU = v.gpu }
  tags       = ["${REGISTRY}/base:${s}-${v.name}"]
  cache-from = cache("base-${s}-${v.name}", "from")
  cache-to   = cache("base-${s}-${v.name}", "to")
}

# shared, non-matrix attributes (Bake's documented inheritance pattern)
target "_module" {
  dockerfile = "docker/module.Dockerfile"
}

# modules x variants -> perception-cpu, perception-gpu, ...
target "module" {
  inherits = ["_module"]
  matrix   = { m = MODULES, v = VARIANTS }
  name     = "${m}-${v.name}"
  target   = "deploy"
  contexts = { "base-devel" = "target:base-devel-${v.name}", "base-runtime" = "target:base-runtime-${v.name}" }
  args     = { MODULE = m }
  tags     = compact(["${REGISTRY}/${m}:${v.name}", GIT_SHA != "" ? "${REGISTRY}/${m}:${v.name}-${GIT_SHA}" : ""])
  cache-from = cache("${m}-${v.name}", "from")
  cache-to   = cache("${m}-${v.name}", "to")
}

# same matrix, different stage. Matrix blocks cannot be inherited, so the matrix,
# contexts and args repeat; everything else comes from _module.
target "module-test" {
  inherits = ["_module"]
  matrix   = { m = MODULES, v = VARIANTS }
  name     = "${m}-test-${v.name}"
  target   = "test"
  contexts = { "base-devel" = "target:base-devel-${v.name}", "base-runtime" = "target:base-runtime-${v.name}" }
  args     = { MODULE = m }
  tags     = ["${REGISTRY}/${m}:${v.name}-test${GIT_SHA != "" ? "-${GIT_SHA}" : ""}"]
  cache-from = cache("${m}-${v.name}", "from")   # shares deploy's cache; test adds one layer
}

target "module-dev" {
  inherits = ["_module"]
  matrix   = { m = MODULES, v = VARIANTS }
  name     = "${m}-dev-${v.name}"
  target   = "develop"
  contexts = { "base-devel" = "target:base-devel-${v.name}", "base-runtime" = "target:base-runtime-${v.name}" }
  args     = { MODULE = m }
  tags     = ["${REGISTRY}/${m}:${v.name}-dev"]
  cache-from = cache("${m}-${v.name}", "from")
}
```

Bake builds `base-*` once per invocation and shares it between the module
targets that reference it, so `bake perception-cpu sensors-cpu` builds ROS once.
Verify the graph with `docker buildx bake --print perception-test-gpu` before
wiring CI to it.

Adding a module after this phase = one entry in `MODULES`, one compose
service, optionally `docker/modules/<module>/`. No new bake blocks.

### 2.4 `ap1d`

`build` calls `docker buildx bake --load <module>-<variant>`; module names are
validated against `docker buildx bake --list=targets` rather than a hard-coded
array.

Done: `docker buildx bake --print perception-gpu` shows a single
`module.Dockerfile` build with `MODULE=perception`; CPU and GPU deploy images
build; the deploy image has no `gcc`; `template.Dockerfile` and
`perception.Dockerfile` are deleted; README "How the build works" rewritten.

---

## Phase 3. Registry, CI, and the pull path

Make the per-commit image the thing that gets deployed, and stop rebuilding ROS
on every CI run.

Files: `.github/workflows/perception-images.yml` -> `images.yml` (one workflow
for all modules, matrix over module x variant), `ap1d`,
`modules/docker-compose.yaml`.

1. CI job per (module, variant):
   1. `docker/setup-buildx-action`, login to GHCR (needed on PRs too, for
      `cache-from`; use `GITHUB_TOKEN` with `packages: read`).
   2. `docker/bake-action` with `CACHE=registry`, `GIT_SHA=${{ github.sha }}`,
      targets `<module>-<variant>` and `<module>-test-<variant>`, `load: true`.
   3. Tests via compose (Phase 4), against `:<variant>-<sha>` and
      `:<variant>-test-<sha>` exactly as built.
   4. On `main` only: push the sha-tagged image, the test image, and the cache.
      Do not push the moving tag from the build job.
2. Promotion job (on `main`, after all matrix legs pass):
   `docker buildx imagetools create -t <module>:<variant> <module>:<variant>-<sha>`
   for every module/variant. Moving tags therefore only ever point at a digest
   that passed every test, and `cpu`/`gpu` can never be at different commits.
   Remove `cancel-in-progress` for `main` pushes (keep it for PRs).
3. `base-<variant>` is pushed from `main` as well; PR builds get it from the
   registry cache. GHA cache (`type=gha`) is dropped entirely: the 10 GB per-repo
   limit cannot hold two multi-GB graphs in `mode=max`.
4. `ap1d pull <module> [--sha <sha>]`: `docker pull` the sha tag (default: the
   sha of `origin/main`), then `up` without building. This is what the car runs.
   `ap1d up` without `--build` uses whatever tag `AP1_TAG_SUFFIX` resolves to;
   `ap1d up --build` does today's build-then-up.
5. Car runbook (new section in `docker/README.md`): `git fetch`,
   `./ap1d pull perception sensors --gpu`, `./ap1d up perception sensors --gpu`;
   containers restart on boot because of `restart: unless-stopped` and the
   Docker service. Rollback = `ap1d pull --sha <previous>`.

Done: a PR builds perception in under the time of a cold ROS install because
`base` is pulled from cache; `main` publishes `perception:cpu-<sha>` and the
promotion job moves `perception:cpu`; `./ap1d pull perception` on a clean
machine starts the node without building anything.

---

## Phase 4. Tests as compose services on built images

Principle: a test never builds anything and never reaches the network. It runs
a published image (`<module>:<variant>-test-<sha>`) against fixtures that are in
that image. The same compose file runs in CI and from `ap1d test`.

Files: `modules/docker-compose.test.yaml` (new), `docker/tests/<module>/`,
`ap1d`, CI workflow.

### 4.1 Three layers

| Layer | Where it runs | What it proves | Invocation |
|---|---|---|---|
| Unit (`colcon test`) | `test` stage, during `bake` or as a compose service with `network_mode: none` | package code | `docker compose run --rm perception-unit` |
| Image smoke | `test` image, `network_mode: none` | deps import, cv_bridge/numpy ABI, provider list, node executable, one inference | `docker compose run --rm perception-smoke` |
| Integration (e2e) | deploy image under test + a feeder/replay service + a checker service | ROS graph behaviour | `docker compose --profile perception --profile e2e up --exit-code-from perception-e2e-check` |

### 4.2 `modules/docker-compose.test.yaml` (shape)

```yaml
x-test: &test
  init: true
  network_mode: none          # offline by construction; e2e services override to host

services:
  perception-smoke:
    <<: *test
    profiles: [test]
    image: ${AP1_REGISTRY}/perception:${AP1_VARIANT}-test${AP1_TAG_SUFFIX:-}
    command: python3 /tests/perception_smoke.py --device ${AP1_DEVICE}

  perception-unit:
    <<: *test
    profiles: [test]
    image: ${AP1_REGISTRY}/perception:${AP1_VARIANT}-test${AP1_TAG_SUFFIX:-}
    working_dir: /ws
    command: bash -c "colcon test --merge-install --install-base /opt/ap1 --packages-select ap1_perception && colcon test-result --verbose"

  # e2e: the module under test is the plain deploy service from docker-compose.yaml.
  perception-e2e-feed:                       # plays the sensor role; Phase 5 swaps this for sensors-replay
    extends: { file: docker-compose.yaml, service: perception }   # inherits host net/ipc/env
    profiles: [e2e]
    image: ${AP1_REGISTRY}/perception:${AP1_VARIANT}-test${AP1_TAG_SUFFIX:-}
    restart: "no"
    healthcheck: { disable: true }
    command: python3 /tests/perception_e2e_feed.py --image /tests/fixtures/stop_sign.jpg
    depends_on: { perception: { condition: service_healthy } }
  perception-e2e-check:
    extends: { file: docker-compose.yaml, service: perception }
    profiles: [e2e]
    image: ${AP1_REGISTRY}/perception:${AP1_VARIANT}-test${AP1_TAG_SUFFIX:-}
    restart: "no"
    healthcheck: { disable: true }
    command: python3 /tests/perception_e2e_check.py --expect-depth 5.0 --messages 5
    depends_on: { perception-e2e-feed: { condition: service_started } }
```

And in `docker-compose.yaml`, deploy services get a health check so e2e waits
for the node rather than sleeping. It must run through the entrypoint:
`/etc/bash.bashrc` returns early for non-interactive shells, so a plain
`bash -c` would not have ROS sourced.

```yaml
healthcheck:
  test: ["CMD", "/usr/local/bin/ap1_entrypoint.sh", "bash", "-c", "ros2 node list | grep -q /yolo_node"]
  interval: 5s
  timeout: 10s
  start_period: 30s
  retries: 12
```

Each check spawns a short-lived DDS participant; at a 5-10 s interval that is
negligible, but do not go below that.

### 4.3 Fixtures

- `docker/tests/perception/fixtures/stop_sign.jpg` vendored in LFS (replaces
  the runtime download from cocodataset.org). Checksums live in the test, not in
  the network.
- Replay data for integration tests: one short RealSense `.bag` (seconds, not
  minutes) in LFS or as a GitHub release asset with a sha256 recorded in the
  test image. The test image `COPY`s it in; nothing is mounted at test time.
- Models: in the image after Phase 2.

### 4.4 Wiring

- `ap1d test <module>`: build (or pull) the test image, then
  `compose -f ... -f docker-compose.test.yaml --profile test run --rm <module>-smoke`
  and `<module>-unit`.
- `ap1d e2e <module>`: `compose --profile e2e up --exit-code-from <module>-e2e-check ...`,
  then `compose --profile e2e down`.
- CI runs exactly those two `ap1d` commands with `AP1_TAG_SUFFIX=-<sha>`.
  Hosted runners have no GPU: the gpu variant runs smoke/unit with
  `--device cpu` and the e2e is skipped for `gpu` unless a self-hosted GPU
  runner label is present (`runs-on: [self-hosted, gpu]` behind an `if:`).
  Until that runner exists, the README states plainly that the GPU image's
  CUDA path is only verified by `ap1d test perception --gpu` on a GPU machine.

Done: `docker/tests/perception_e2e.py` is split into feed/check services;
`./ap1d test perception` runs with the network disabled; CI runs the same
compose commands; no test downloads anything.

---

## Phase 5. Sensors module (RealSense)

A `sensors` module owns hardware drivers and their bring-up. Perception consumes
topics and no longer `exec_depend`s on `realsense2_camera`.

### 5.1 Package: `src/sensors/ap1_sensors` (ament_python or ament_cmake, launch-only)

- `package.xml`: `<exec_depend>realsense2_camera</exec_depend>`,
  `<exec_depend>tf2_ros</exec_depend>` (static camera -> base TF), later
  `rplidar_ros`/`velodyne` etc.
- `launch/realsense.launch.py`: wraps `rs_launch.py` with AP1's fixed
  parameters (`align_depth.enable`, `pointcloud.enable`, resolution, FPS,
  `camera_namespace`), publishes the static TF.
- `launch/replay.launch.py`: `rs_launch.py rosbag_filename:=<path> rosbag_loop:=<bool>`.
  Same package, same topics, no hardware.
- `config/`: the topic contract perception depends on
  (`/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/camera_info`,
  `/camera/camera/depth/color/points`) written down once, in one place.

### 5.2 Image

`sensors` goes into `MODULES`; the cross product builds `sensors:cpu` and
`sensors:gpu`. The gpu variant is only used if the camera host is the GPU box;
the extra build is cheap because `base-gpu` is cached. If that bothers people,
give the matrix a per-module variant list later; do not special-case it now.

The ROS-packaged `ros-jazzy-librealsense2` is container-friendly (no kernel
patch); verify in the container with `rs-enumerate-devices` during the first
bring-up and note which backend it uses.

### 5.3 Compose

```yaml
  sensors:
    <<: *ros
    profiles: [sensors]
    image: ${AP1_REGISTRY}/sensors:${AP1_VARIANT}${AP1_TAG_SUFFIX:-}
    volumes:
      - /dev:/dev                      # device nodes, incl. hot-plug re-enumeration
    device_cgroup_rules:
      - "c 81:* rmw"                   # video4linux
      - "c 189:* rmw"                  # usb_device
    # privileged: true                 # only if the IMU (HID) is needed; try without first
    command: ros2 launch ap1_sensors realsense.launch.py
    healthcheck:
      test: ["CMD", "/usr/local/bin/ap1_entrypoint.sh", "bash", "-c",
             "timeout 5 ros2 topic echo --once /camera/camera/color/image_raw > /dev/null"]
      interval: 10s
      start_period: 40s
      retries: 6

  sensors-replay:
    <<: *ros
    profiles: [replay]
    image: ${AP1_REGISTRY}/sensors:${AP1_VARIANT}${AP1_TAG_SUFFIX:-}
    volumes:
      - ../bags:/ws/bags:ro
    command: ros2 launch ap1_sensors replay.launch.py bag:=/ws/bags/${AP1_BAG:?set AP1_BAG} loop:=true
```

Host prerequisites (documented, and checked by a new `ap1d doctor`):

- RealSense udev rules on the host (`60-librealsense2-udev-rules.rules` from the
  `librealsense2-udev-rules` apt package, or Intel's `99-realsense-libusb.rules`).
  The container runs as root so it would work without them, but hot-plug and
  power-state handling are more reliable with them.
- `nvidia-container-toolkit` only on GPU hosts.

### 5.4 How perception and sensors interact

- On the car: `./ap1d up sensors perception --gpu`. Two containers, host
  networking, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (or `DEFAULT` once native nodes
  are gone).
- Developer without a camera: `AP1_BAG=lab_loop.bag ./ap1d up sensors --replay`
  then `./ap1d up perception --dev`. Replay replaces the synthetic feeder for
  most work.
- e2e in CI: `sensors-replay` with a short vendored bag becomes the feeder
  for `perception-e2e-check`; the synthetic `perception-e2e-feed` is kept as
  the fast, deterministic check (fixed depth wall).

Done: `sensors:cpu` image builds; `./ap1d up sensors` publishes camera topics
from a plugged-in D4xx; `--replay` publishes from a bag; perception's
`package.xml` no longer lists `realsense2_camera`; perception e2e runs against
`sensors-replay`.

---

## Phase 6. One source of truth for dependencies; retire uv

Goal: `package.xml` + `docker/config/pip-constraints.txt` (+ optional
`docker/modules/<module>/pip-extras.txt`) describe every dependency. The
native path keeps working until Phase 7 removes it, using the same files.

Steps, in order, each independently mergeable:

1. Move every runtime dependency from `src/perception/pyproject.toml` into
   `package.xml` with rosdep keys. Resolve each with
   `rosdep resolve <key> --os ubuntu:noble`; prefer apt (`python3-*`), fall
   back to `-pip` keys. Where no key exists, add it to
   `docker/modules/perception/pip-extras.txt` (and consider contributing the
   key to `ros/rosdistro`). Dev-only tools (`ipython`, `matplotlib`, `tqdm`)
   go to the `develop` stage or a `<test_depend>`, not to deploy.
   Torch (UFLD node): decide whether UFLD ships in the perception image; if
   yes, pin `torch` in `pip-constraints.txt` with the CUDA-13 index for the gpu
   variant via `pip-extras.txt`; if no, split UFLD out or mark it unsupported.
2. Pin transitive pip packages in `pip-constraints.txt` (generate once with
   `pip freeze` in the built image, keep only what pip installed). This is the
   lock file now; `uv.lock` is deleted.
3. Delete `src/perception/pyproject.toml`, `uv.lock`, `.python-version`. Delete
   `docker/config/pyproject-mask.toml` and its bind mount, and the `rm -f` in
   the source stage. `setup.py`/`setup.cfg` stay (ament_python needs them).
4. `src/perception/launch/yolo.launch.py`: remove `_find_venv_site_packages`
   and the `PYTHONPATH` manipulation; the interpreter is the system one in
   every supported environment.
5. `ap1_setup.sh` during the transition (so native users are not stranded
   before Phase 7):
   - Remove steps 5-6 (uv install, `uv sync`, venv activation, PYTHONPATH line).
   - Replace with: `rosdep install --from-paths src --ignore-src -y` (no
     `--skip-keys` for perception any more; rosdep installs the `-pip` keys
     itself) followed by `pip install --user --break-system-packages -c
     docker/config/pip-constraints.txt onnxruntime` so the native machine gets
     the same pinned versions as the image. Document that this is a temporary
     bridge until Phase 7.
   - Remove the PyQt6 and empy special cases if `package.xml` keys
     (`python3-pyqt6` now exists in rosdistro) and apt `python3-empy` cover them.
   - Print a deprecation notice pointing at `docker/README.md`.
6. `SETUP.md`: mark native setup as deprecated at the top, link to Docker docs,
   keep only the sections still true (bag replay, topic checks) and move those
   to `docker/README.md`.

Done: `grep -r uv src/ ap1_setup.sh SETUP.md` is empty; `rosdep check
--from-paths src --ignore-src` passes in the perception image with no skip
keys; `yolo.launch.py` has no venv logic; `ap1_setup.sh` installs perception
deps from `package.xml` + constraints.

---

## Phase 7. Remaining modules, then delete the native path

Order by difficulty, each as "add to `MODULES`, add compose service(s)":

1. `control`, `planning` (ament_cmake, deps already in `package.xml`). One image
   `pnc` or two; one container per launch file. Add `colcon test` to the test
   stage for C++ gtest.
2. `localization`/`mapping` with LIO-SAM: `vcs import` in the `source` stage
   from `ap1.repos`, apply the Eigen3/cv_bridge patches from `ap1_setup.sh` as
   a `docker/modules/localization/patch.sh` run right after the import (or
   fork LIO-SAM and pin the fixed commit, which is cleaner). GTSAM and NDT come
   from apt via rosdep as today.
3. `bringup`: either a `full-system` compose profile that starts every module's
   service (preferred; one process per container, restart policies per node) or
   a single container running `full_system.launch.py` (simplest, matches the
   native experience). Start with the profile; the launch file remains for
   in-container debugging.
4. `console` (PyQt6): needs X11/Wayland. `DISPLAY`, `/tmp/.X11-unix` mount,
   `xhost +local:` in `ap1d`, or run the console natively against the
   containerised stack (UDPv4 makes this trivial). Decide per team preference;
   do not block the rest on it.
5. When every launch file in `SETUP.md` has a compose profile: delete
   `ap1_setup.sh`, `SETUP.md`, `scripts/software_smoke_test.sh` (replace with a
   `full-system` e2e in `docker-compose.test.yaml`), `colcon.meta` (flags live
   in `module.Dockerfile`), and `.vscode` leftovers. Update `README.md`.

Done: `./ap1d up --profile full-system` runs the system; the repo has no
native setup instructions.

---

## Phase 8. GPU validation, CUDA-from-pip, and arm64

Independent of 5-7; do after Phase 3.

1. Self-hosted GPU runner (a lab machine with the NVIDIA toolkit) labelled
   `gpu`; the `gpu` matrix leg's smoke/e2e run with `--device cuda` there. This
   is the only way the CUDA path is tested before it reaches the car.
2. Spike: build the gpu variant `FROM ubuntu:24.04` with
   `onnxruntime-gpu[cuda,cudnn]` in `docker/modules/perception/pip-extras.txt`
   and `onnxruntime.preload_dlls()` in `_yolo.py`. If inference runs, drop the
   `nvidia/cuda` base: `VARIANTS` becomes two entries that differ only in
   `AP1_GPU`, the base image is shared, and the "CUDA must match the base image"
   constraint disappears (the host driver still needs CUDA 13 support, R580+).
3. arm64 (if Phase 0 confirms Orin on JetPack 7.x): add
   `platforms = ["linux/arm64"]` to a `jetson` variant with the Jetson CUDA
   base image, build on a native arm64 runner (GitHub's `ubuntu-24.04-arm` or
   the Orin itself as a self-hosted runner); emulated CUDA builds are not
   viable. Check that the aarch64 `onnxruntime-gpu` wheel works on Jetson
   (historically Jetson wheels came from NVIDIA's own index; if still true, put
   the index URL in `pip-extras.txt` for that variant). Tags become
   `<module>:jetson-<sha>`; the car runbook uses `--variant jetson`.

Done: GPU inference is exercised in CI; a decision is recorded on CUDA-from-pip;
if applicable, `perception:jetson-<sha>` runs on the Orin via `ap1d pull`.

---

## Risks and open questions

- CUDA 13 requires driver R580+ on every GPU host, including the car. If the
  car cannot be upgraded, pin `onnxruntime-gpu` to a CUDA 12.8 build and a
  `nvidia/cuda:12.8.*` base until it can.
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` disables shared memory. For a single
  camera at 30 FPS plus one point cloud this is fine; measure before adding
  more sensors, and flip to `DEFAULT` once nothing runs natively.
- `/dev:/dev` plus cgroup rules is the standard RealSense recipe but the IMU
  (HID) may need `privileged: true`. Try without first.
- Registry storage: the cache repo (`cache:*` tags with `mode=max`) grows;
  add a scheduled cleanup (GHCR has no retention policy UI yet; use the
  packages API or `actions/delete-package-versions`).
- GHCR visibility: packages under an org default to private; the car and lab
  machines need a PAT with `read:packages`, or the packages are made public.
  Decide in Phase 3.

---

## Learning resources

Build graphs, Bake, HCL

- Bake overview, matrices, contexts, variables, inheritance:
  https://docs.docker.com/build/bake/ (read "Matrix targets",
  "Using Bake with additional contexts", "Inheritance", and the full reference
  https://docs.docker.com/build/bake/reference). The reference also documents
  typed variables and how env overrides are coerced.
- Why Bake is GA and what changed: https://www.docker.com/blog/ga-launch-docker-bake/
- `docker/bake-action` README (inputs, `set`, `load`, `push`):
  https://github.com/docker/bake-action
- Compose's own Bake integration (how `docker compose build` maps to a bake
  definition; useful to understand what the HCL is replacing):
  https://docs.docker.com/compose/how-tos/dependent-images/

Dockerfiles, caching, multi-stage

- Multi-stage builds: https://docs.docker.com/build/building/multi-stage/
- Build cache and invalidation rules: https://docs.docker.com/build/cache/
- Cache backends (`registry`, `gha`, `inline`; what `mode=max` means):
  https://docs.docker.com/build/cache/backends/ and
  https://docs.docker.com/build/cache/backends/registry/
- Build drivers (why `type=gha`/`registry` cache needs `docker-container`):
  https://docs.docker.com/build/builders/drivers/
- Multi-platform builds (arm64 for the Orin):
  https://docs.docker.com/build/building/multi-platform/
- Dockerfile best practices: https://docs.docker.com/build/building/best-practices/
- Hadolint (lint the Dockerfiles in pre-commit): https://github.com/hadolint/hadolint

Registries, tags, digests, promotion

- What a registry is and how images are addressed:
  https://docs.docker.com/get-started/docker-concepts/the-basics/what-is-a-registry/
- The OCI distribution spec (the protocol GHCR, Docker Hub, ECR all speak):
  https://github.com/opencontainers/distribution-spec/blob/main/spec.md
- GHCR specifics (auth, visibility, `GITHUB_TOKEN` scopes):
  https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry
- `docker buildx imagetools create` (server-side retag/promotion by digest):
  https://docs.docker.com/reference/cli/docker/buildx/imagetools/create/
- `docker/metadata-action` (generates tags/labels from git refs, instead of
  hand-rolled `compact([...])`): https://github.com/docker/metadata-action
- `crane` and `skopeo` for inspecting/copying images without a daemon:
  https://github.com/google/go-containerregistry/tree/main/cmd/crane ,
  https://github.com/containers/skopeo

Compose for services and tests

- Compose file reference (`profiles`, `extends`, `depends_on` conditions,
  `healthcheck`, `device_cgroup_rules`, `devices`):
  https://docs.docker.com/reference/compose-file/services/
- Startup order and health conditions:
  https://docs.docker.com/compose/how-tos/startup-order/
- `docker compose up --exit-code-from` / `run`:
  https://docs.docker.com/reference/cli/docker/compose/up/ ,
  https://docs.docker.com/reference/cli/docker/compose/run/
- Dev Containers spec (what `.devcontainer.json` can do, UID remapping):
  https://containers.dev/implementors/json_reference/

ROS 2 dependency handling

- rosdep docs: https://docs.ros.org/en/independent/api/rosdep/html/ (read
  "rosdep.yaml format", "sources list", "contributing rules", and the
  pip/PEP 668 page https://docs.ros.org/en/independent/api/rosdep/html/pip_and_pep_668.html)
- The rosdep database itself (search for keys here before inventing one):
  https://github.com/ros/rosdistro/tree/master/rosdep (`base.yaml`, `python.yaml`)
  and contributing guidelines
  https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md
- REP 149 (package.xml format 3: `condition=`, dependency tags):
  https://www.ros.org/reps/rep-0149.html
- REP 2000 (which Ubuntu/arch each ROS distro supports; Jazzy = 24.04,
  amd64 + arm64): https://www.ros.org/reps/rep-2000.html
- colcon docs (mixins, `colcon.meta`, `--merge-install`, `colcon test`):
  https://colcon.readthedocs.io/
- vcstool: https://github.com/dirk-thomas/vcstool

ROS 2 in Docker

- Docker's ROS 2 guide: https://docs.docker.com/guides/ros2/
- Nav2 "Docker for Development: Zero to Hero" (dev vs deploy images, the
  official image variants): https://docs.nav2.org/tutorials/docs/docker_dev.html
- Articulated Robotics "Docker for Robotics" series (4 parts; part 4 is
  devices/USB): https://articulatedrobotics.xyz/tutorials/docker/docker-overview
- Allison Thackston, "VSCode, Docker, and ROS2" and her Dockerfiles:
  https://www.allisonthackston.com/articles/vscode-docker-ros2.html ,
  https://github.com/athackst/dockerfiles
- The official `ros:*` image Dockerfiles (how OSRF installs ROS into Ubuntu;
  our `base.Dockerfile` should look like these):
  https://github.com/osrf/docker_images
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/
- Isaac ROS common (NVIDIA's reference for RealSense + Jetson in containers,
  including udev handling): https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common

Fast DDS in containers

- SHM in Docker (why `network_mode: host` + `ipc: host`):
  https://fast-dds.docs.eprosima.com/en/latest/docker/shm_docker.html
- Environment variables, including `FASTDDS_BUILTIN_TRANSPORTS`:
  https://fast-dds.docs.eprosima.com/en/stable/fastdds/env_vars/env_vars.html
- Shared-memory transport notes (root vs non-root):
  https://fast-dds.docs.eprosima.com/en/stable/fastdds/transport/shared_memory/shared_memory.html
- ROS 2 discovery range / localhost-only:
  https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html

RealSense

- ROS 2 wrapper (`realsense2_camera`), launch parameters, bag replay:
  https://github.com/IntelRealSense/realsense-ros
- librealsense Docker notes (device cgroup rules):
  https://github.com/IntelRealSense/librealsense/blob/master/scripts/Docker/readme.md

Reference implementations

- WATonomous `wato_monorepo` (`docker/DEVELOPING.md`, `watod`,
  `modules/docker-compose*.yaml`): https://github.com/WATonomous/wato_monorepo
  Read it for the compose/profile conventions; note their choices that we are
  not copying (rmw_zenoh, host-UID dev images FROM the deps stage, per-stage
  registry images).
- ONNX Runtime CUDA EP requirements and `preload_dlls`:
  https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html
