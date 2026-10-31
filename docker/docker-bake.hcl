// Build graph for AP1 images: a base image per hardware variant, then modules on top.
// Used locally and by CI, so builds are defined in exactly one place.
//
// Run from the repo root:
//   docker buildx bake -f docker/docker-bake.hcl --load perception-cpu
//   docker buildx bake -f docker/docker-bake.hcl --print perception-gpu   # show resolved config
//
// Two Dockerfiles, chained through named contexts:
//   docker/<module>.Dockerfile  module sources and extra deps
//                               (<module>-source-<variant>, <module>-deps-<variant>)
//   docker/base.Dockerfile      base-<variant> (stage `base`), and the shared pipeline
//                               that turns a module into <module>-<variant> (stage `deploy`)
//                               and <module>-dev-<variant> (stage `develop`)
//
// Variables are read from the environment, under the same names compose uses
// (modules/docker-compose.yaml), e.g. AP1_UID=$(id -u) for the dev image.

variable "AP1_REGISTRY" {
  default = "ghcr.io/we-autopilot/ap1"
}

variable "ROS_DISTRO" {
  default = "jazzy"
}

// Set in CI to add an immutable per-commit tag
variable "GIT_SHA" {
  default = ""
}

// GitHub Actions sets CI=true, which turns on the GitHub Actions layer cache
variable "CI" {
  default = false
}

// Dev images give their "ubuntu" user the host's IDs, so bind-mounted files keep their owner
variable "AP1_UID" {
  default = "1000"
}
variable "AP1_GID" {
  default = "1000"
}

// Hardware variants. Adding one (e.g. Jetson) means adding an entry here;
// module Dockerfiles don't change.
variable "VARIANTS" {
  default = [
    { name = "cpu", image = "ubuntu:24.04", gpu = "0" },
    // CUDA/cuDNN must match onnxruntime-gpu in docker/config/pip-constraints.txt
    { name = "gpu", image = "nvidia/cuda:13.0.3-cudnn-runtime-ubuntu24.04", gpu = "1" },
  ]
}

function "cache_from" {
  params = [scope]
  result = CI ? ["type=gha,scope=${scope}"] : []
}

function "cache_to" {
  params = [scope]
  result = CI ? ["type=gha,scope=${scope},mode=max"] : []
}

group "default" {
  targets = ["perception-cpu"]
}

############################ base ############################

target "base" {
  matrix     = { v = VARIANTS }
  name       = "base-${v.name}"
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "base"
  args = {
    GENERIC_IMAGE = v.image
    AP1_GPU       = v.gpu
    ROS_DISTRO    = ROS_DISTRO
  }
  tags       = ["${AP1_REGISTRY}/base:${ROS_DISTRO}-${v.name}"]
  cache-from = cache_from("base-${v.name}")
  cache-to   = cache_to("base-${v.name}")
}

######################### perception #########################

target "perception-source" {
  matrix     = { v = VARIANTS }
  name       = "perception-source-${v.name}"
  context    = "."
  dockerfile = "docker/perception.Dockerfile"
  target     = "source"
  contexts   = { base = "target:base-${v.name}" }
  cache-from = cache_from("perception-source-${v.name}")
  cache-to   = cache_to("perception-source-${v.name}")
}

target "perception-deps" {
  matrix     = { v = VARIANTS }
  name       = "perception-deps-${v.name}"
  context    = "."
  dockerfile = "docker/perception.Dockerfile"
  target     = "dependencies"
  contexts   = { base = "target:base-${v.name}" }
  cache-from = cache_from("perception-deps-${v.name}")
  cache-to   = cache_to("perception-deps-${v.name}")
}

// Deploy image: what runs on the car / in integration tests
target "perception" {
  matrix     = { v = VARIANTS }
  name       = "perception-${v.name}"
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "deploy"
  contexts = {
    "module-source" = "target:perception-source-${v.name}"
    "module-deps"   = "target:perception-deps-${v.name}"
  }
  tags = compact([
    "${AP1_REGISTRY}/perception:${v.name}",
    notequal(GIT_SHA, "") ? "${AP1_REGISTRY}/perception:${v.name}-${GIT_SHA}" : "",
  ])
  cache-from = cache_from("perception-${v.name}")
  cache-to   = cache_to("perception-${v.name}")
}

// Dev image: dependencies + dev tools + host user's IDs; source is mounted and built at runtime
target "perception-dev" {
  matrix     = { v = VARIANTS }
  name       = "perception-dev-${v.name}"
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "develop"
  contexts = {
    "module-source" = "target:perception-source-${v.name}"
    "module-deps"   = "target:perception-deps-${v.name}"
  }
  args = {
    USER_UID = AP1_UID
    USER_GID = AP1_GID
  }
  tags = ["${AP1_REGISTRY}/perception:${v.name}-dev"]
}
