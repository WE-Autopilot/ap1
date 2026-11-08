// Build graph for AP1 images: a base image per hardware variant, then modules on top.
// Docker bake and HCL file reference: https://docs.docker.com/build/bake/reference/ 
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

variable "AP1_REGISTRY" {
  default = "ghcr.io/we-autopilot/ap1"
}

variable "ROS_DISTRO" {
  default = "jazzy"
}

variable "GIT_SHA" {
  default = ""
}

// GitHub Actions sets CI=true, turns on the GitHub Actions layer cache
variable "CI" {
  default = false
}

// Hardware variants. TODO: add arm64 image for Jetson Orin deployment
variable "VARIANTS" {
  default = [
    { name = "cpu", image = "ubuntu:24.04", gpu = "0" },
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

// Every per-module deploy image in one command, to check each module builds on its own
group "modules" {
  targets = ["perception-cpu", "control"]
}

############################ base ############################

# Each target defines a docker build invocation
target "base" {
  name       = "base-${v.name}"
  matrix     = { v = VARIANTS }
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
  name       = "perception-source-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/perception.Dockerfile"
  target     = "source"
  contexts   = { base = "target:base-${v.name}" }
  cache-from = cache_from("perception-source-${v.name}")
  cache-to   = cache_to("perception-source-${v.name}")
}

target "perception-deps" {
  name       = "perception-deps-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/perception.Dockerfile"
  target     = "dependencies"
  contexts   = { base = "target:base-${v.name}" }
  cache-from = cache_from("perception-deps-${v.name}")
  cache-to   = cache_to("perception-deps-${v.name}")
}

// Deploy image: what runs on the car / in integration tests
target "perception" {
  name       = "perception-${v.name}"
  matrix     = { v = VARIANTS }
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

// Dev image: dependencies + dev tools; source is mounted and built at runtime
target "perception-dev" {
  name       = "perception-dev-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "develop"
  contexts = {
    "module-source" = "target:perception-source-${v.name}"
    "module-deps"   = "target:perception-deps-${v.name}"
  }
  tags = ["${AP1_REGISTRY}/perception:${v.name}-dev"]
}

######################### control #########################
// CPU only: control has no GPU dependencies, so no matrix

target "control-source" {
  context    = "."
  dockerfile = "docker/control.Dockerfile"
  target     = "source"
  contexts   = { base = "target:base-cpu" }
  cache-from = cache_from("control-source")
  cache-to   = cache_to("control-source")
}

target "control-deps" {
  context    = "."
  dockerfile = "docker/control.Dockerfile"
  target     = "dependencies"
  contexts   = { base = "target:base-cpu" }
  cache-from = cache_from("control-deps")
  cache-to   = cache_to("control-deps")
}

// Deploy image: what runs on the car / in integration tests
target "control" {
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "deploy"
  contexts = {
    "module-source" = "target:control-source"
    "module-deps"   = "target:control-deps"
  }
  tags = compact([
    "${AP1_REGISTRY}/control:cpu",
    notequal(GIT_SHA, "") ? "${AP1_REGISTRY}/control:cpu-${GIT_SHA}" : "",
  ])
  cache-from = cache_from("control")
  cache-to   = cache_to("control")
}

// Dev image: dependencies + dev tools; source is mounted and built at runtime
target "control-dev" {
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "develop"
  contexts = {
    "module-source" = "target:control-source"
    "module-deps"   = "target:control-deps"
  }
  tags = ["${AP1_REGISTRY}/control:cpu-dev"]
}
