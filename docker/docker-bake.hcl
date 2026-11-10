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
//   docker/all.Dockerfile       merges every module's sources for the all-modules
//                               images (all-deploy, all-dev-<variant>)

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

// One cache scope per image: perception-cpu, perception-gpu, control, all-gpu.
// Only the image's own target exports it. mode=max includes every stage the
// image was built from, so the base and module stages are stored once per
// image instead of once per target (the repo's cache is capped at 10 GB).
// Every target imports the scopes of the images built on it.
function "cache_from" {
  params = [scopes]
  result = CI ? [for scope in scopes : "type=gha,scope=${scope}"] : []
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
  cache-from = cache_from(v.name == "cpu" ? ["perception-cpu", "control"] : ["perception-gpu", "all-gpu"])
}

######################### perception #########################

target "perception-source" {
  name       = "perception-source-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/perception.Dockerfile"
  target     = "source"
  contexts   = { base = "target:base-${v.name}" }
  cache-from = cache_from(["perception-${v.name}", "all-${v.name}"])
}

target "perception-deps" {
  name       = "perception-deps-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/perception.Dockerfile"
  target     = "dependencies"
  contexts   = { base = "target:base-${v.name}" }
  cache-from = cache_from(["perception-${v.name}", "all-${v.name}"])
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
  cache-from = cache_from(["perception-${v.name}"])
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
  cache-from = cache_from(["control", "all-gpu"])
}

target "control-deps" {
  context    = "."
  dockerfile = "docker/control.Dockerfile"
  target     = "dependencies"
  contexts   = { base = "target:base-cpu" }
  cache-from = cache_from(["control"])
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
  cache-from = cache_from(["control"])
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

######################### all modules #########################
// Every module in one image. all.Dockerfile merges the module source stages.
// The module dependencies stages are stacked instead: each one is built with
// the previous module's stage as its `base`.
// Adding a module: a COPY line in all.Dockerfile, its source in all-source's
// contexts, and one more step in the all-deps stack.
// Both are matrixed so rosdep_scan resolves package.xml conditions with the
// variant's AP1_GPU (onnxruntime vs onnxruntime-gpu).

target "all-source" {
  name       = "all-source-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/all.Dockerfile"
  target     = "source"
  contexts = {
    base                = "target:base-${v.name}"
    "perception-source" = "target:perception-source-${v.name}"
    // control has no variants: its source files are the same for both
    "control-source" = "target:control-source"
  }
  cache-from = cache_from(["all-${v.name}"])
}

target "all-deps" {
  name       = "all-deps-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/control.Dockerfile"
  target     = "dependencies"
  contexts   = { base = "target:perception-deps-${v.name}" }
  cache-from = cache_from(["all-${v.name}"])
}

// Deploy image: GPU only, like the car. Compose runs one container per module from it
target "all-deploy" {
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "deploy"
  contexts = {
    "module-source" = "target:all-source-gpu"
    "module-deps"   = "target:all-deps-gpu"
  }
  tags = compact([
    "${AP1_REGISTRY}/all:gpu",
    notequal(GIT_SHA, "") ? "${AP1_REGISTRY}/all:gpu-${GIT_SHA}" : "",
  ])
  cache-from = cache_from(["all-gpu"])
  cache-to   = cache_to("all-gpu")
}

// Dev image: every module's dependencies + dev tools; source is mounted and built at runtime
target "all-dev" {
  name       = "all-dev-${v.name}"
  matrix     = { v = VARIANTS }
  context    = "."
  dockerfile = "docker/base.Dockerfile"
  target     = "develop"
  contexts = {
    "module-source" = "target:all-source-${v.name}"
    "module-deps"   = "target:all-deps-${v.name}"
  }
  tags = ["${AP1_REGISTRY}/all:${v.name}-dev"]
}
