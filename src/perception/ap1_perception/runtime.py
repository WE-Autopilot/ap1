"""ONNX Runtime sessions shared by every perception model (YOLO, road segmentation, UFLD).

The same code runs on CPU and GPU. Which one is used depends on the installed
onnxruntime package (onnxruntime vs onnxruntime-gpu), whether TensorRT is
installed (GPU images, the Jetson), and the `device` argument:

- "tensorrt": TensorRT, through onnxruntime's TensorRT execution provider. Layers
  TensorRT can't run fall back to CUDA, then CPU. The first session on a machine
  builds a TensorRT engine for the model (seconds on a desktop GPU, minutes on the
  Jetson) and caches it in trt_cache_dir(); later sessions load the cached engine.
- "cuda": CUDA through onnxruntime, no TensorRT.
- "cpu": CPU only.
- "auto": the fastest of those the installed onnxruntime supports.
"""
import os
from pathlib import Path

import onnxruntime as ort

CPU_PROVIDER = "CPUExecutionProvider"
CUDA_PROVIDER = "CUDAExecutionProvider"
TRT_PROVIDER = "TensorrtExecutionProvider"

_LFS_POINTER = b"version https://git-lfs.github.com/spec/v1"


def trt_cache_dir() -> Path:
    """Where TensorRT engines are cached: $AP1_TRT_CACHE, or ~/.cache/ap1/trt."""
    return Path(os.environ.get("AP1_TRT_CACHE", Path.home() / ".cache" / "ap1" / "trt"))


def select_providers(device: str) -> list[str]:
    """Map a device name ("auto", "tensorrt", "cuda", "cpu") to ONNX Runtime providers, fastest first."""
    available = ort.get_available_providers()
    if device == "cpu":
        return [CPU_PROVIDER]
    if device == "cuda":
        if CUDA_PROVIDER not in available:
            raise RuntimeError(
                f"device='cuda' requested but onnxruntime has no CUDA support "
                f"(available providers: {available}). Use the GPU image, or device='cpu'."
            )
        return [CUDA_PROVIDER, CPU_PROVIDER]
    if device == "tensorrt":
        if TRT_PROVIDER not in available:
            raise RuntimeError(
                f"device='tensorrt' requested but onnxruntime has no TensorRT support "
                f"(available providers: {available}). Use the GPU image, or device='cuda'/'cpu'."
            )
        return [TRT_PROVIDER, CUDA_PROVIDER, CPU_PROVIDER]
    if device == "auto":
        return [p for p in (TRT_PROVIDER, CUDA_PROVIDER) if p in available] + [CPU_PROVIDER]
    raise ValueError(f"Unknown device '{device}', expected 'auto', 'tensorrt', 'cuda' or 'cpu'")


def _with_options(providers: list[str]) -> list:
    """Attach provider options: TensorRT runs in FP16 and caches its engines."""
    if TRT_PROVIDER not in providers:
        return providers
    cache = trt_cache_dir()
    cache.mkdir(parents=True, exist_ok=True)
    trt_options = {
        "trt_fp16_enable": True,
        "trt_engine_cache_enable": True,
        "trt_engine_cache_path": str(cache),
        "trt_timing_cache_enable": True,
        "trt_timing_cache_path": str(cache),
    }
    return [(p, trt_options) if p == TRT_PROVIDER else p for p in providers]


def _session_options(threads: int) -> ort.SessionOptions:
    """CPU threading: a few threads that sleep while idle, so a node shares the CPU fairly.

    onnxruntime's default is one thread per core, busy-waiting ("spinning") between
    jobs. That keeps several cores at 100% even while idle, starves the other nodes
    and makes laptops throttle.
    """
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = threads  # threads that split one operation's maths between them
    opts.inter_op_num_threads = 1        # run the model's operations one after another
    opts.add_session_config_entry("session.intra_op.allow_spinning", "0")  # sleep, don't spin
    return opts


def create_session(model_path, device: str = "auto", threads: int = 4) -> ort.InferenceSession:
    """Open an .onnx model on the providers `device` selects, using `threads` CPU threads.

    Fails early on a missing file or an unfetched Git LFS pointer, and when a
    requested accelerator silently falls back to a slower provider.
    """
    model_path = Path(model_path)
    if not model_path.is_file():
        raise FileNotFoundError(f"Model not found at '{model_path}'")
    with open(model_path, "rb") as f:
        if f.read(len(_LFS_POINTER)) == _LFS_POINTER:
            raise RuntimeError(
                f"'{model_path}' is a Git LFS pointer, not the model. "
                f"Run 'git lfs pull' in the repo, then rebuild."
            )

    providers = select_providers(device)
    session = ort.InferenceSession(
        str(model_path),
        sess_options=_session_options(threads),
        providers=_with_options(providers),
    )
    # onnxruntime silently falls back (TensorRT -> CUDA -> CPU) if a provider's libraries fail to load
    requested = {"tensorrt": TRT_PROVIDER, "cuda": CUDA_PROVIDER}.get(device)
    if requested and requested not in session.get_providers():
        raise RuntimeError(
            f"device='{device}' requested but the {requested} failed to load "
            f"(check the onnxruntime log above for the missing library)."
        )
    return session
