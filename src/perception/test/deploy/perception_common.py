"""Settings shared by the perception deploy tests."""
import os
import subprocess
from pathlib import Path

# Where the deploy and dev services mount the YOLO model (modules/docker-compose.yaml)
MODEL_PATH = Path('/opt/ap1/models/yolo11n.onnx')


def pick_device() -> str:
    """Return 'cuda' on a GPU image with a GPU visible, otherwise 'cpu'.

    The NVIDIA runtime only mounts nvidia-smi into containers started with GPU
    access, so a missing binary means there's no GPU to use.
    """
    if os.environ.get('AP1_GPU') != '1':
        return 'cpu'
    try:
        result = subprocess.run(['nvidia-smi', '-L'], capture_output=True, timeout=30, check=False)
    except (OSError, subprocess.SubprocessError):
        return 'cpu'
    return 'cuda' if result.returncode == 0 else 'cpu'
