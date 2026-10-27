"""Where perception's model files live once installed.

Model files are kept in src/perception/models/ (Git LFS) and installed to the
package's share folder, so the same lookup works natively, in the dev
container and in the deploy image. This is the only place the YOLO model's
file name appears.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

YOLO_MODEL_FILE = 'yolo11n.onnx'


def models_dir() -> Path:
    """The installed models folder: <ap1_perception share>/models."""
    return Path(get_package_share_directory('ap1_perception')) / 'models'


def default_yolo_model() -> str:
    """Path to the YOLO model installed with the package."""
    return str(models_dir() / YOLO_MODEL_FILE)
