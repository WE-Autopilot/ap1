"""Unit tests for the shared onnxruntime session helper. No model or GPU needed."""
import pytest

from ap1_perception import runtime
from ap1_perception.runtime import CPU_PROVIDER, CUDA_PROVIDER, TRT_PROVIDER


# onnxruntime's real providers depend on the image (CPU or GPU), so fake them
@pytest.fixture
def trt_available(monkeypatch):
    monkeypatch.setattr(runtime.ort, 'get_available_providers',
                        lambda: [TRT_PROVIDER, CUDA_PROVIDER, CPU_PROVIDER])


@pytest.fixture
def cuda_available(monkeypatch):
    monkeypatch.setattr(runtime.ort, 'get_available_providers', lambda: [CUDA_PROVIDER, CPU_PROVIDER])


@pytest.fixture
def cuda_missing(monkeypatch):
    monkeypatch.setattr(runtime.ort, 'get_available_providers', lambda: [CPU_PROVIDER])


def test_cpu_never_uses_gpu(trt_available):
    assert runtime.select_providers('cpu') == [CPU_PROVIDER]


def test_cuda_with_cuda(cuda_available):
    assert runtime.select_providers('cuda') == [CUDA_PROVIDER, CPU_PROVIDER]


def test_cuda_skips_tensorrt(trt_available):
    assert runtime.select_providers('cuda') == [CUDA_PROVIDER, CPU_PROVIDER]


def test_cuda_without_cuda_fails(cuda_missing):
    with pytest.raises(RuntimeError):
        runtime.select_providers('cuda')


def test_tensorrt_with_tensorrt(trt_available):
    assert runtime.select_providers('tensorrt') == [TRT_PROVIDER, CUDA_PROVIDER, CPU_PROVIDER]


def test_tensorrt_without_tensorrt_fails(cuda_available):
    with pytest.raises(RuntimeError):
        runtime.select_providers('tensorrt')


def test_auto_prefers_tensorrt(trt_available):
    assert runtime.select_providers('auto') == [TRT_PROVIDER, CUDA_PROVIDER, CPU_PROVIDER]


def test_auto_uses_cuda_without_tensorrt(cuda_available):
    assert runtime.select_providers('auto') == [CUDA_PROVIDER, CPU_PROVIDER]


def test_auto_falls_back_to_cpu(cuda_missing):
    assert runtime.select_providers('auto') == [CPU_PROVIDER]


def test_unknown_device_fails():
    with pytest.raises(ValueError):
        runtime.select_providers('tpu')


def test_tensorrt_options_cache_engines(tmp_path, monkeypatch):
    monkeypatch.setenv('AP1_TRT_CACHE', str(tmp_path / 'trt'))
    providers = runtime._with_options([TRT_PROVIDER, CUDA_PROVIDER, CPU_PROVIDER])
    name, options = providers[0]
    assert name == TRT_PROVIDER
    assert options['trt_engine_cache_enable'] is True
    assert options['trt_engine_cache_path'] == str(tmp_path / 'trt')
    assert (tmp_path / 'trt').is_dir()
    assert providers[1:] == [CUDA_PROVIDER, CPU_PROVIDER]


def test_no_options_without_tensorrt():
    assert runtime._with_options([CUDA_PROVIDER, CPU_PROVIDER]) == [CUDA_PROVIDER, CPU_PROVIDER]


def test_session_options_limit_threads_and_spinning():
    opts = runtime._session_options(threads=3)
    assert opts.intra_op_num_threads == 3
    assert opts.inter_op_num_threads == 1
    assert opts.get_session_config_entry('session.intra_op.allow_spinning') == '0'


def test_missing_model_fails(tmp_path):
    with pytest.raises(FileNotFoundError):
        runtime.create_session(tmp_path / 'missing.onnx')


def test_lfs_pointer_fails(tmp_path):
    pointer = tmp_path / 'model.onnx'
    pointer.write_bytes(b'version https://git-lfs.github.com/spec/v1\noid sha256:abc\nsize 1\n')
    with pytest.raises(RuntimeError, match='Git LFS pointer'):
        runtime.create_session(pointer)
