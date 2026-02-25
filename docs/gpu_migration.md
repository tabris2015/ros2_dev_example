# GPU Migration Guide for ml_inference

This guide explains how to switch from CPU-only PyTorch to CUDA-accelerated inference.

## Prerequisites

- NVIDIA GPU with compute capability >= 3.5
- NVIDIA driver >= 525.x installed on the **host** machine
- NVIDIA Container Toolkit (`nvidia-docker2`) installed on the host:
  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

## Step 1: Update the Dockerfile

In `.devcontainer/Dockerfile`, change the `PYTORCH_INDEX_URL` build arg:

```dockerfile
# For CUDA 11.8:
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu118

# For CUDA 12.1:
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu121
```

Or build with an override without editing the file:

```bash
docker build --build-arg PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu118 .
```

## Step 2: Update devcontainer.json

Add `"--gpus", "all"` to the `runArgs` array in `.devcontainer/devcontainer.json`:

```json
"runArgs": [
    "--network=host",
    "--privileged",
    "--gpus", "all"
]
```

## Step 3: Rebuild the devcontainer

In VS Code: `Ctrl+Shift+P` -> "Dev Containers: Rebuild Container"

## Step 4: Run the node with GPU

```bash
ros2 run ml_inference detector_node --ros-args -p device:=cuda:0
```

The `device` parameter accepts any valid PyTorch device string:
- `cpu` — CPU inference (default)
- `cuda:0` — First GPU
- `cuda:1` — Second GPU (multi-GPU systems)

The node automatically falls back to CPU if CUDA is requested but unavailable.

## Verification

```bash
python3 -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

## Image Size Impact

- CPU-only PyTorch: ~800 MB added to image
- CUDA PyTorch: ~2.5 GB added to image (includes CUDA runtime bundled in the wheel)
