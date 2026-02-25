# .devcontainer/Dockerfile
FROM osrf/ros:humble-desktop-full

# Avoid warnings by switching to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Create a non-root user (matches VS Code's default)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install basic tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    gdb \
    && rm -rf /var/lib/apt/lists/*

# ---- ML dependencies ----
# CPU-only PyTorch. To switch to GPU, change the index URL:
#   https://download.pytorch.org/whl/cu118  (CUDA 11.8)
#   https://download.pytorch.org/whl/cu121  (CUDA 12.1)
# and add "--gpus", "all" to devcontainer.json runArgs.
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cpu
RUN pip3 install --no-cache-dir \
    torch torchvision \
    --index-url ${PYTORCH_INDEX_URL}

# Set the default user
USER $USERNAME

# Source ROS 2 setup file automatically in every new terminal
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc