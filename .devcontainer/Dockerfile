# syntax=docker/dockerfile:1
#
# Two targets, both amd64 and arm64:
#   base  headless. ROS 2 core, build and lint tooling, teleop. Fits a Jetson and CI.
#   full  base plus ros-jazzy-desktop (rviz2, rqt), PlotJuggler, xacro, and X11 bits.
#
# osrf/ros:*-desktop* images are amd64-only, so `full` is built up from ros-base
# instead of pulled. Gazebo (ros-jazzy-desktop-full) is intentionally absent; it
# arrives with the Gazebo appendix.
#
# Build context is this directory. Nothing is copied from the workspace, so the
# image never depends on package files.

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------- base ----
FROM ros:${ROS_DISTRO}-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive \
    # PEP 668 escape hatch. Disposable container, and ROS Python deps come
    # through rosdep/apt anyway.
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    # Stable log lines so README "expected output" blocks can be pasted verbatim.
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}" \
    # setuptools on noble warns on every ament_python build; the build still succeeds.
    PYTHONWARNINGS="ignore:setup.py install is deprecated,ignore:easy_install command is deprecated"

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    gdb \
    git \
    less \
    nano \
    python3-pip \
    python3-pytest \
    python3-venv \
    sudo \
    vim \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-ament-cmake-pytest \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-launch-testing \
    ros-${ROS_DISTRO}-launch-testing-ros \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching VS Code's default uid. Noble images ship an `ubuntu`
# user at uid 1000, which would make useradd fail, so evict it first.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
RUN if id -u ${USER_UID} >/dev/null 2>&1; then userdel -r "$(id -un ${USER_UID})"; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -m -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}

# rosdep cache lands in the user's home, where rosdep install can read it.
# The bashrc sources ROS, colcon completion, and the workspace overlay when
# $WS (set by devcontainer.json) points at a built workspace.
RUN rosdep update --rosdistro ${ROS_DISTRO} \
    && printf '%s\n' \
        '' \
        '# ROS 2' \
        'source /opt/ros/${ROS_DISTRO}/setup.bash' \
        '[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' \
        '[ -n "$WS" ] && [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"' \
        >> ~/.bashrc

# ---------------------------------------------------------------- full ----
FROM base AS full

ARG USERNAME=vscode
USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-xacro \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
