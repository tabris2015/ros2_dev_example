#!/usr/bin/env bash
# Runs once after the devcontainer is created.
# Installs whatever the packages under src/ declare in package.xml.
# The image deletes apt lists to stay small, so refresh them first.
set -euo pipefail

sudo apt-get update
rosdep update --rosdistro "${ROS_DISTRO}"
rosdep install --from-paths src --ignore-src -y
