#!/usr/bin/env bash
set -euo pipefail

catkin config --init --mkdirs \
              --extend /opt/ros/${ROS_DISTRO} \
              --merge-devel \
              --cmake-args -DCMAKE_BUILD_TYPE=Release \
                            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build "$@"

./merge_compile_commands.py