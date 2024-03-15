#!/bin/bash
set -e

# souce ros2 environment
source "/ros_ws/install/setup.bash" --
source "/opt/ros/humble/setup.bash" --
exec "$@"
