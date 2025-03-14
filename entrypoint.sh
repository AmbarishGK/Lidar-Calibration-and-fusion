#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source MOLA workspace if it exists
if [ -f "/root/mola_ws/install/setup.bash" ]; then
  source /root/mola_ws/install/setup.bash
fi

# Execute the command passed to docker run
exec "$@"
