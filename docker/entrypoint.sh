#!/usr/bin/env bash
set -euo pipefail

# Source ROS + workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi

# Better ccache/tmp shm perf in containers
export RCUTILS_COLORIZED_OUTPUT=1
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}

exec "$@"

