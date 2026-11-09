#!/usr/bin/env bash
# CPU-only navigation environment; CuPy/height_env.sh is not required for frozen maps.
_nav_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$_nav_ws/.deps_install/local_setup.bash" ]]; then source "$_nav_ws/.deps_install/local_setup.bash"; fi
source "$_nav_ws/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$_nav_ws/src/legged_bringup/config/fastdds_common.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
unset _nav_ws
