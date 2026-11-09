#!/usr/bin/env bash
# Source this file. All additions are local to this shell and this workspace.
_height_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$_height_ws/.deps_install/local_setup.bash" ]]; then
    source "$_height_ws/.deps_install/local_setup.bash"
fi
if [[ -f "$_height_ws/install/local_setup.bash" ]]; then
    source "$_height_ws/install/local_setup.bash"
fi
# ROS executables may use /usr/bin/python3 in their shebang. Make the private
# CuPy wheel visible there too, without installing anything in system Python.
export PYTHONPATH="$_height_ws/.venv-height/lib/python3.10/site-packages:${PYTHONPATH:-}"
export CUDA_PATH=/usr/local/cuda
export PATH="$CUDA_PATH/bin:${PATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset _height_ws
