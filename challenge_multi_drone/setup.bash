#!/bin/bash
export WORKSPACE="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )/.." && pwd )"
# --- 用 CycloneDDS：WSL 下 Fast-DDS 共享内存反复失败会导致 ROS2 发现不稳定 ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source "$WORKSPACE/install/setup.bash"
echo "Sourced WORKSPACE at $WORKSPACE"
