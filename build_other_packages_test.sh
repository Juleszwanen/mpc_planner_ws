#!/bin/bash
# Usage: ./build.sh <system_name> <generate_solver>
cd /workspace
clear

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/workspace/acados/lib"
export ACADOS_SOURCE_DIR="/workspace/acados"

. /opt/ros/noetic/setup.sh

# Source the workspace if it exists
if [ -f /workspace/devel/setup.sh ]; then
  . /workspace/devel/setup.sh
fi

BUILD_TYPE=RelWithDebInfo # Release, Debug, RelWithDebInfo, MinSizeRel
catkin config --cmake-args -DCATKIN_ENABLE_TESTING=ON -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DPYTHON_VERSION=3 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 

catkin build $1 

