#!/usr/bin/env bash
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$USV_NAV_ROOT"
cmake -S external/ardupilot_gazebo -B external/ardupilot_gazebo/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=TRUE \
  -DCMAKE_CXX_FLAGS="-I$USV_NAV_ROOT/external/deps/usr/include"
cmake --build external/ardupilot_gazebo/build --target ArduPilotPlugin -j8
colcon build --symlink-install --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
cmake -S tools/task1_gazebo -B build/task1_gazebo -DCMAKE_BUILD_TYPE=Release
cmake --build build/task1_gazebo -j4
bash scripts/build_render_fix.sh
cmake -S tools/tf_audit -B build/tf_audit
cmake --build build/tf_audit -j4
