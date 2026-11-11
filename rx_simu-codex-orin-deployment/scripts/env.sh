#!/usr/bin/env bash
# Source this file; all additions are scoped to this shell and its children.
USV_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export USV_NAV_ROOT
export VRX_ROOT="${VRX_ROOT:-/home/zjy/vrx_docking}"
source /opt/ros/humble/setup.bash
source "$VRX_ROOT/install/setup.bash"
USV_DEPS="$USV_NAV_ROOT/external/deps"
USV_ROS_PREFIX="$USV_DEPS/opt/ros/humble"
export AMENT_PREFIX_PATH="$USV_ROS_PREFIX:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$USV_ROS_PREFIX:$USV_DEPS/usr:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$USV_ROS_PREFIX/lib:$USV_DEPS/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$USV_ROS_PREFIX/local/lib/python3.10/dist-packages:$USV_ROS_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}"
export PATH="$USV_ROS_PREFIX/bin:$USV_DEPS/usr/bin:$PATH"
export GEOGRAPHICLIB_GEOID_PATH="$USV_DEPS/usr/share/GeographicLib/geoids"
export GZ_VERSION=garden
export GZ_SIM_SYSTEM_PLUGIN_PATH="$USV_NAV_ROOT/build/task1_gazebo:$USV_NAV_ROOT/external/ardupilot_gazebo/build:$VRX_ROOT/install/vrx_gz/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$USV_NAV_ROOT/models:$USV_NAV_ROOT/runs/generated:$VRX_ROOT/vrx_urdf:$VRX_ROOT/vrx_gz/models:$VRX_ROOT/vrx_urdf/vrx_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
if [[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]]; then
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
export ROS_DOMAIN_ID="${USV_ROS_DOMAIN_ID:-42}"
export GZ_PARTITION="${USV_GZ_PARTITION:-usv_nav_validation}"
# All simulator components run on this host. Avoid routing local transport
# through a changing Wi-Fi / VPN interface; callers may explicitly override it.
export GZ_IP="${GZ_IP:-127.0.0.1}"
if [[ -f "$USV_NAV_ROOT/build/vrx_render_fix/libWaveVisual.so" ]]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$USV_NAV_ROOT/build/vrx_render_fix:$GZ_SIM_SYSTEM_PLUGIN_PATH"
fi
export ROS_LOG_DIR="$USV_NAV_ROOT/log/ros"
if [[ -f "$USV_NAV_ROOT/install/setup.bash" ]]; then
    source "$USV_NAV_ROOT/install/setup.bash"
fi
if [[ -f "$USV_NAV_ROOT/external/livox_gz_sim/install/lib/liblivox_gz_sim.so" ]]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$USV_NAV_ROOT/external/livox_gz_sim/install/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"
    export AMENT_PREFIX_PATH="$USV_NAV_ROOT/external/livox_gz_sim/install:$AMENT_PREFIX_PATH"
    export CMAKE_PREFIX_PATH="$USV_NAV_ROOT/external/livox_gz_sim/install:$CMAKE_PREFIX_PATH"
    export GZ_SIM_RESOURCE_PATH="$USV_NAV_ROOT/external/livox_gz_sim/install/share:$GZ_SIM_RESOURCE_PATH"
fi
