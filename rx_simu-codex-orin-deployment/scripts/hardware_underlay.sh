#!/usr/bin/env bash
# Shared by the hardware builder and runtime environment; starts no nodes.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Source this file from Bash; it only configures the current shell.' >&2
    exit 2
fi
USV_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
export USV_WORKSPACE
if [[ -f "$USV_WORKSPACE/.env.boat" ]]; then
    source "$USV_WORKSPACE/.env.boat" || return 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'ROS 2 Humble is missing. Follow docs/hardware_deployment.md, step 3.' >&2
    return 1
fi
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash || return 1
# apt installs need no overlay. Existing or separately built drivers are opt-in.
for USV_SETUP_PATH in "${USV_MAVROS_SETUP:-}" "${USV_LIVOX_SETUP:-}"; do
    [[ -z "$USV_SETUP_PATH" ]] && continue
    if [[ ! -f "$USV_SETUP_PATH" ]]; then
        echo "Configured setup file does not exist: $USV_SETUP_PATH" >&2
        unset USV_SETUP_PATH
        return 1
    fi
    # shellcheck source=/dev/null
    source "$USV_SETUP_PATH" || return 1
done
if [[ -n "${USV_LIVOX_SDK_PREFIX:-}" ]]; then
    if [[ ! -f "$USV_LIVOX_SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so" ]]; then
        echo "Livox SDK library missing under $USV_LIVOX_SDK_PREFIX" >&2
        return 1
    fi
    export LD_LIBRARY_PATH="$USV_LIVOX_SDK_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export CMAKE_PREFIX_PATH="$USV_LIVOX_SDK_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-62}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
unset USV_SETUP_PATH
