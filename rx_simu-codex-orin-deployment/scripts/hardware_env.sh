#!/usr/bin/env bash
# Source in each hardware shell. Does not start drivers or enable control.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source scripts/hardware_env.sh' >&2
    exit 2
fi
source "$(dirname "${BASH_SOURCE[0]}")/hardware_underlay.sh" || return 1
for USV_SETUP_PATH in "$USV_WORKSPACE/.venv/bin/activate" \
    "$USV_WORKSPACE/install/hardware/local_setup.bash"; do
    if [[ ! -f "$USV_SETUP_PATH" ]]; then
        echo "Missing $USV_SETUP_PATH; complete the hardware build first." >&2
        unset USV_SETUP_PATH
        return 1
    fi
done
source "$USV_WORKSPACE/.venv/bin/activate" || return 1
source "$USV_WORKSPACE/install/hardware/local_setup.bash" || return 1
unset USV_SETUP_PATH
