#!/usr/bin/env bash
# Optional, isolated Garden build; the hardware driver is not needed for PointCloud2.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
USV_LIVOX_SOURCE="$USV_NAV_ROOT/external/livox_gz_sim"
USV_LIVOX_REVISION="$(python3 -c 'import json,os; print(json.load(open(os.environ["USV_NAV_ROOT"]+"/config/dependencies.lock.json"))["livox_gz_sim"])')"
if [[ ! -d "$USV_LIVOX_SOURCE/.git" ]]; then
    git clone https://github.com/mathaimp/livox_gz_sim.git "$USV_LIVOX_SOURCE"
    git -C "$USV_LIVOX_SOURCE" checkout --detach "$USV_LIVOX_REVISION"
fi
[[ "$(git -C "$USV_LIVOX_SOURCE" rev-parse HEAD)" == "$USV_LIVOX_REVISION" ]]
if git -C "$USV_LIVOX_SOURCE" diff --quiet; then
    git -C "$USV_LIVOX_SOURCE" apply "$USV_NAV_ROOT/patches/livox_gz_sim_garden.patch"
fi
git -C "$USV_LIVOX_SOURCE" diff --binary > "$USV_NAV_ROOT/external/livox_current.patch"
cmp "$USV_NAV_ROOT/external/livox_current.patch" "$USV_NAV_ROOT/patches/livox_gz_sim_garden.patch"
cmake -S "$USV_LIVOX_SOURCE" -B "$USV_LIVOX_SOURCE/build" \
    -DCMAKE_INSTALL_PREFIX="$USV_LIVOX_SOURCE/install" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$USV_LIVOX_SOURCE/build" -j4
cmake --install "$USV_LIVOX_SOURCE/build"
