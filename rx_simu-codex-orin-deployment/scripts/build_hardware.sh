#!/usr/bin/env bash
# Build the hardware navigation packages and the Task 1 Python package on
# Ubuntu 22.04 / Humble. robotx_task1_nav (C++, OMPL) is not built here.
# This script never opens hardware connections or starts ROS nodes.
set -eo pipefail
USV_BUILD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$USV_BUILD_ROOT"
if [[ "$(uname -m)" != aarch64 && "$(uname -m)" != x86_64 ]]; then
    echo 'Supported architectures: aarch64 (Orin) and x86_64 (offline checks).' >&2
    exit 2
fi
USV_BUILD_RUN="$USV_BUILD_ROOT/runs/hardware_build_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$USV_BUILD_RUN"
exec > >(tee -a "$USV_BUILD_RUN/build.log") 2>&1
source scripts/hardware_underlay.sh
/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version'
if [[ ! -f "$USV_BUILD_ROOT/.venv/pyvenv.cfg" ]]; then
    /usr/bin/python3 -m venv --system-site-packages "$USV_BUILD_ROOT/.venv"
fi
touch "$USV_BUILD_ROOT/.venv/COLCON_IGNORE"
source "$USV_BUILD_ROOT/.venv/bin/activate"
python3 -m pip install --disable-pip-version-check -r requirements/hardware-pip.txt
# A user-installed NumPy must not shadow the system SciPy dependency.
python3 -W error::UserWarning - <<'PY'
import site, sys
import numpy, scipy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from pymavlink import mavutil
assert not site.ENABLE_USER_SITE
assert sys.prefix != sys.base_prefix, sys.executable
assert cKDTree([[0., 0.], [1., 0.]]).query([0.1, 0.])[1] == 0
assert numpy.allclose(Rotation.from_euler('z', 90, degrees=True).apply([1., 0., 0.]), [0., 1., 0.])
print('Python:', sys.executable, 'NumPy:', numpy.__version__, 'SciPy:', scipy.__version__)
PY
# Resolve Python dependencies using this interpreter and its pip, not ~/.local.
# robotx_task1 also lists simulation-only dependencies of nodes that are not
# started on the vessel (observer, mission, traffic_guard); those are skipped.
python3 "$(command -v rosdep)" check --rosdistro humble \
    --from-paths src/robotx_task1_interfaces src/usv_nav2 src/usv_sim src/robotx_task1 \
    --ignore-src --skip-keys 'robotx_task1_nav ros_gz_interfaces'
python3 -m colcon --log-base log/hardware build --symlink-install \
    --build-base build/hardware --install-base install/hardware \
    --base-paths src/robotx_task1_interfaces src/usv_nav2 src/usv_sim src/robotx_task1 \
    --packages-select robotx_task1_interfaces usv_nav2 usv_sim robotx_task1 \
    --executor sequential --event-handlers console_direct+ \
    --cmake-args "-DPython3_EXECUTABLE=$USV_BUILD_ROOT/.venv/bin/python3" \
                 "-DPYTHON_EXECUTABLE=$USV_BUILD_ROOT/.venv/bin/python3"
source install/hardware/local_setup.bash
python3 - "$USV_BUILD_RUN" <<'PY'
import importlib.metadata, json, platform, subprocess, sys
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from robotx_task1_interfaces.msg import MotionPermission
from usv_nav2.rover_bridge import RoverBridge
from usv_sim.livox_filter import LivoxFilter
import rosbag2_py
from robotx_task1 import survey_bag, survey_check
from robotx_task1.buoy_mapper import BuoyMapper
from robotx_task1.usv_mission import UsvMission
from robotx_task1.robocommand import RoboCommand
report = {
    'time': datetime.now().astimezone().isoformat(),
    'architecture': platform.machine(),
    'python': sys.executable,
    'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
    'git_status': subprocess.check_output(['git', 'status', '--short'], text=True),
    'packages': {name: get_package_prefix(name) for name in (
        'robotx_task1_interfaces', 'usv_nav2', 'usv_sim', 'robotx_task1', 'nav2_bringup',
        'mavros', 'pointcloud_to_laserscan')},
    'python_versions': {name: importlib.metadata.version(name) for name in (
        'numpy', 'scipy', 'pymavlink', 'setuptools')},
    'scope': 'Build and import checks only; no hardware connections or control.',
}
try:
    report['packages']['livox_ros_driver2'] = get_package_prefix('livox_ros_driver2')
except LookupError:
    report['packages']['livox_ros_driver2'] = None
    print('Livox driver not found; install it before the live sensor stage.')
out = Path(sys.argv[1])
(out / 'build_result.json').write_text(json.dumps(report, indent=2) + '\n')
(out / 'pip_freeze.txt').write_text(subprocess.check_output(
    [sys.executable, '-m', 'pip', 'freeze'], text=True))
print(json.dumps(report, indent=2))
PY
echo "Build completed. Evidence: $USV_BUILD_RUN"
echo 'In each runtime shell: source scripts/hardware_env.sh'
