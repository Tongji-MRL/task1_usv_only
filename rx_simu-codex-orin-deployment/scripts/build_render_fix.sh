#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
python3 - <<'PY'
from pathlib import Path
import subprocess,json,os
root=Path(os.environ['USV_NAV_ROOT']);vrx=Path(os.environ['VRX_ROOT'])
rev=json.loads((root/'config/dependencies.lock.json').read_text())['vrx']
out=root/'external/vrx_render_fix';out.mkdir(exist_ok=True)
for name in ['WaveVisual.cc','WaveVisual.hh','Wavefield.hh']:
    (out/name).write_bytes(subprocess.check_output(['git','show',rev+':vrx_gz/src/'+name],cwd=vrx))
subprocess.run(['patch','-p3','-i',str(root/'patches/vrx_wavevisual_teardown.patch')],cwd=out,check=True)
PY
cmake -S "$USV_NAV_ROOT/tools/vrx_render_fix" -B "$USV_NAV_ROOT/build/vrx_render_fix" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$USV_NAV_ROOT/build/vrx_render_fix" -j4
