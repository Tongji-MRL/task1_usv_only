#!/usr/bin/env python3
"""Start/stop only this workspace's processes; run after sourcing scripts/env.sh."""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tarfile
import time
from contextlib import contextmanager

ROOT = Path(__file__).resolve().parents[1]
PIDFILE = ROOT/'runs/session.json'
REVISION = 'd0356316eb0fa34ed1c9f39f7b059b9ae7ba4664'


@contextmanager
def session_lock():
    with PIDFILE.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        yield


def read_session():
    try:
        return json.loads(PIDFILE.read_text())
    except FileNotFoundError:
        return None


def write_session(state):
    with session_lock():
        temporary=PIDFILE.with_name(f'session.{os.getpid()}.tmp')
        temporary.write_text(json.dumps(state,indent=2)+'\n')
        temporary.replace(PIDFILE)


def process_start(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()[19]
    except FileNotFoundError:
        return None


def stop(expected_run=None):
    state = read_session()
    if state is None:
        return
    if expected_run is not None and state['run_dir'] != str(expected_run):
        return
    started=time.monotonic()
    def active(child):
        try:
            stat=Path(f"/proc/{child['pid']}/stat").read_text().split(') ',1)[1].split()
            return stat[19]==child['start'] and stat[0]!='Z'
        except FileNotFoundError:return False
    def signal_group(child,sig):
        if process_start(child['pid'])==child['start']:
            try:os.killpg(child['pid'],sig)
            except ProcessLookupError:pass
    for child in reversed(state['processes']):
        signal_group(child,signal.SIGINT)
    deadline = time.monotonic()+8
    while time.monotonic() < deadline:
        if not any(active(p) for p in state['processes']):
            break
        time.sleep(0.2)
    terminated=[p['name'] for p in state['processes'] if active(p)]
    for child in state['processes']:
        signal_group(child,signal.SIGTERM)
    # Gazebo may remain blocked during shutdown after SIGTERM. Do not discard
    # its identity and allow the next world to share its transport publishers.
    deadline = time.monotonic()+3
    while time.monotonic()<deadline:
        if not any(active(p) for p in state['processes']):break
        time.sleep(.1)
    killed=[p['name'] for p in state['processes'] if active(p)]
    for child in state['processes']:
        signal_group(child,signal.SIGKILL)
    deadline=time.monotonic()+2
    while any(active(p) for p in state['processes']) and time.monotonic()<deadline:time.sleep(.1)
    if any(active(p) for p in state['processes']):
        raise RuntimeError('Owned processes survived shutdown; retaining session identity and refusing a new session')
    run=Path(state['run_dir'])
    if run.is_dir():
        log=(run/'gazebo.log').read_text(errors='replace') if (run/'gazebo.log').exists() else ''
        crashes=[line for line in log.splitlines() if any(s in line for s in
                 ('Segmentation fault','core dumped','Stack trace','SIGSEGV'))]
        (run/'shutdown_audit.json').write_text(json.dumps(dict(
            duration_wall_s=time.monotonic()-started,sigterm_required=terminated,sigkill_required=killed,
            gazebo_crash_lines=crashes,all_owned_processes_stopped=True,
            graceful_gazebo=not crashes and 'gazebo' not in terminated and 'gazebo' not in killed),indent=2)+'\n')
    # Another stop caller may already have removed this session, followed by a
    # new start. Never unlink the replacement session's process identities.
    with session_lock():
        current=read_session()
        if current is not None and current['run_dir'] == state['run_dir']:
            PIDFILE.unlink(missing_ok=True)
    print('Stopped workspace session:', state['run_dir'])


def start(args):
    if PIDFILE.exists():
        state = json.loads(PIDFILE.read_text())
        if any(process_start(p['pid']) == p['start'] for p in state['processes']):
            raise RuntimeError('A managed session is already active; stop it first.')
    actual = subprocess.check_output(['git', '-C', str(ROOT/'external/ardupilot'), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != REVISION:
        raise RuntimeError('Firmware revision mismatch: '+actual)
    firmware=ROOT/'external/ardupilot'
    dirty=subprocess.check_output(['git','status','--porcelain','--untracked-files=no'],cwd=firmware,text=True)
    submodules=subprocess.check_output(['git','submodule','status','--recursive'],cwd=firmware,text=True)
    if dirty or any(line[0]!=' ' for line in submodules.splitlines()):
        raise RuntimeError('Firmware source or recursive submodules differ from the pinned checkout')
    lock=json.loads((ROOT/'config/dependencies.lock.json').read_text())
    plugin=ROOT/'external/ardupilot_gazebo'
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=plugin,text=True).strip()!=lock['ardupilot_gazebo']:
        raise RuntimeError('Gazebo plugin revision mismatch')
    if subprocess.check_output(['git','diff','HEAD','--binary'],cwd=plugin)!=(ROOT/'patches/ardupilot_gazebo_rover44.patch').read_bytes():
        raise RuntimeError('Gazebo plugin checkout does not match the tracked compatibility patch')
    vrx=Path(os.environ.get('VRX_ROOT','/home/zjy/vrx_docking'))
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=vrx,text=True).strip()!=lock['vrx']:
        raise RuntimeError('VRX revision mismatch')
    if args.lidar == 'mid360':
        source=ROOT/'external/livox_gz_sim'
        if subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()!=lock['livox_gz_sim']:
            raise RuntimeError('MID-360 plugin revision mismatch')
        if subprocess.check_output(['git','diff','--binary'],cwd=source)!=(ROOT/'patches/livox_gz_sim_garden.patch').read_bytes():
            raise RuntimeError('MID-360 checkout differs from the tracked Garden patch')
        if not (source/'install/lib/liblivox_gz_sim.so').exists():
            raise RuntimeError('Run bash scripts/build_mid360.sh and source scripts/env.sh first')
    for family, port in ((socket.SOCK_DGRAM, 9002), (socket.SOCK_DGRAM, 14550), (socket.SOCK_STREAM, 5760), (socket.SOCK_STREAM, 5762)):
        with socket.socket(socket.AF_INET, family) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('127.0.0.1', port))
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run = ROOT/'runs'/f'{stamp}_{args.name}'
    run.mkdir(parents=True)
    workspace_patch=subprocess.check_output(['git','diff','--binary'],cwd=ROOT)
    (run/'workspace.patch').write_bytes(workspace_patch)
    provenance={'workspace_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                'workspace_patch_sha256':hashlib.sha256(workspace_patch).hexdigest(),
                'firmware_revision':actual,'submodules':submodules.splitlines(),'files':{}}
    # Retain this run's own code, including work in progress, without third-party trees.
    paths=subprocess.check_output(['git','ls-files','-z','--cached','--others','--exclude-standard'],cwd=ROOT).decode().split('\0')
    with tarfile.open(run/'source_snapshot.tar.gz','w:gz') as archive:
        for relative in sorted(set(paths)):
            path=ROOT/relative
            if relative and path.is_file():archive.add(path,arcname=relative,recursive=False)
    provenance['source_snapshot_sha256']=hashlib.sha256((run/'source_snapshot.tar.gz').read_bytes()).hexdigest()
    for path in ('external/ardupilot/build/sitl/bin/ardurover',
                 'external/ardupilot_gazebo/build/libArduPilotPlugin.so',
                 'patches/ardupilot_gazebo_rover44.patch','config/dependencies.lock.json'):
        provenance['files'][path]=hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
    if args.task1:
        for path in ('build/task1_gazebo/libTask1Scene.so','install/robotx_task1_nav/lib/libtask1_nav.so',
                     'install/robotx_task1_nav/lib/libtask1_rotate.so'):
            provenance['files'][path]=hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
        provenance['system_libraries']={str(path):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(Path('/opt/ros/humble/lib/x86_64-linux-gnu').glob('libompl.so.*')) if not path.is_symlink()}
    if (ROOT/'build/vrx_render_fix/libWaveVisual.so').exists():
        for path in ('build/vrx_render_fix/libWaveVisual.so','patches/vrx_wavevisual_teardown.patch'):
            provenance['files'][path]=hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
    for name in ('Waves','Surface','USVWind','SimpleHydrodynamics'):
        library=vrx/'install/vrx_gz/lib'/('lib'+name+'.so')
        if library.exists():
            provenance.setdefault('system_libraries',{})[str(library)]=hashlib.sha256(library.read_bytes()).hexdigest()
    if args.lidar == 'mid360':
        for relative in ('external/livox_gz_sim/install/lib/liblivox_gz_sim.so',
                         'external/livox_gz_sim/scan_patterns/mid360.csv',
                         'patches/livox_gz_sim_garden.patch'):
            provenance['files'][relative]=hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
    (run/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    generated = run/'generated'
    command = [sys.executable, str(ROOT/'scripts/generate_world.py'), '--output', str(generated),
               '--wind', str(args.wind), '--wave-gain', str(args.wave_gain)]
    if args.lockstep:
        command.append('--lockstep')
    subprocess.run(command, check=True, stdout=(run/'generation.log').open('w'), stderr=subprocess.STDOUT)
    if args.task1:
        from prepare_task1 import prepare
        prepare(generated,args.seed,args.changes,args.dynamic,args.task1_blocked,args.task1_visibility_guard,args.course)
    if args.lidar == 'mid360':
        from prepare_mid360 import prepare as prepare_mid360
        prepare_mid360(generated,args.mid360_mount)
    if args.task1_loiter:
        if not args.task1:raise ValueError('--task1-loiter requires --task1')
        scene=json.loads((generated/'scene.json').read_text());scene['task1']['native_loiter']=True
        (generated/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    # A build that finishes during preparation invalidates the initial binary
    # hashes. Refuse to launch rather than silently record a different library.
    for relative,expected in provenance['files'].items():
        if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Artifact changed during startup preparation; finish builds before retrying: '+relative)
    state = {'run_dir': str(run), 'firmware_revision': actual, 'processes': []}

    def spawn(name, command, cwd=ROOT):
        process = subprocess.Popen(command, cwd=cwd, start_new_session=True,
                                   stdout=(run/(name+'.log')).open('w'), stderr=subprocess.STDOUT)
        state['processes'].append({'name': name, 'pid': process.pid, 'start': process_start(process.pid), 'command': command})
        write_session(state)
        time.sleep(0.4)
        if process.poll() is not None:
            raise RuntimeError(f'{name} exited: see {run}/{name}.log')

    try:
        spawn('gazebo', ['gz', 'sim', '--force-version', '7', '-s', '-r', '--headless-rendering', '-v', '3', str(generated/'usv_nav.sdf')])
        spawn('gz_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args', '-p', 'use_sim_time:=true', '-p', 'config_file:='+str(ROOT/'config/gz_bridge.yaml')])
        if args.task1:
            spawn('task1_gz_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
                  '/task1/scene_state@std_msgs/msg/String[gz.msgs.StringMsg',
                  '/task1/scene_command@std_msgs/msg/String]gz.msgs.StringMsg',
                  '/task1/beacon_view@sensor_msgs/msg/Image[gz.msgs.Image',
                  '--ros-args','-p','use_sim_time:=true'])
            spawn('task1_observer',['ros2','run','robotx_task1','known_observer','--ros-args','-p','use_sim_time:=true'])
        sitl = run/'sitl'
        sitl.mkdir()
        parameters = (ROOT/'config/rover_wamv.parm').read_text()
        if args.gps_input:
            parameters = parameters.replace('GPS_TYPE 1\n', 'GPS_TYPE 14\n')
        if args.task1_loiter:
            parameters=parameters.replace('SYSID_MYGCS 255\n','SYSID_MYGCS 252\n').replace('FS_TIMEOUT 1.5\n','FS_TIMEOUT 0.5\n')
            parameters+='\nFS_GCS_TIMEOUT 1\nLOIT_RADIUS 0.3\nLOIT_TYPE 0\nLOIT_SPEED_GAIN 0.5\nWP_SPEED 0.3\nATC_STR_RAT_MAX 23\n'
        (run/'rover.parm').write_text(parameters)
        # This exact firmware advertises --rate but has no case for it in its CLI.
        spawn('sitl', [str(ROOT/'external/ardupilot/build/sitl/bin/ardurover'), '--model', 'JSON:127.0.0.1',
                       '--home', '-33.724223,150.679736,0,90', '--speedup', '1',
                       '--defaults', str(run/'rover.parm'), '--serial0', 'tcp:5760',
                       '--serial1', 'udpclient:127.0.0.1:14550', '--serial2', 'tcp:5762'], sitl)
        spawn('mavros', ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '--params-file', str(ROOT/'config/mavros.yaml')])
        if args.gps_input:
            spawn('gnss', ['ros2','run','usv_sim','gnss_injector','--ros-args','-p','use_sim_time:=true'])
    except Exception:
        stop(expected_run=run)
        raise
    (ROOT/'runs/latest.txt').write_text(str(run)+'\n')
    print(run, flush=True)
    if args.navigation:
        navigation(args)
    # ArduPilot deliberately exits when its parent exits. Keep a supervisor alive.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        while (read_session() or {}).get('run_dir') == str(run):
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop(expected_run=run)


def navigation(args):
    state = json.loads(PIDFILE.read_text())
    run = Path(state['run_dir'])
    generated = run/'generated'
    names={'navigation','lidar_tf','cloud_filter','cloud_to_scan','map_origin','task1_mission','task1_judge','traffic_guard'}
    existing=[p for p in state['processes'] if p['name'] in names and process_start(p['pid'])==p['start']]
    if existing and not args.replace:
        raise RuntimeError('Navigation already attached; use --replace to restart that layer.')
    if existing:
        for process in existing:os.killpg(process['pid'],signal.SIGINT)
        time.sleep(3)
        state['processes']=[p for p in state['processes'] if p['name'] not in names]
    subprocess.run([sys.executable, str(ROOT/'scripts/prepare_navigation.py'), str(generated/'scene.json')], check=True)
    task1='task1' in json.loads((generated/'scene.json').read_text())
    scene=json.loads((generated/'scene.json').read_text())
    if task1:
        from prepare_task1 import navigation as task_navigation
        task_navigation(generated)
    provenance=json.loads((run/'provenance.json').read_text())
    provenance['generated_files']={str(path.relative_to(run)):hashlib.sha256(path.read_bytes()).hexdigest()
                                   for path in generated.iterdir() if path.is_file()}
    (run/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    commands = {
        'map_origin': ['ros2','run','usv_sim','map_origin','--ros-args','-p','use_sim_time:=true'],
        'lidar_tf': ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '0.7', '--y', '-0.3', '--z', '1.8',
                     '--frame-id', 'base_link', '--child-frame-id', 'wamv/wamv/base_link/lidar_wamv_sensor',
                     '--ros-args', '-p', 'use_sim_time:=true'],
        'cloud_filter': ['ros2', 'run', 'usv_sim', 'cloud_filter', '--ros-args', '-p', 'use_sim_time:=true',
                         '-p', 'scene_file:='+str(generated/'scene.json')],
        'cloud_to_scan': ['ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', '--ros-args',
                          '--params-file', str(ROOT/'config/cloud_to_scan.yaml'), '-r', 'cloud_in:=/usv/lidar/filtered', '-r', 'scan:=/usv/scan_raw'],
        'navigation': ['ros2', 'launch', 'usv_nav2', 'navigation.launch.py', 'map:='+str(generated/'water.yaml'),
                       'params_file:='+str(generated/'nav2_wamv.yaml'), 'use_sim_time:=true', 'rviz:=false',
                       'enable_control:='+str(args.control).lower()],
    }
    if task1:
        commands['cloud_filter']+=['-p','publish_clearing_cloud:=true']
        footprint=json.loads((generated/'scene.json').read_text())['footprint']
        dynamics=json.loads((generated/'nav2_wamv.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
        speed=dynamics['max_vel_x']
        commands['traffic_guard']=['ros2','run','robotx_task1','traffic_guard','--ros-args','-p','use_sim_time:=true',
                                   '-p','footprint:='+json.dumps(json.dumps(footprint)),'-p','speed_bound:='+str(speed),
                                   '-p','route_yaw_rate:='+str(dynamics['max_vel_theta']),
                                   '-p','route_acceleration:='+str(dynamics['acc_lim_x']),
                                   '-p','route_deceleration:='+str(abs(dynamics['decel_lim_x'])),
                                   '-p','run_dir:='+str(run)]
        commands['task1_mission']=['ros2','run','robotx_task1','mission','--ros-args','-p','use_sim_time:=true','-p','run_dir:='+str(run)]
        if scene['task1'].get('native_loiter',False):commands['task1_mission']+=['-p','enable_stationkeeping:=true']
        commands['task1_judge']=['ros2','run','robotx_task1','judge','--ros-args','-p','use_sim_time:=true','-p','run_dir:='+str(run)]
    if scene.get('lidar_model') == 'mid360':
        commands['lidar_tf']=['ros2','run','tf2_ros','static_transform_publisher',
                             '--x',str(scene['lidar_xyz'][0]),'--y',str(scene['lidar_xyz'][1]),'--z',str(scene['lidar_xyz'][2]),'--frame-id','base_link',
                             '--child-frame-id','mid360_frame','--ros-args','-p','use_sim_time:=true']
        commands['cloud_filter']=['ros2','run','usv_sim','livox_filter','--ros-args',
                                 '-p','use_sim_time:=true','-p','footprint:='+json.dumps(json.dumps(scene['footprint']))]
    for name, command in commands.items():
        previous_log=run/(name+'.log')
        if previous_log.exists():
            previous_log.rename(run/(name+'.'+datetime.datetime.now().strftime('%H%M%S')+'.log'))
        process = subprocess.Popen(command, cwd=ROOT, start_new_session=True,
                                   stdout=(run/(name+'.log')).open('w'), stderr=subprocess.STDOUT)
        state['processes'].append({'name': name, 'pid': process.pid, 'start': process_start(process.pid), 'command': command})
        write_session(state)
    print('Attached navigation, control='+str(args.control), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['start', 'stop', 'status', 'navigation'])
    parser.add_argument('--name', default='baseline')
    parser.add_argument('--wind', type=float, default=0)
    parser.add_argument('--wave-gain', type=float, default=0)
    parser.add_argument('--lidar', choices=('vrx','mid360'), default='vrx')
    parser.add_argument('--mid360-mount',choices=('mast','bow','low_bow'),default='mast')
    parser.add_argument('--task1-loiter',action='store_true')
    parser.add_argument('--lockstep', action='store_true')
    parser.add_argument('--navigation', action='store_true')
    parser.add_argument('--control', action='store_true')
    parser.add_argument('--gps-input', action='store_true')
    parser.add_argument('--replace', action='store_true')
    parser.add_argument('--task1', action='store_true')
    parser.add_argument('--task1-blocked', action='store_true', help='Synthetic full-width wall for the no-path safety test')
    parser.add_argument('--task1-visibility-guard', action='store_true', help='Experimental lidar stand-off; observed to stall, disabled by default')
    parser.add_argument('--seed', type=int, choices=(1,2,3), default=1)
    parser.add_argument('--course', choices=('baseline','harbor'), default='baseline')
    parser.add_argument('--changes', type=int, choices=(0,1,2), default=0)
    parser.add_argument('--dynamic', choices=('none','cross','block'), default='none')
    args = parser.parse_args()
    if args.action == 'start':
        start(args)
    elif args.action == 'stop':
        stop()
    elif args.action == 'navigation':
        navigation(args)
    elif PIDFILE.exists():
        print(PIDFILE.read_text())
    else:
        print('No managed session.')


if __name__ == '__main__':
    main()
