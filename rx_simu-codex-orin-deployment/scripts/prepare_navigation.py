#!/usr/bin/env python3
"""Create a WAM-V-specific configuration without changing the small-boat baseline."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/usv_nav2'))
from usv_nav2.make_water_map import write_map


def prepare(scene_file):
    scene_file = Path(scene_file)
    scene = json.loads(scene_file.read_text())
    out = scene_file.parent
    config = json.loads((ROOT/'src/usv_nav2/config/nav2_params.yaml').read_text())
    for key in ('local_costmap', 'global_costmap'):
        params = config[key][key]['ros__parameters']
        params.update(footprint=json.dumps(scene['footprint']), footprint_padding=0.5)
        params['inflation_layer'].update(inflation_radius=4.5, cost_scaling_factor=1.0)
        # The guarded scan supplies +inf only for observed no-hit directions.
        params['obstacle_layer']['scan']['inf_is_valid'] = True
    # Allow a large differential-thrust hull enough space on the global path.
    config['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.5
    # At 0.2 m/s² and 10 Hz, the first reachable speed is only 0.02 m/s.
    # A 3 s rollout moves 0.06 m; 0.1 m cells made slow trajectories tie and stall.
    config['local_costmap']['local_costmap']['ros__parameters']['resolution'] = 0.05
    # Stationary EKF velocity noise reached 0.02 m/s during cold starts. Treat it
    # as zero for DWB sampling so a negative estimate does not collapse its window.
    config['controller_server']['ros__parameters']['min_x_velocity_threshold'] = 0.03
    config['controller_server']['ros__parameters']['FollowPath'].update(
        # Predict acceleration over the horizon. The 0.1 s LimitedAccel window
        # repeatedly stalled at cell boundaries with noisy near-zero EKF speed.
        # ArduRover's ATC_ACCEL/DECEL_MAX still enforces the measured 0.2 m/s² limit.
        trajectory_generator_name='dwb_plugins::StandardTrajectoryGenerator',
        max_vel_x=0.3, max_speed_xy=0.3,
        sim_time=3.0, publish_evaluation=True, debug_trajectory_details=True,
        xy_goal_tolerance=0.4)
    # A moving obstacle was observed correctly but the original soft clearance
    # weight allowed contact. Reserve more hull clearance and shorten stopping distance.
    config['controller_server']['ros__parameters']['FollowPath']['ObstacleFootprint.scale'] = 1.0
    config['rover_bridge']['ros__parameters']['max_speed'] = 0.3
    config['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'] = 0.4
    config['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance'] = 0.2
    config['odom_tf']['ros__parameters']['expected_parent_frame'] = 'odom'
    config['rover_bridge']['ros__parameters']['expected_parent_frame'] = 'odom'
    (out/'nav2_wamv.yaml').write_text(json.dumps(config, indent=2)+'\n')
    area = {'resolution': 0.2, 'padding': 1.0,
            'navigable_boundary': [[-50,-50],[50,-50],[50,50],[-50,50]],
            # Static known obstacle plus distant dock safety exclusion.
            'exclusions': [[[9,-1],[11,-1],[11,1],[9,1]],
                           [[25,20],[45,20],[45,40],[25,40]]]}
    if not (out/'water.yaml').exists():
        write_map(area, out/'water')
    (out/'water_area.json').write_text(json.dumps(area, indent=2)+'\n')
    return out


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('scene_file')
    args=parser.parse_args()
    print(prepare(args.scene_file))
