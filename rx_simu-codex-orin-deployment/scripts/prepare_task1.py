#!/usr/bin/env python3
"""Adapt a newly generated independent VRX world to the 2026 Safe Passage task."""
import argparse
import json
import math
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as E

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/usv_nav2'))
from usv_nav2.make_water_map import write_map


def prepare(out, seed=1, changes=0, dynamic='none', blocked=False, visibility_guard=False, course='baseline'):
    from generate_world import element, set_text, box
    out=Path(out);rng=random.Random(seed)
    defaults=json.loads((ROOT/'config/task1/defaults.json').read_text())
    buoys=[dict(id='entry',x=10.,y=0.,state=3,radius=.36),dict(id='exit',x=60.,y=0.,state=4,radius=.36)]
    for i,x in enumerate((24.,32.,40.,48.)):
        for j,y in enumerate((7.,-7.)):
            # Stable seed 1 is the straight reference; seeds 2/3 are staggered.
            dx=0 if seed==1 else rng.uniform(-1.,1.)
            dy=0 if seed==1 else rng.uniform(-2.,2.)
            buoys.append(dict(id=f'buoy_{i*2+j+1:02}',x=x+dx,y=y+dy,state=2 if j==0 else 1,radius=.36))
    # An inactive light remains a physical/costmap obstacle in the third layout.
    if seed==3:next(b for b in buoys if b['id']=='buoy_07')['state']=0
    events=[{'after_s':20.,'id':'buoy_03','state':1}, {'after_s':65.,'id':'buoy_06','state':2}][:changes]
    infrastructure=[];traffic=None
    if course=='harbor':
        if changes not in (0,1):raise ValueError('Harbor match tests support zero or one semantic change')
        harbor=json.loads((ROOT/'config/task1/harbor.json').read_text());layout=harbor['layouts'][str(seed)]
        buoys=[dict(id=key,x=layout[key][0],y=layout[key][1],state=state,radius=.36) for key,state in [('entry',3),('exit',4)]]
        buoys += [dict(id=f'buoy_{i+1:02}',x=x,y=y,state=state,radius=.36) for i,(x,y,state) in enumerate(layout['markers'])]
        events=[layout['change']] if changes else [];infrastructure=harbor['infrastructure'];traffic=layout['traffic']
        defaults['boundary']=harbor['boundary']
    scenario={'seed':seed,'tier':'disruptive','buoys':buoys,'changes':events,'dynamic':dynamic,
              'boundary':defaults['boundary'],'circle_radius':defaults['circle_radius_m'],'blocked_fixture':blocked,
              'experimental_visibility_guard':visibility_guard,'course':course,'infrastructure':infrastructure}
    if traffic:scenario['dynamic_motion']=traffic
    (out/'task1_scenario.json').write_text(json.dumps(scenario,indent=2)+'\n')
    tree=E.parse(out/'usv_nav.sdf');world=tree.getroot().find('world')
    for n in list(world.findall('include')):
        if n.findtext('uri')!='coast_waves':world.remove(n)
    for n in list(world.findall('model')):
        if n.get('name') in ('static_obstacle','dynamic_obstacle'):world.remove(n)
    template=E.parse(ROOT/'models/robotx_2026_robobuoy/model.sdf').getroot().find('model')
    for b in buoys:
        m=E.fromstring(E.tostring(template));m.set('name',b['id'])
        element(m,'pose',f"{b['x']} {b['y']} 0 0 0 0");world.append(m)
    if dynamic!='none':
        if course=='harbor':
            m=E.parse(ROOT/'models/task1_workboat/model.sdf').getroot().find('model');m.set('name','task1_moving_obstacle')
            element(m,'pose',f"{traffic['x']} {traffic['y_min']} .45 0 0 0");world.append(m)
        else:box(world,'task1_moving_obstacle',28,-14,sx=2.,sy=3.,height=.9)
    for body in infrastructure:
        box(world,body['name'],body['x'],body['y'],*body['size'],height=body['height'])
        material=world.find(f"model[@name='{body['name']}']//visual/material")
        set_text(material,'diffuse','0.35 0.38 0.42 1');set_text(material,'ambient','0.35 0.38 0.42 1')
    if blocked:box(world,'task1_no_path_wall',4,0,sx=1.,sy=100.,height=.9)
    p=element(world,'plugin',filename='libTask1Scene.so',name='Task1Scene')
    element(p,'scenario',str(out/'task1_scenario.json'))
    observer=world.find("model[@name='overview_camera']")
    camera_pose=[35.,0.,58.,0.,math.pi/2,math.pi/2]
    set_text(observer,'pose',' '.join(map(str,camera_pose)))
    set_text(observer.find('.//sensor'),'update_rate',5)
    observer.find('.//camera/horizontal_fov').text='1.3'
    observer.find('.//camera/image/width').text='1600'
    observer.find('.//camera/image/height').text='1000'
    close=E.fromstring(E.tostring(observer));close.set('name','beacon_camera')
    set_text(close,'pose',f"{buoys[0]['x']} {buoys[0]['y']} 2 0 1.57079632679 0")
    set_text(close.find('.//sensor'),'topic','/task1/beacon_view')
    world.append(close)
    E.indent(tree);tree.write(out/'usv_nav.sdf',encoding='utf-8',xml_declaration=True)
    scene=json.loads((out/'scene.json').read_text())
    scene['task1']=scenario
    scene['overview_camera']={'pose':camera_pose,'width':1600,'height':1000,'horizontal_fov':1.3}
    scene['obstacles']=[dict(name=b['id'],x=b['x'],y=b['y'],size=[.508,.508]) for b in buoys]
    if dynamic!='none':scene['obstacles'].append(dict(name='task1_moving_obstacle',x=traffic['x'] if traffic else 28.,y=traffic['y_min'] if traffic else -14.,size=[2.,3.]))
    scene['obstacles']+=infrastructure
    if blocked:scene['obstacles'].append(dict(name='task1_no_path_wall',x=4.,y=0.,size=[1.,100.]))
    (out/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    return scenario


def voxel_obstacles():
    # Keep height while clearing: a ray over a low object's roof is not proof
    # that its entire 2D column is empty. Only actual returns mark occupancy.
    common=dict(data_type='PointCloud2',observation_persistence=0.,expected_update_rate=.5,
                obstacle_min_range=0.,obstacle_max_range=15.,raytrace_min_range=0.,raytrace_max_range=20.)
    return dict(plugin='nav2_costmap_2d::VoxelLayer',enabled=True,combination_method=1,
                footprint_clearing_enabled=False,origin_z=-.2,z_resolution=.2,z_voxels=16,
                unknown_threshold=15,mark_threshold=0,max_obstacle_height=3.,
                observation_sources='returns rays',
                returns=dict(common,topic='/usv/lidar/marking',marking=True,clearing=False,
                             min_obstacle_height=-100.,max_obstacle_height=100.),
                rays=dict(common,topic='/usv/lidar/clearing',marking=False,clearing=True,
                          min_obstacle_height=-100.,max_obstacle_height=100.))


def navigation(out):
    out=Path(out);scene=json.loads((out/'scene.json').read_text());scenario=scene['task1']
    params=json.loads((out/'nav2_wamv.yaml').read_text())
    planner=params['planner_server']['ros__parameters']
    planner['planner_plugins']=['GridBased','Task1']
    planner['Task1']={'plugin':'robotx_task1_nav/RulePlanner','search_time_limit':10.,'continuity_weight':.2}
    # Remain outside the near-field blind zone for the lowest marking height.
    # Fixed surveyed buoy bodies still use their normal physical collision checks.
    clearance=0.
    if scenario.get('experimental_visibility_guard',False):
        clearance=(scene['lidar_xyz'][2]-.25)/math.tan(abs(scene['lidar_vertical_min_rad']))+.2
    visibility=dict(visibility_clearance=clearance,lidar_x=scene['lidar_xyz'][0],lidar_y=scene['lidar_xyz'][1])
    planner['Task1'].update(visibility)
    controller=params['controller_server']['ros__parameters']
    controller['speed_limit_topic']='/task1/speed_limit'
    # The boat still drifts below 0.03 m/s. Preserve that motion in DWB's
    # initial state and allow for the measured, slower braking response.
    controller['min_x_velocity_threshold']=.005
    controller['min_y_velocity_threshold']=.005
    controller['min_theta_velocity_threshold']=.005
    controller['FollowPath']['decel_lim_x']=-.08
    controller['FollowPath']['decel_lim_theta']=-.2
    controller['FollowPath']['sim_time']=5.
    controller['FollowPath']['critics'].insert(0,'Task1Rule')
    controller['FollowPath']['critics'].insert(0,'Task1Hull')
    controller['FollowPath']['Task1Hull.class']='robotx_task1_nav::FullHullCritic'
    controller['FollowPath']['Task1Hull.scale']=1.0
    controller['FollowPath'].update({'Task1Hull.'+k:v for k,v in visibility.items()})
    controller['FollowPath']['Task1Rule.class']='robotx_task1_nav::RuleCritic'
    controller['FollowPath']['Task1Rule.scale']=1.0
    controller['FollowPath']['RotateToGoal.class']='robotx_task1_nav::RecoverableRotateCritic'
    # Load the standard critic library before its subclass; avoid pluginlib
    # registering transitive factories under the wrong owning ClassLoader.
    critics=controller['FollowPath']['critics'];critics.remove('RotateToGoal');critics.insert(3,'RotateToGoal')
    controller['failure_tolerance']=.5
    for name in ('local_costmap','global_costmap'):
        costmap=params[name][name]['ros__parameters']
        if name=='global_costmap':
            # Leave room for tracking/braking before the controller's hard
            # hull limit. Measured stops use ~0.33 m; the 0.2 m vs 0.05 m
            # grids add ~0.11 m of cell-radius discrepancy. The extra 0.5 m
            # planning reserve prevents a legal local stop from trapping the
            # coarser global planner at an occupied start near a fixed buoy.
            local_padding=params['local_costmap']['local_costmap']['ros__parameters']['footprint_padding']
            costmap['footprint_padding']=max(costmap['footprint_padding'],local_padding+.5)
        # Self returns are removed by cloud_filter using the physical hull.
        # Clearing the padded footprint here erases external returns before
        # physical contact, including a moving obstacle entering the margin.
        costmap['obstacle_layer']=voxel_obstacles()
        if scene.get('lidar_model')=='mid360':
            # Livox publishes several source-timed clearing groups per frame.
            # A zero-duration buffer keeps only the last group, losing lower
            # rays. Retain 0.2 s of measured rays; marking still uses newest data.
            costmap['obstacle_layer']['rays']['observation_persistence']=.2
        costmap['plugins'].insert(-1,'known_buoys')
        costmap['known_buoys']={'plugin':'robotx_task1_nav/KnownBuoys'}
        costmap['plugins'].insert(-1,'traffic_prediction')
        costmap['traffic_prediction']=dict(plugin='robotx_task1_nav/TrafficLayer',
            map_topic='/task1/traffic_costmap',map_subscribe_transient_local=True,
            subscribe_to_updates=False,footprint_clearing_enabled=False)
    params['rover_bridge']['ros__parameters']['require_task_permission']=True
    params['rover_bridge']['ros__parameters']['enable_stationkeeping']=scenario.get('native_loiter',False)
    (out/'nav2_wamv.yaml').write_text(json.dumps(params,indent=2)+'\n')
    area={'resolution':.2,'padding':1.,'navigable_boundary':scenario['boundary'],'exclusions':[]}
    for body in scenario.get('infrastructure',[]):
        x,y=body['x'],body['y'];sx,sy=body['size']
        area['exclusions'].append([[x-sx/2,y-sy/2],[x+sx/2,y-sy/2],[x+sx/2,y+sy/2],[x-sx/2,y+sy/2]])
    if scenario.get('blocked_fixture'):area['exclusions']=[[[3.5,-50],[4.5,-50],[4.5,50],[3.5,50]]]
    write_map(area,out/'water',overwrite=True);(out/'water_area.json').write_text(json.dumps(area,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--seed',type=int,default=1)
    p.add_argument('--changes',type=int,choices=(0,1,2),default=0);p.add_argument('--dynamic',choices=('none','cross','block'),default='none')
    p.add_argument('--course',choices=('baseline','harbor'),default='baseline')
    a=p.parse_args();prepare(a.output,a.seed,a.changes,a.dynamic,course=a.course)
