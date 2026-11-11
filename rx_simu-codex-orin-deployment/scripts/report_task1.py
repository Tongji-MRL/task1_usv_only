#!/usr/bin/env python3
"""Summarize retained attempts without replacing failed evidence or inventing passes."""
import hashlib
import json
import tarfile
from pathlib import Path
from task1_cases import cases,case_key,FAULTS

ROOT=Path(__file__).resolve().parents[1]
RUNTIME_SOURCES=[
    'scripts/prepare_task1.py','scripts/prepare_navigation.py','scripts/generate_world.py',
    'scripts/sim_session.py','src/usv_sim/usv_sim/cloud_filter.py','src/usv_sim/usv_sim/clearing_cloud.py',
    'src/robotx_task1/robotx_task1/mission.py','src/robotx_task1/robotx_task1/core.py',
    'src/robotx_task1/robotx_task1/traffic.py','src/robotx_task1/robotx_task1/traffic_guard.py',
    'src/robotx_task1/robotx_task1/route_policy.py','src/robotx_task1_interfaces/msg/RouteAssessment.msg',
    'src/robotx_task1_interfaces/msg/MissionContext.msg',
    'src/robotx_task1_interfaces/msg/TrafficSafety.msg',
    'src/robotx_task1/robotx_task1/observer.py','src/usv_nav2/usv_nav2/rover_bridge.py',
    'src/usv_nav2/launch/navigation.launch.py','src/usv_nav2/usv_nav2/launch_config.py',
    'src/usv_nav2/usv_nav2/task_lease.py',
    'src/robotx_task1_nav/src/plugins.cpp','src/robotx_task1_nav/src/rotate.cpp',
    'src/robotx_task1_nav/include/robotx_task1_nav/rules.hpp',
    'src/robotx_task1_nav/include/robotx_task1_nav/dubins_estimate.hpp',
    'src/robotx_task1_nav/include/robotx_task1_nav/hull_grid.hpp',
    'src/robotx_task1_nav/include/robotx_task1_nav/route_preference.hpp',
    'src/robotx_task1_nav/include/robotx_task1_nav/traffic_layer.hpp','src/robotx_task1_nav/plugins.xml',
    'tools/task1_gazebo/Task1Scene.cc','models/robotx_2026_robobuoy/model.sdf',
    'config/task1/harbor.json','models/task1_workboat/model.sdf']


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    attempts=[];latest={};models=[];lidars=[]
    for run in sorted((ROOT/'runs').iterdir()):
        if not run.is_dir():continue
        for filename,collection in [('model_validation.json',models),('lidar_validation.json',lidars)]:
            p=run/filename
            if p.exists():
                j=json.loads(p.read_text());collection.append(dict(run=str(run.relative_to(ROOT)),passed=j.get('passed'),sha256=digest(p)))
        source=run/'generated/task1_scenario.json';result=run/'task1_result.json'
        if not source.exists() or not result.exists():continue
        scenario=json.loads(source.read_text());j=json.loads(result.read_text());fault=run/'task1_fault_result.json'
        scene=json.loads((run/'generated/scene.json').read_text())
        key=case_key(dict(seed=scenario['seed'],changes=len(scenario['changes']),dynamic=scenario['dynamic'],
                          course=scenario.get('course','baseline'),wind=scene.get('wind_mps',0),wave_gain=scene.get('wave_gain',0)))
        parameters=run/'generated/nav2_wamv.yaml'
        config=json.loads(parameters.read_text()) if parameters.exists() else {}
        guard=config.get('controller_server',{}).get('ros__parameters',{}).get('FollowPath',{}).get('Task1Hull.visibility_clearance',0.)
        verdict=j.get('passed',False);f=None
        map_result=None
        buoy_map=None
        if not fault.exists() and (run/'visualization/raw/events.jsonl').exists():
            buoy_file=run/'buoy_map_result.json'
            if buoy_file.exists():buoy_map=json.loads(buoy_file.read_text())
            else:
                from analyze_task1_maps import buoy_persistence
                buoy_map=buoy_persistence(run)
            verdict=verdict and buoy_map.get('passed',False)
        if scenario['dynamic']!='none' and not fault.exists():
            map_file=run/'dynamic_map_result.json'
            if map_file.exists():map_result=json.loads(map_file.read_text())
            else:
                from analyze_task1_maps import analyze
                map_result=analyze(run)
            verdict=verdict and map_result.get('passed',False)
        if fault.exists():
            f=json.loads(fault.read_text());key='fault_'+f['fault'];verdict=f.get('passed',False)
        if guard>0:key+='_visibility_guard'
        item=dict(case=key,run=str(run.relative_to(ROOT)),passed=verdict,normal_task_passed=j.get('passed',False),
                  endpoint=j.get('endpoint_error'),clearance_m=j.get('min_clearance_m'),collisions=j.get('collision_contacts'),
                  circles=j.get('circles'),stops=j.get('stops'),fault=f,
                  semantic_timing=j.get('semantic_timing'),drift_m=j.get('post_completion_drift_m'),
                  post_completion_record_s=j.get('post_completion_record_s'),
                  buoy_map=buoy_map,
                  dynamic_map={k:v for k,v in map_result.items() if k!='samples'} if map_result else None,
                  evidence_sha256={p.name:digest(p) for p in [result,source,run/'provenance.json',
                                    run/'buoy_map_result.json',run/'dynamic_map_result.json'] if p.exists()})
        if fault.exists():item['evidence_sha256'][fault.name]=digest(fault)
        mission_log=run/'task1_mission.jsonl'
        item['task_started']=False
        if mission_log.exists():
            events=[json.loads(l) for l in mission_log.read_text().splitlines()]
            item['task_started']=any(e['event']=='started' for e in events)
            planning=[]
            for i,e in enumerate(events):
                if e['event']!='planning':continue
                outcome=next((x for x in events[i+1:] if x['generation']==e['generation'] and
                              x['event'] in ('path_candidate','path_ready','plan_failed','stop_requested','aborted')),None)
                if outcome:planning.append(dict(outcome=outcome['event'],sim_s=outcome['sim']-e['sim'],
                                                wall_s=outcome['wall']-e['wall']))
            item['execution_metrics']={
                'planning_attempts':sum(e['event']=='planning' for e in events),
                'planning_failures':sum(e['event']=='plan_failed' for e in events),
                'controller_failures':sum(e['event']=='controller_result' and e['status'] not in (4,5) for e in events),
                'candidate_traffic_rejections':sum(e['event']=='path_blocked_traffic' for e in events),
                'reported_semantic_changes':sum(e['event']=='semantic_update_reported' for e in events),
                'planning_duration_samples':planning,
                'total_planning_wall_s':sum(p['wall_s'] for p in planning),
                'max_planning_wall_s':max((p['wall_s'] for p in planning),default=None),
                'semantic_pause_sim_s':[s['resume_sim']-s['start_sim'] for s in j.get('stops',[])
                                        if s['reason']=='semantic revision changed' and s['resume_sim'] is not None]}
        item['outcome']='passed' if verdict else 'collision' if j.get('collision_contacts',0)>0 else 'not_started' if not item['task_started'] and not f else 'failed'
        snapshot=run/'source_snapshot.tar.gz'
        if snapshot.exists():
            with tarfile.open(snapshot) as archive:
                item['runtime_source_sha256']={name:hashlib.sha256(archive.extractfile(name).read()).hexdigest()
                                               for name in RUNTIME_SOURCES if name in archive.getnames()}
        item['changed_runtime_sources']=[name for name in RUNTIME_SOURCES if (ROOT/name).exists() and
                                        item.get('runtime_source_sha256',{}).get(name)!=digest(ROOT/name)]
        item['matches_current_runtime_sources']=not item['changed_runtime_sources']
        if parameters.exists():
            controller=config['controller_server']['ros__parameters']
            item['controller_parameters']={key:controller['FollowPath'].get(key) for key in
                                           ('sim_time','decel_lim_x','decel_lim_theta','max_vel_x','max_vel_theta')}
            item['controller_parameters']['visibility_clearance_m']=controller['FollowPath'].get('Task1Hull.visibility_clearance',0.)
            item['obstacle_layer_plugin']=config['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['plugin']
        attempts.append(item);latest[key]=item
    normal=[case_key(c) for c in cases()]
    match=[case_key(c) for c in cases() if c['changes']==1]
    baseline=[case_key(c) for c in cases() if c['changes']==0]
    fault_keys=['fault_'+f for f in FAULTS]
    expected=normal+fault_keys
    stress=[case_key(c) for c in cases(stress=True) if case_key(c) in latest]
    harbor=[k for k in latest if k.startswith('harbor_')]
    competition_coverage={label:sum(latest.get(k,{}).get('outcome')==label for k in match)
                          for label in ('passed','collision','not_started','failed')}
    competition_coverage['not_attempted']=sum(k not in latest for k in match)
    harbor_coverage={label:sum(latest[k]['outcome']==label for k in harbor)
                     for label in ('passed','collision','not_started','failed')}
    summary={'expected':expected,'competition_cases':match,'baseline_cases':baseline,'recorded_stress_cases':stress,'recorded_harbor_cases':harbor,
             'competition_coverage':competition_coverage,
             'harbor_coverage':harbor_coverage,
             'competition_change_count':1,'constraint_source':'User confirmation on 2026-11-01; blinking phases are not semantic changes.',
             'latest':latest,'attempts':attempts,'model_checks':models,'lidar_checks':lidars,
             'scope':'Boat-side simulation; known UAV observation and local reporting substitutes; no official score claimed.'}
    out=ROOT/'docs/task1';(out/'results.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    lines=['# Task 1 实际验证记录','',
           '由 `scripts/report_task1.py` 根据保留的独立裁判结果生成。完整历史（包括调试失败）见 [results.json](results.json)。',
           '每项展示最近一次已完成的尝试，可能来自较早提交；运行目录的 provenance.json 与源码快照标识当时版本。新修改不能继承旧运行的通过结论。',
           '根据用户 2026-11-01 确认，正式比赛仅一次语义灯态切换；闪烁亮灭不计次数。一次切换为主验收，零切换为对照，两次切换仅作额外压力测试。',
           '比赛主验收最近记录：'+str(competition_coverage['passed'])+' 项通过，'+str(competition_coverage['collision'])+' 项接触碰撞，'+str(competition_coverage['not_started'])+' 项启动失败、任务未开始；启动失败不能算作已完成场景验证。',
           '独立港区工程场景最近记录：'+str(harbor_coverage['passed'])+' 项通过，'+str(harbor_coverage['collision'])+' 项接触碰撞；不替代上面的 baseline 矩阵。',
           '正常任务与故障测试分开判定；程序报告 COMPLETE 不等于独立验收通过。尚未运行的组合明确列出。','',
           '| 范围 | 用例 | 判定 | 行为源码版本 | 位置 / 航向误差 | 最小净空 | 证据 |',
           '|---|---|---|---|---|---|---|']
    for key in expected+stress+harbor:
        category='港区工程测试' if key in harbor else '比赛主验收' if key in match else '零切换对照' if key in baseline else '额外压力测试' if key in stress else '异常响应'
        r=latest.get(key)
        if not r:lines.append(f'| {category} | {key} | 未验证 | — | — | — | — |');continue
        endpoint=r.get('endpoint')
        errors=f"{endpoint['position_m']:.3f} m / {endpoint['yaw_rad']:.3f} rad" if endpoint else '不适用／未完成'
        clearance='—' if r['clearance_m'] is None else f"{r['clearance_m']:.3f} m"
        file='task1_fault_result.json' if r['fault'] else 'task1_result.json'
        version='当前' if r['matches_current_runtime_sources'] else '较早；需回归'
        label={'passed':'通过','collision':'失败：接触碰撞','not_started':'启动失败：任务未开始','failed':'失败'}[r['outcome']]
        lines.append(f"| {category} | {key} | {label} | {version} | {errors} | {clearance} | [{Path(r['run']).name}](../../{r['run']}/{file}) |")
    experiments=[r for key,r in latest.items() if key.endswith('_visibility_guard')]
    if experiments:
        lines+=['','额外观测距离为非默认实验，不替代上面的默认配置验收：','']
        for r in experiments:
            lines.append(f"- `{r['case']}`：{'通过' if r['passed'] else '失败'}；[独立结果](../../{r['run']}/task1_result.json)。")
    late=latest.get('fault_late_change',{}).get('fault')
    if late:
        motion=late.get('status_after_fault',{}).get('motion','未知')
        lines+=['',f'临近浮标灯变测试只测注入后 10 s 的停止和规则响应；记录末端运动状态为 `{motion}`。'
                '若为 STOPPING / PLANNING / WAIT_CLEAR，只能证明安全停等，不能声称已恢复航行或完成整场。']
    lines+=['','模型和感知检查：']
    for name,items,file in [('相机灯态/实体',models,'model_validation.json'),('LiDAR 三层回波',lidars,'lidar_validation.json')]:
        if items:
            r=items[-1];lines+=['',f"- {name}：{'通过' if r['passed'] else '失败'}；[最近证据](../../{r['run']}/{file})。"]
        else:lines+=['',f'- {name}：未验证。']
    lines+=['','固定固件、规则及范围见 [implementation.md](implementation.md)，启动命令见 [runbook.md](runbook.md)，根因与修复见 [debug_log.md](debug_log.md)。',
            'WAM-V、固定系泊模型和本地报告不能代表正式 UAV 协同、RoboCommand 接入或实船比赛得分。']
    (out/'validation.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({'normal_passed':sum(latest[k]['passed'] for k in normal if k in latest),
                      'competition_passed':sum(latest[k]['passed'] for k in match if k in latest),
                      'harbor_coverage':harbor_coverage,
                      'current_normal_passed':sum(latest[k]['passed'] and latest[k]['matches_current_runtime_sources'] for k in normal if k in latest),
                      'fault_passed':sum(latest[k]['passed'] for k in fault_keys if k in latest),
                      'not_started':[k for k in normal if latest.get(k,{}).get('outcome')=='not_started'],
                      'unverified':[k for k in expected if k not in latest],'attempts':len(attempts)},ensure_ascii=False))


if __name__=='__main__':main()
