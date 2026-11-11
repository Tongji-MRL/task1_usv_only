"""Compare spatial routes using arrival intervals and measured traffic only.

This advisory never grants motion permission and never clears a costmap cell.
The independent short-range TrafficSafety lease remains mandatory. Long-range
constant-velocity estimates are refreshed while stopped; possible target stops
add a cost rather than being treated as a known future trajectory.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class RouteConfig:
    speed: float = .3
    yaw_rate: float = .4
    acceleration: float = .2
    deceleration: float = .08
    terminal_distance: float = 4.
    terminal_ratio: float = .5
    horizon: float = 90.
    max_wait: float = 60.
    wait_step: float = 5.
    timing_fraction: float = .2
    margin: float = .6
    switch_base: float = 10.
    switch_distance_weight: float = 2.
    improvement: float = 10.
    replan_time: float = 5.
    stop_risk_cost: float = 30.

    def __post_init__(self):
        for name in ('speed','yaw_rate','acceleration','deceleration','horizon','wait_step','terminal_distance','terminal_ratio'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<=0:
                raise ValueError('Invalid route '+name)
        if self.terminal_ratio>1.:raise ValueError('Terminal speed cannot exceed cruise speed')
        for name in ('max_wait','timing_fraction','margin','switch_base','switch_distance_weight',
                     'improvement','replan_time','stop_risk_cost'):
            if not math.isfinite(getattr(self,name)) or getattr(self,name)<0:
                raise ValueError('Invalid route '+name)


def wrap(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))


def terminal_scale(distance, config=RouteConfig()):
    """Same final-approach speed policy for DWB and the arrival predictor."""
    return np.where(np.asarray(distance)<=config.terminal_distance,config.terminal_ratio,1.)


def remaining_path(path, pose, spacing=.4):
    """Use heading to avoid snapping to the opposite leg of a loop."""
    path=np.asarray(path,dtype=float);pose=np.asarray(pose,dtype=float)
    if path.ndim!=2 or path.shape[1]!=3 or not len(path) or not np.isfinite(path).all():
        raise ValueError('Invalid route poses')
    distance=np.linalg.norm(path[:,:2]-pose[:2],axis=1)
    candidates=np.flatnonzero(distance<=distance.min()+.6)
    index=candidates[np.argmin(distance[candidates]+.8*np.abs(wrap(path[candidates,2]-pose[2])))]
    points=[pose];last=pose
    for p in path[index:]:
        if np.linalg.norm(p[:2]-last[:2])>=spacing or abs(wrap(p[2]-last[2]))>=.15:
            points.append(p);last=p
    if np.linalg.norm(path[-1][:2]-last[:2])+abs(wrap(path[-1,2]-last[2]))>1e-6:
        points.append(path[-1])
    sampled=[points[0]]
    for a,b in zip(points,points[1:]):
        angle=float(wrap(b[2]-a[2]))
        n=max(1,math.ceil(np.linalg.norm(b[:2]-a[:2])/spacing),math.ceil(abs(angle)/.15))
        for j in range(1,n+1):
            q=a+(b-a)*j/n;q=q.copy();q[2]=float(wrap(a[2]+angle*j/n));sampled.append(q)
    return np.asarray(sampled)


def observation_goal(reference,pose,goal,distance=3.):
    """Propose a nearby point on the same corridor; this does NOT certify safety.

    The normal rule planner must find a fresh full-hull-safe forward path to
    this point, and both traffic assessments must permit it before execution.
    """
    if reference is None or len(reference)<2 or not 1.<=distance<=3.:return None
    reference=np.asarray(reference);pose=np.asarray(pose);goal=np.asarray(goal)
    if (np.linalg.norm(reference[-1,:2]-goal[:2])>.5 or abs(wrap(reference[-1,2]-goal[2]))>.25
            or np.linalg.norm(goal[:2]-pose[:2])<6.):return None
    near=(np.linalg.norm(reference[:,:2]-pose[:2],axis=1)<=2.)&(np.abs(wrap(reference[:,2]-pose[2]))<=1.2)
    if not near.any():return None
    points=remaining_path(reference,pose)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1))]
    indices=np.flatnonzero(arc>=distance)
    if not len(indices):return None
    target=points[indices[0]];delta=target[:2]-pose[:2]
    if (abs(wrap(math.atan2(delta[1],delta[0])-pose[2]))>.8
            or abs(wrap(target[2]-pose[2]))>.7):return None
    return target.tolist()


def bounded_observation_path(path,reference,target):
    """Reject a detour disguised as a short observation step; not a hull check."""
    path=np.asarray(path);reference=np.asarray(reference);target=np.asarray(target)
    if len(path)<2 or not np.isfinite(path).all():return False
    length=np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum()
    corridor=remaining_path(reference,path[0])
    distance=cKDTree(corridor[:,:2]).query(path[:,:2])[0]
    return bool(length<=5. and distance.max()<=.75 and np.linalg.norm(path[-1,:2]-target[:2])<.1
                and abs(wrap(path[-1,2]-target[2]))<.1)


def arrival_profile(path, pose, config=RouteConfig()):
    """Stopped departure, curvature cap, forward acceleration, terminal braking.

    Positions are hypotheses along a path, not restamped sensor measurements.
    Arrival intervals allow 20% timing variation and one second of latency.
    """
    points=remaining_path(path,pose)
    if len(points)==1:
        return dict(poses=points,eta=np.zeros(1),earliest=np.zeros(1),latest=np.ones(1),length=0.)
    ds=np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1)
    angle=np.abs(wrap(np.diff(points[:,2])))
    scale=terminal_scale(np.linalg.norm(points[:,:2]-points[-1,:2],axis=1),config)
    edge_scale=np.minimum(scale[:-1],scale[1:])
    edge_limit=edge_scale*np.minimum(config.speed,config.yaw_rate*ds/np.maximum(angle,1e-8))
    speed=config.speed*scale
    speed[:-1]=np.minimum(speed[:-1],edge_limit);speed[1:]=np.minimum(speed[1:],edge_limit)
    speed[0]=speed[-1]=0.
    for i in range(1,len(speed)):
        speed[i]=min(speed[i],math.sqrt(speed[i-1]**2+2*config.acceleration*ds[i-1]))
    for i in range(len(speed)-2,-1,-1):
        speed[i]=min(speed[i],math.sqrt(speed[i+1]**2+2*config.deceleration*ds[i]))
    dt=np.maximum(2*ds/np.maximum(speed[:-1]+speed[1:],1e-6),angle/(config.yaw_rate*edge_scale))
    # A very short two-pose route still accelerates and brakes between endpoints.
    rest=(speed[:-1]+speed[1:]<1e-6)&(ds>1e-6)
    # Even the two-endpoint case must respect the configured speed cap.
    acceleration_time=np.sqrt(2*ds[rest]*(1/config.acceleration+1/config.deceleration))
    cap=edge_limit[rest]
    capped_time=ds[rest]/np.maximum(cap,1e-6)+.5*cap*(1/config.acceleration+1/config.deceleration)
    peak=np.sqrt(2*ds[rest]/(1/config.acceleration+1/config.deceleration))
    dt[rest]=np.maximum(np.where(peak<=cap,acceleration_time,capped_time),angle[rest]/(config.yaw_rate*edge_scale[rest]))
    eta=np.r_[0.,np.cumsum(dt)]
    return dict(poses=points,eta=eta,earliest=np.maximum(0.,eta*(1-config.timing_fraction)-1.),
                latest=eta*(1+config.timing_fraction)+1.,length=float(ds.sum()))


def hull_bounds(poses, footprint):
    c=np.cos(poses[:,2]);s=np.sin(poses[:,2]);fp=np.asarray(footprint)
    x=c[:,None]*fp[:,0]-s[:,None]*fp[:,1]+poses[:,0,None]
    y=s[:,None]*fp[:,0]+c[:,None]*fp[:,1]+poses[:,1,None]
    return np.stack([np.column_stack([x.min(axis=1),y.min(axis=1)]),
                     np.column_stack([x.max(axis=1),y.max(axis=1)])],axis=1)


def route_conflict(profile, footprint, tracks, stamp, delay, config, possible_stop=False):
    """Conservative AABBs at arrival intervals; not a full dynamic planner.

    Forecast coverage is the next `horizon` seconds of travel, independent of
    the proposed wait: waiting cannot hide an intersection beyond that window.
    A lost object's last observed position remains occupied as in the guard.
    """
    selected=profile['eta']<=config.horizon
    eta=profile['eta'][selected]
    boxes=hull_bounds(profile['poses'][selected],footprint)
    low=profile['earliest'][selected]+delay;high=profile['latest'][selected]+delay
    # The vessel occupies its departure pose for the entire proposed wait,
    # not only at departure. A target crossing it earlier forbids that wait.
    if delay>0:low[0]=0.
    earliest=None
    for track in tracks:
        if not track.moving:continue
        age=max(0.,stamp-track.stamp)
        a=(low+age)[:,None]*track.velocity;b=(high+age)[:,None]*track.velocity
        if possible_stop:
            # Beyond the original 20 s forecast, a target can stop instead of
            # continuing at constant velocity. This is uncertainty, not truth.
            a=np.minimum(low+age,20.)[:,None]*track.velocity
        lo=track.bounds[0]+np.minimum(a,b);hi=track.bounds[1]+np.maximum(a,b)
        if age>2.:
            lo=np.minimum(lo,track.bounds[0]);hi=np.maximum(hi,track.bounds[1])
        padding=config.margin+.15+.04*high+.2*age
        # Cover interpolation between the .4 m / .15 rad route samples too.
        padding+=.2+np.linalg.norm(np.asarray(footprint),axis=1).max()*.075
        overlap=np.all(boxes[:,0]<=hi+padding[:,None],axis=1)&np.all(boxes[:,1]>=lo-padding[:,None],axis=1)
        if overlap.any():
            value=float(eta[np.flatnonzero(overlap)[0]])
            earliest=value if earliest is None else min(earliest,value)
    return earliest


def route_metrics(path,pose,footprint,tracks,stamp,config):
    profile=arrival_profile(path,pose,config)
    conflict=route_conflict(profile,footprint,tracks,stamp,0.,config)
    delay=None
    for candidate in np.arange(0.,config.max_wait+.001,config.wait_step):
        if route_conflict(profile,footprint,tracks,stamp,float(candidate),config) is None:
            delay=float(candidate);break
    stopped=route_conflict(profile,footprint,tracks,stamp,delay or 0.,config,possible_stop=True)
    return dict(eta_s=float(profile['eta'][-1]),length_m=profile['length'],wait_s=delay,
                conflict_eta_s=conflict,stop_conflict_eta_s=stopped,profile=profile)


def assess_routes(candidate, reference, pose, footprint, tracks, stamp, config=RouteConfig()):
    current=route_metrics(candidate,pose,footprint,tracks,stamp,config)
    old=None;switch_cost=0.;deviation=0.
    compatible=False
    if reference is not None:
        reference=np.asarray(reference)
        distances=np.linalg.norm(reference[:,:2]-np.asarray(pose)[:2],axis=1)
        heading=np.abs(wrap(reference[:,2]-pose[2]))
        compatible=(np.any((distances<=2.)&(heading<=1.2)) and
                    np.linalg.norm(reference[-1,:2]-candidate[-1,:2])<.5 and abs(wrap(reference[-1,2]-candidate[-1,2]))<.25)
    if compatible:
        old=route_metrics(reference,pose,footprint,tracks,stamp,config)
        distance=cKDTree(old['profile']['poses'][:,:2]).query(current['profile']['poses'][:,:2])[0]
        deviation=float(distance.mean())
        if deviation>.75:
            switch_cost=config.switch_base+config.switch_distance_weight*deviation/config.speed
    def score(route):
        if route['wait_s'] is None:return math.inf
        return route['eta_s']+route['wait_s']+(config.stop_risk_cost if route['stop_conflict_eta_s'] is not None else 0.)
    new_score=score(current)+switch_cost
    old_score=score(old)+config.replan_time if old else math.inf
    prefer=old is not None and math.isfinite(old_score) and old_score+config.improvement<new_score
    # An observed stop is not evidence of a permanent closure. Price a bounded
    # period of gathering more observations before paying for a costly switch.
    # This is an affordable observation budget, NOT a predicted departure time:
    # the old route remains blocked and must be freshly planned before use.
    observation_budget=0.
    if old is not None and old['wait_s'] is None and current['wait_s']==0.:
        affordable=min(config.max_wait,max(0.,new_score-old['eta_s']-config.replan_time-config.improvement))
        stationary=dict(poses=np.asarray([pose]),eta=np.zeros(1),earliest=np.zeros(1),
                        latest=np.array([affordable]))
        if affordable>=config.wait_step and route_conflict(stationary,footprint,tracks,stamp,0.,config,possible_stop=True) is None:
            observation_budget=affordable
    reason=('wait for previous corridor to reduce arrival time and switching cost' if prefer else
            'observe blocked reference for a bounded budget; departure time is unknown' if observation_budget else
            'candidate has no predicted clear departure within wait horizon' if current['wait_s'] is None else
            'wait for candidate arrival conflict to clear' if current['wait_s']>0 else
            'candidate arrival estimate is preferable')
    result=dict(eta_s=current['eta_s'],wait_s=current['wait_s'],reference_eta_s=old['eta_s'] if old else None,
                reference_wait_s=old['wait_s'] if old else None,switch_cost_s=switch_cost,
                candidate_score_s=new_score if math.isfinite(new_score) else None,
                reference_score_s=old_score if math.isfinite(old_score) else None,
                conflict_eta_s=current['conflict_eta_s'],stop_conflict_eta_s=current['stop_conflict_eta_s'],
                reference_stop_conflict_eta_s=old['stop_conflict_eta_s'] if old else None,
                observation_budget_s=observation_budget,
                mean_route_deviation_m=deviation,prefer_wait=prefer,reason=reason,
                assessed_travel_horizon_s=config.horizon)
    return result,current['profile']
