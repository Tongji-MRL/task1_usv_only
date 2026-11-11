"""Record independent simulation evidence; evaluate in source-time order offline."""
import bisect
import json
import math
from pathlib import Path

from .core import CircleTracker, passage_events, wrap
from .safety_evidence import stop_evidence


def evaluate(run):
    from usv_sim.geometry import vessel_clearance
    run=Path(run);scene=json.loads((run/'generated/scene.json').read_text())
    rows=[json.loads(line) for line in (run/'task1_judge.jsonl').read_text().splitlines()]
    truths=sorted((r for r in rows if r['type']=='truth'),key=lambda r:r['stamp'])
    scenes=sorted((r for r in rows if r['type']=='scene'),key=lambda r:r['stamp'])
    statuses=sorted((r for r in rows if r['type']=='status'),key=lambda r:r['stamp'])
    st=[r['stamp'] for r in scenes];mt=[r['stamp'] for r in statuses]
    # The scene records physics-tick effective times, not just the next 10 Hz publication.
    history=scenes[-1]['data'].get('semantic_history',[]) if scenes else []
    ht=[h['stamp'] for h in history]
    circles={};passes=[];last_pass={};previous=None;previous_phase=None;minimum=math.inf
    completed=None;last=None;previous_stamp=None;contact_count=sum(r['count'] for r in rows if r['type']=='contacts')
    changes=sorted({r['data']['revision'] for r in scenes})
    for row in truths:
        si=bisect.bisect_right(st,row['stamp'])-1;mi=bisect.bisect_right(mt,row['stamp'])-1
        if si<0 or mi<0:continue
        data=scenes[si]['data'];status=statuses[mi]['data'];phase=status['stage'];p=row['pose']
        obstacles=[dict(name=b['id'],x=b['x'],y=b['y'],size=[.508,.508]) for b in data['buoys']]
        obstacles+=scene['task1'].get('infrastructure',[])
        if 'dynamic_pose' in data:obstacles.append(dict(name='moving',x=data['dynamic_pose'][0],y=data['dynamic_pose'][1],size=[2.,3.]))
        minimum=min(minimum,vessel_clearance(scene['footprint'],p[0],p[1],p[2],obstacles))
        if phase in ('CIRCLE_ENTRY','CIRCLE_EXIT'):
            if phase not in circles:
                state=3 if phase=='CIRCLE_ENTRY' else 4;b=next(b for b in data['buoys'] if b['state']==state)
                circles[phase]=CircleTracker([b['x'],b['y']],-1 if state==3 else 1,scene['task1']['circle_radius'])
            circles[phase].update(p)
        if phase=='TRANSIT' and previous and previous_phase=='TRANSIT':
            cuts=[previous_stamp]+[t for t in ht if previous_stamp<t<row['stamp']]+[row['stamp']]
            def at(t):
                f=(t-previous_stamp)/(row['stamp']-previous_stamp)
                return [previous[0]+f*(p[0]-previous[0]),previous[1]+f*(p[1]-previous[1]),wrap(previous[2]+f*wrap(p[2]-previous[2]))]
            for a,b in zip(cuts,cuts[1:]):
                semantic=history[max(0,bisect.bisect_right(ht,(a+b)/2)-1)] if history else data
                for event in passage_events(at(a),at(b),semantic['buoys']):
                    stamp=a+event['fraction']*(b-a)
                    if stamp-last_pass.get(event['id'],-1e9)<2:continue
                    last_pass[event['id']]=stamp;event.update(stamp=stamp,revision=semantic['revision'])
                    passes.append(event)
        if phase=='COMPLETE' and completed is None:completed=dict(stamp=row['stamp'],pose=p,target=status['target'])
        previous=p;previous_phase=phase;previous_stamp=row['stamp'];last=row
    errors=None;drift=None
    if completed:
        target=completed['target'];p=completed['pose']
        errors=dict(position_m=math.dist(p[:2],target[:2]),yaw_rad=abs(wrap(p[2]-target[2])))
        if last:drift=math.dist(p[:2],last['pose'][:2])
    circle_results={k:dict(complete=v.complete,angle_rad=v.angle,wrong_way_rad=v.wrong_way) for k,v in circles.items()}
    observed={e['state'] for e in passes if e['legal']}
    correct_circles=all(k in circles and circles[k].complete for k in ('CIRCLE_ENTRY','CIRCLE_EXIT'))
    event_file=run/'task1_mission.jsonl';physical_file=run/'task1.jsonl'
    stops=stop_evidence([json.loads(l) for l in event_file.read_text().splitlines()],
                        [json.loads(l) for l in physical_file.read_text().splitlines()]) if event_file.exists() and physical_file.exists() else []
    ok=bool(completed and errors['position_m']<=.5 and errors['yaw_rad']<=.25 and correct_circles
            and observed=={1,2} and all(e['legal'] for e in passes) and contact_count==0 and minimum>0
            and all(e['passed'] for e in stops))
    report=dict(passed=ok,scope='Disruptive boat behavior; UAV observations and RoboCommand reporting are substitutes',
                mission_status=statuses[-1]['data'] if statuses else None,circles=circle_results,
                passage_events=passes,collision_contacts=contact_count,min_clearance_m=minimum if math.isfinite(minimum) else None,
                revisions=changes,completion=completed,endpoint_error=errors,post_completion_drift_m=drift,
                stops=stops,semantic_timing='physics effective history' if history else '10 Hz scene publication',
                post_completion_record_s=last['stamp']-completed['stamp'] if last and completed else None,
                evidence_samples=len(truths),rule_tolerances={'nearest_tie_m':.01,'circle_closure_m':.5,'circle_radial_m':1.})
    (run/'task1_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from ros_gz_interfaces.msg import Contacts
    class Recorder(Node):
        def __init__(self):
            super().__init__('task1_independent_judge');self.declare_parameter('run_dir','')
            self.log=(Path(self.get_parameter('run_dir').value)/'task1_judge.jsonl').open('w')
            self.last_truth=-1.
            self.create_subscription(Odometry,'/usv/ground_truth',self.truth,qos_profile_sensor_data)
            self.create_subscription(String,'/task1/scene_state',lambda m:self.data('scene',m),10)
            self.create_subscription(String,'/task1/status',lambda m:self.data('status',m),10)
            self.create_subscription(Contacts,'/usv/contacts',self.contacts,qos_profile_sensor_data)
        def record(self,row):self.log.write(json.dumps(row,allow_nan=False)+'\n');self.log.flush()
        def truth(self,msg):
            t=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if t-self.last_truth<.09:return
            self.last_truth=t;p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
            self.record(dict(type='truth',stamp=t,pose=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
                             velocity=[v.linear.x,v.linear.y,v.angular.z]))
        def data(self,kind,msg):
            d=json.loads(msg.data);self.record(dict(type=kind,stamp=d['stamp'] if kind=='scene' else d['sim'],data=d))
        def contacts(self,msg):
            if msg.contacts:self.record(dict(type='contacts',stamp=self.get_clock().now().nanoseconds/1e9,
                                             count=len(msg.contacts),data=str(msg)))
    rclpy.init();node=Recorder()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.log.close();node.destroy_node();rclpy.try_shutdown()
