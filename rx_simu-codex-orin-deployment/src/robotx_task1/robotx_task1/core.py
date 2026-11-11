"""Independent sampled-trajectory rule oracle (no planner or ROS dependencies)."""
import math


def wrap(a): return math.atan2(math.sin(a),math.cos(a))


def passage_events(before, after, buoys, tie=.01):
    """Interpolate abeam crossings; nearest comparison includes OFF and blue bodies.

    before/after are (x,y,yaw). A stationary pivot does not traverse a buoy.
    1 cm nearest-distance ties (`tie`) are flagged as ambiguous, never credited.
    """
    if math.dist(before[:2],after[:2])<1e-6:return []
    events=[]
    def at(t):return [before[0]+t*(after[0]-before[0]),before[1]+t*(after[1]-before[1]),
                      before[2]+t*wrap(after[2]-before[2])]
    def local(p,b):
        dx,dy=b['x']-p[0],b['y']-p[1];c,s=math.cos(p[2]),math.sin(p[2])
        return dx*c+dy*s,-dx*s+dy*c
    for b in buoys:
        if b['state'] not in (1,2):continue
        x0=local(before,b)[0];x1=local(after,b)[0]
        if x0*x1>0 or abs(x1-x0)<1e-8:continue
        lo,hi=0.,1.
        for _ in range(25):
            mid=(lo+hi)/2
            if local(at(mid),b)[0]*x0>0:lo=mid
            else:hi=mid
        p=at((lo+hi)/2);side=local(p,b)[1];distance=math.dist(p[:2],[b['x'],b['y']])
        same=[math.dist(p[:2],[c['x'],c['y']]) for c in buoys if c['id']!=b['id'] and local(p,c)[1]*side>0]
        if same and min(same)<distance-tie:continue
        ambiguous=bool(same and abs(min(same)-distance)<=tie)
        events.append(dict(id=b['id'],state=b['state'],side='port' if side>0 else 'starboard',
                           legal=(side<0 if b['state']==1 else side>0) and not ambiguous,
                           ambiguous=ambiguous,position=p,fraction=(lo+hi)/2))
    return events


class CircleTracker:
    def __init__(self,center,direction,radius=6.,minimum_angle=2*math.pi):
        self.center=center;self.direction=direction;self.radius=radius
        self.minimum_angle=minimum_angle
        self.last=None;self.first=None;self.angle=0.;self.complete=False;self.wrong_way=0.

    def update(self,position):
        x,y=position[0]-self.center[0],position[1]-self.center[1]
        a=math.atan2(y,x)
        if self.last is None:self.first=position[:2]
        else:
            delta=wrap(a-self.last)*self.direction
            self.angle+=delta
            if delta<0:self.wrong_way-=delta
        self.last=a
        if self.angle>=self.minimum_angle and math.dist(position[:2],self.first)<=.5 and abs(math.hypot(x,y)-self.radius)<=1.:
            self.complete=True
        return self.complete


def valid_snapshot(data):
    buoys=data.get('buoys',[])
    return (len(buoys)==10 and len({b['id'] for b in buoys})==10 and
            sum(b['state']==3 for b in buoys)==1 and sum(b['state']==4 for b in buoys)==1 and
            all(b['state'] in range(5) and all(math.isfinite(b[k]) for k in ('x','y','radius'))
                and b['radius']>0 for b in buoys))
