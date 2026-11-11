"""Optional fail-closed Task1 permission; independent of ROS for fault tests."""
from .core import stamp_is_fresh


class TaskLease:
    def __init__(self, timeout=.5):
        self.timeout=timeout;self.revision=-1;self.stamp=0.;self.received=None
        self.allow=False;self.reason='task permission missing';self.station_action=''

    def update(self, revision, allow, stamp, sim_now, wall_now, reason='',station_action=''):
        if (station_action not in ('','hold','resume','stop') or revision<self.revision or stamp<=self.stamp
                or not stamp_is_fresh(stamp,sim_now,self.timeout)):
            return False
        changed=(revision!=self.revision or allow!=self.allow or station_action!=self.station_action
                 or not self.fresh(sim_now,wall_now))
        self.revision=revision;self.allow=allow;self.stamp=stamp;self.received=wall_now;self.reason=reason
        self.station_action=station_action
        return changed

    def valid(self, sim_now, wall_now):
        return self.allow and self.fresh(sim_now,wall_now)

    def fresh(self,sim_now,wall_now):
        return (self.received is not None and 0<=wall_now-self.received<=self.timeout
                and stamp_is_fresh(self.stamp,sim_now,self.timeout))
