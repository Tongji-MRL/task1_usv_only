"""Mode ownership for optional native station keeping; no actuator commands."""
import math


class StationKeeping:
    def __init__(self, excursion=.8, mode_timeout=3.):
        self.excursion=excursion;self.mode_timeout=mode_timeout
        self.owned=False;self.pending=None;self.requested_at=0.;self.anchor=None
        self.yielded=False;self.failed=False;self.reason='inactive'

    def observe(self,mode,armed):
        if not armed:
            self.owned=False;self.pending=None;self.anchor=None
            return
        if self.pending and mode==self.pending:
            self.owned=mode=='LOITER';self.pending=None
            if not self.owned:self.anchor=None
            self.reason='mode confirmed: '+mode
        elif (self.owned and mode!='LOITER') or (self.pending and mode not in ('GUIDED','LOITER')):
            self.owned=False;self.pending=None;self.anchor=None;self.yielded=True
            self.reason='external mode change; relinquished ownership'

    def request(self,mode,now,reason):
        self.pending=mode;self.requested_at=now;self.reason=reason
        return mode

    def decide(self,mode,armed,healthy,action,position,now):
        if self.yielded or not armed:return None
        if self.failed:
            if mode in ('GUIDED','LOITER') and (self.pending!='HOLD' or now-self.requested_at>.5):
                return self.request('HOLD',now,'station keeping fault')
            return None
        displaced=(self.owned and self.anchor is not None and position is not None
                   and math.dist(self.anchor,position)>self.excursion)
        if (self.owned or self.pending=='LOITER') and (not healthy or action not in ('hold','resume') or displaced):
            self.failed=True
            return self.request('HOLD',now,'stale/withdrawn hold permission or excursion')
        if self.pending:
            if now-self.requested_at>self.mode_timeout:
                self.failed=True
                return self.request('HOLD',now,'mode confirmation timeout')
            return None
        if self.owned and action=='resume':
            return self.request('GUIDED',now,'fresh replan requested')
        if not self.owned and mode=='GUIDED' and healthy and action=='hold' and position is not None:
            self.anchor=tuple(position)
            return self.request('LOITER',now,'cancelled navigation; observed hold area clear')
        return None
