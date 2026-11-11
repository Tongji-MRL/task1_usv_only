"""Reject isolated marking returns without inventing clearing observations."""
from collections import deque
import numpy as np
from scipy.spatial import cKDTree


class ReturnSupport:
    """Three nearby points now, or a hit in each of two preceding frames.

    Input coordinates must share a fixed FCU frame. Sparse persistent objects
    can be accepted after two 10 Hz intervals; dense moving surfaces need no
    temporal confirmation. Missing returns never become free-space evidence.
    """
    def __init__(self,radius=.3,age=.3):
        self.radius=radius;self.age=age;self.history=deque(maxlen=2);self.stamp=-np.inf

    def select(self,points,stamp):
        points=np.asarray(points,dtype=float).reshape(-1,3)
        if stamp<=self.stamp:self.history.clear()
        self.stamp=stamp
        self.history=deque(((t,p) for t,p in self.history if 0<stamp-t<=self.age),maxlen=2)
        selected=np.zeros(len(points),bool)
        if len(points):
            selected=cKDTree(points).query_ball_point(points,self.radius,return_length=True)>=3
            if len(self.history)==2:
                repeated=np.ones(len(points),bool)
                for _,previous in self.history:
                    repeated &= (cKDTree(previous).query_ball_point(points,self.radius,return_length=True)>0
                                 if len(previous) else np.zeros(len(points),bool))
                selected |= repeated
        self.history.append((stamp,points.copy()))
        return selected
