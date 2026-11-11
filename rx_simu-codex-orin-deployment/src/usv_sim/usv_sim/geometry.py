"""Planar convex-body clearance for independent simulation evaluation."""
import numpy as np


def polygon_distance(first, second):
    a=np.asarray(first,dtype=float);b=np.asarray(second,dtype=float)
    separate=False
    for polygon in (a,b):
        edges=np.roll(polygon,-1,axis=0)-polygon
        axes=np.stack([-edges[:,1],edges[:,0]],axis=1)
        pa=a@axes.T;pb=b@axes.T
        if np.any((pa.max(axis=0)<pb.min(axis=0)) | (pb.max(axis=0)<pa.min(axis=0))):separate=True
    if not separate:return 0.
    distances=[]
    for points,polygon in ((a,b),(b,a)):
        edges=np.roll(polygon,-1,axis=0)-polygon
        delta=points[:,None,:]-polygon
        fractions=np.clip(np.sum(delta*edges,axis=2)/np.sum(edges**2,axis=1),0,1)
        distances.append(np.linalg.norm(delta-fractions[...,None]*edges,axis=2).min())
    return float(min(distances))


def vessel_clearance(footprint,x,y,heading,obstacles):
    c,s=np.cos(heading),np.sin(heading)
    boat=np.asarray(footprint)@np.array([[c,s],[-s,c]])+np.array([x,y])
    values=[]
    for obj in obstacles:
        sx,sy=np.array(obj['size'])/2
        rectangle=np.array([[-sx,-sy],[sx,-sy],[sx,sy],[-sx,sy]])+[obj['x'],obj['y']]
        values.append(polygon_distance(boat,rectangle))
    return min(values,default=float('inf'))
