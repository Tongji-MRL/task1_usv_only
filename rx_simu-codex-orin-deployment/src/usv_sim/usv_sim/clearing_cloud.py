"""True 3D ray endpoints for the generated organized lidar's no-return beams."""
import numpy as np


def ray_directions(shape, vertical_min, vertical_max):
    azimuth=np.linspace(-np.pi,np.pi,shape[1])[None,:]
    elevation=np.linspace(vertical_min,vertical_max,shape[0])[:,None]
    return np.stack(np.broadcast_arrays(np.cos(elevation)*np.cos(azimuth),
                    np.cos(elevation)*np.sin(azimuth),np.sin(elevation)),axis=-1)


def clearing_points(raw, self_mask, directions, ray_range=20.):
    if raw.shape!=directions.shape:raise ValueError('Organized lidar dimensions changed')
    points=np.array(raw,copy=True)
    # NaN means unknown. Removed self hits are occluded, not free space.
    no_hit=np.isinf(raw).any(axis=-1)&~np.isnan(raw).any(axis=-1)
    points[no_hit]=directions[no_hit]*ray_range
    points[self_mask]=np.nan
    return points
