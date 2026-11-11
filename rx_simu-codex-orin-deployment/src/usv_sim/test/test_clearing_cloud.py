import unittest
import numpy as np
from usv_sim.clearing_cloud import clearing_points,ray_directions


class ClearingRays(unittest.TestCase):
    def test_no_hit_preserves_vertical_ray_and_unknown_is_not_free(self):
        raw=np.full((2,3,3),np.inf);raw[0,1]=np.nan;raw[1,1]=[2,1,.5]
        directions=ray_directions((2,3),-.25,.25)
        out=clearing_points(raw,np.zeros((2,3),dtype=bool),directions)
        self.assertTrue(np.isnan(out[0,1]).all())
        np.testing.assert_array_equal(out[1,1],raw[1,1])
        self.assertLess(out[0,0,2],0);self.assertGreater(out[1,0,2],0)
        self.assertAlmostEqual(np.linalg.norm(out[0,0]),20.)

    def test_self_occlusion_is_not_a_max_range_ray(self):
        raw=np.ones((2,3,3));mask=np.zeros((2,3),dtype=bool);mask[0,0]=True
        out=clearing_points(raw,mask,ray_directions((2,3),-.25,.25))
        self.assertTrue(np.isnan(out[0,0]).all());np.testing.assert_array_equal(out[1,1],raw[1,1])

    def test_unknown_layout_rejected(self):
        with self.assertRaises(ValueError):clearing_points(np.zeros((2,3,3)),np.zeros((2,3),bool),np.zeros((3,3,3)))

if __name__=='__main__':unittest.main()
