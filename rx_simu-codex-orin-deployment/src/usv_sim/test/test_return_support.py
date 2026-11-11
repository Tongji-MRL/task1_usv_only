import unittest
import numpy as np
from usv_sim.return_support import ReturnSupport


class SupportTests(unittest.TestCase):
    def test_single_noise_hit_is_rejected_but_moving_surface_is_immediate(self):
        s=ReturnSupport();p=np.array([[1,0,.5],[1.1,0,.5],[1.2,0,.5],[5,0,.5]])
        np.testing.assert_equal(s.select(p,1.),[True,True,True,False])
        np.testing.assert_equal(s.select(p[:3]+[.5,0,0],1.1),[True]*3)

    def test_sparse_persistent_obstacle_keeps_its_newest_measurement(self):
        s=ReturnSupport();p=np.array([[10,1,.5]])
        self.assertFalse(s.select(p,1.)[0]);self.assertFalse(s.select(p,1.1)[0])
        self.assertTrue(s.select(p,1.2)[0])
        self.assertEqual(len(s.select(np.empty((0,3)),1.3)),0)
        self.assertFalse(s.select(p,1.4)[0])

    def test_time_gap_and_reset_cannot_reuse_historical_support(self):
        s=ReturnSupport();p=[[10,1,.5]]
        for t in (1.,1.1,1.2):s.select(p,t)
        self.assertFalse(s.select(p,2.)[0]);self.assertFalse(s.select(p,.5)[0])


if __name__=='__main__':unittest.main()
