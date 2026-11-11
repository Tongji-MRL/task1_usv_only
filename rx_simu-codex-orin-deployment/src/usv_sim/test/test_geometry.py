import math
import unittest
from usv_sim.geometry import polygon_distance, vessel_clearance


class GeometryTests(unittest.TestCase):
    def test_separation_intersection_and_touch(self):
        a=[[-1,-1],[1,-1],[1,1],[-1,1]]
        self.assertAlmostEqual(polygon_distance(a,[[3,3],[5,3],[5,5],[3,5]]),math.sqrt(8))
        self.assertEqual(polygon_distance(a,[[0,0],[2,0],[2,2],[0,2]]),0)
        self.assertEqual(polygon_distance(a,[[1,-1],[3,-1],[3,1],[1,1]]),0)

    def test_rotated_vessel_in_world(self):
        hull=[[-2,-.5],[2,-.5],[2,.5],[-2,.5]]
        box=[{'x':1,'y':5,'size':[2,2]}]
        self.assertAlmostEqual(vessel_clearance(hull,1,1,0,box),2.5)
        self.assertAlmostEqual(vessel_clearance(hull,1,1,math.pi/2,box),1)
