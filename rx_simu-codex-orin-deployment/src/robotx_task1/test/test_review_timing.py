"""A replay must not depict future data or cancelled plans as current evidence."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts'))
from annotate_task1 import Review, camera_project, prior_index


class ReviewTiming(unittest.TestCase):
    def test_gap_retains_only_prior_source_sample(self):
        self.assertEqual(prior_index([1.,2.,8.],.9),-1)
        self.assertEqual(prior_index([1.,2.,8.],6.),1)
        self.assertEqual(prior_index([1.,2.,8.],8.),2)

    def test_down_camera_projects_north_up_east_right(self):
        config={'pose':[35.,0.,58.,0.,math.pi/2,math.pi/2],
                'width':1600,'height':1000,'horizontal_fov':1.3}
        center,east,north=camera_project([[35,0,0],[36,0,0],[35,1,0]],config)
        self.assertAlmostEqual(center[0],800);self.assertAlmostEqual(center[1],500)
        self.assertGreater(east[0],center[0]);self.assertAlmostEqual(east[1],center[1])
        self.assertLess(north[1],center[1]);self.assertAlmostEqual(north[0],center[0])

    def test_light_revision_and_cancellation_revoke_displayed_plan(self):
        review=Review.__new__(Review)
        ready={'sim':10.,'event':'path_ready','revision':1}
        review.data=SimpleNamespace(plan_topic='/task1/reference_path',mission=[ready])
        review.mission_times=[10.]
        review.events={'/task1/reference_path':[{'sim':10.}]}
        review.event_times={'/task1/reference_path':[10.]}
        self.assertTrue(review.plan(11.,{'motion':'EXECUTING','revision':1})[1])
        self.assertFalse(review.plan(11.,{'motion':'EXECUTING','revision':2})[1])
        self.assertFalse(review.plan(11.,{'motion':'STOPPING','revision':1})[1])
        review.data.mission.append({'sim':12.,'event':'stop_requested','revision':1})
        review.mission_times.append(12.)
        self.assertFalse(review.plan(13.,{'motion':'EXECUTING','revision':1})[1])


if __name__=='__main__':unittest.main()
