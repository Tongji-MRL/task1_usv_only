import math
import unittest
from robotx_task1.core import passage_events, CircleTracker, valid_snapshot


class OracleTests(unittest.TestCase):
    def setUp(self):
        self.b=[dict(id='r',x=0.,y=-3.,state=1,radius=.36),dict(id='g',x=0.,y=3.,state=2,radius=.36)]

    def test_direction_not_world_axis(self):
        forward=passage_events([-1,0,0],[1,0,0],self.b)
        self.assertEqual(len(forward),2);self.assertTrue(all(e['legal'] for e in forward))
        self.assertTrue(all(not e['legal'] for e in passage_events([1,0,math.pi],[-1,0,math.pi],self.b)))

    def test_nearest_off_obstacle(self):
        self.b.append(dict(id='off',x=0.,y=-1.,state=0,radius=.36))
        self.assertEqual([e['id'] for e in passage_events([-1,0,0],[1,0,0],self.b)],['g'])

    def test_no_credit_for_pivot(self):
        self.assertEqual(passage_events([-1,0,0],[-1,0,math.pi],self.b),[])

    def test_half_turn_and_wrong_direction_are_not_complete(self):
        c=CircleTracker([0,0],-1)
        for i in range(101):c.update([6*math.cos(math.pi*i/100),6*math.sin(math.pi*i/100)])
        self.assertFalse(c.complete);self.assertLess(c.angle,0)

    def test_full_circle_requires_winding(self):
        c=CircleTracker([0,0],-1)
        c.update([6,0]);c.update([6,0]);self.assertFalse(c.complete)
        for i in range(1,630):
            c.update([6*math.cos(-i*.01),6*math.sin(-i*.01)])
        self.assertTrue(c.complete)

    def test_duplicate_ids_and_roles_rejected(self):
        data={'buoys':[dict(id=str(i),x=float(i),y=0.,radius=.36,state=0) for i in range(10)]}
        data['buoys'][0]['state']=3;data['buoys'][1]['state']=4
        self.assertTrue(valid_snapshot(data));data['buoys'][2]['id']='0';self.assertFalse(valid_snapshot(data))

    def test_navigation_margin_does_not_relax_judge(self):
        judge=CircleTracker([0,0],1);mission=CircleTracker([0,0],1,minimum_angle=2*math.pi+.05)
        for i in range(630):
            p=[6*math.cos(i*.01),6*math.sin(i*.01)];judge.update(p);mission.update(p)
        self.assertTrue(judge.complete);self.assertFalse(mission.complete)
        for i in range(630,636):mission.update([6*math.cos(i*.01),6*math.sin(i*.01)])
        self.assertTrue(mission.complete)

    def test_nearest_tie_not_credited(self):
        self.b.append(dict(id='tie',x=0.,y=-3.,state=0,radius=.36))
        r=next(e for e in passage_events([-1,0,0],[1,0,0],self.b) if e['id']=='r')
        self.assertTrue(r['ambiguous']);self.assertFalse(r['legal'])


if __name__=='__main__':unittest.main()
