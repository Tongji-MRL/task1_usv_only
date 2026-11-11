import unittest
from usv_nav2.stationkeeping import StationKeeping


class StationTests(unittest.TestCase):
    def enter(self):
        p=StationKeeping()
        self.assertEqual(p.decide('GUIDED',True,True,'hold',(0,0),10),'LOITER')
        self.assertFalse(p.owned)
        p.observe('LOITER',True);self.assertTrue(p.owned)
        return p

    def test_confirmed_handoff(self):
        p=self.enter()
        self.assertIsNone(p.decide('LOITER',True,True,'hold',(.3,0),11))
        self.assertEqual(p.decide('LOITER',True,True,'resume',(.3,0),12),'GUIDED')
        p.observe('GUIDED',True);self.assertFalse(p.owned)

    def test_stale_permission_or_sensor_exits_and_latches(self):
        p=self.enter()
        self.assertEqual(p.decide('LOITER',True,False,'hold',(0,0),11),'HOLD')
        p.observe('HOLD',True)
        self.assertIsNone(p.decide('HOLD',True,True,'hold',(0,0),12))
        self.assertTrue(p.failed)

    def test_excursion_and_cancel(self):
        for pos,action in [((.81,0),'hold'),((0,0),'stop'),((0,0),'')]:
            p=self.enter();self.assertEqual(p.decide('LOITER',True,True,action,pos,11),'HOLD')

    def test_manual_takeover_is_not_overridden(self):
        p=self.enter();p.observe('MANUAL',True)
        self.assertIsNone(p.decide('MANUAL',True,True,'hold',(0,0),11))
        self.assertIsNone(p.decide('GUIDED',True,True,'hold',(0,0),12))

    def test_pending_mode_timeout_and_revocation(self):
        for healthy,now in [(True,14),(False,10.2)]:
            p=StationKeeping();p.decide('GUIDED',True,True,'hold',(0,0),10)
            self.assertEqual(p.decide('GUIDED',True,healthy,'hold',(0,0),now),'HOLD')

    def test_unowned_loiter_and_preview_are_not_claimed(self):
        p=StationKeeping();p.observe('LOITER',True)
        self.assertIsNone(p.decide('LOITER',True,False,'stop',(0,0),10))
        self.assertIsNone(p.decide('GUIDED',False,True,'hold',(0,0),11))


if __name__=='__main__':unittest.main()
