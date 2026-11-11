import unittest
from robotx_task1.beacon_evidence import blink_evidence


class BeaconTiming(unittest.TestCase):
    def samples(self,period=2.,skip=()):
        return [dict(stamp=i/5,dominant_rgb_pixels=[100 if (i/5)%period<period/2 else 0,0,0])
                for i in range(40) if i not in skip]

    def test_missing_frames_do_not_shift_source_phase(self):
        r=blink_evidence(self.samples(skip=(5,6,7,10,11)),0)
        self.assertTrue(r['passed']);self.assertGreater(r['max_source_gap_s'],.5)

    def test_wrong_period_rejected(self):
        self.assertFalse(blink_evidence(self.samples(period=2.4),0)['passed'])

    def test_one_phase_alone_cannot_prove_blinking(self):
        self.assertFalse(blink_evidence([s for s in self.samples() if s['stamp']%2<1],0)['passed'])

if __name__=='__main__':unittest.main()
