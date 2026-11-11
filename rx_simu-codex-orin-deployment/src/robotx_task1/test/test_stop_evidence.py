import unittest
from robotx_task1.safety_evidence import stop_evidence


class StopEvidenceTest(unittest.TestCase):
    def fixture(self):
        events=[{'event':'stop_requested','sim':1.,'reason':'semantic revision changed'},
                {'event':'execution_started','sim':4.}]
        rows=[dict(sim=i/10,truth=[i/100,0,0,0,.2 if i<20 else 0.,0,0],truth_age=.02,
                   preview=[.2 if i<12 else 0.,0],frame=i*25,thrust_source='thruster_feedback',
                   left_n=10. if i<25 else 0.,right_n=10. if i<25 else 0.) for i in range(10,40)]
        return events,rows

    def test_separate_command_force_and_motion(self):
        events,rows=self.fixture();r=stop_evidence(events,rows)[0]
        self.assertTrue(r['passed']);self.assertAlmostEqual(r['zero_command_delay_s'],.2)
        self.assertAlmostEqual(r['first_force_below_1N_delay_s'],1.5)
        self.assertAlmostEqual(r['stopped_dwell_confirmed_delay_s'],2.)

    def test_frozen_simulator_is_not_a_stop(self):
        events,rows=self.fixture()
        for r in rows:r['frame']=1
        self.assertFalse(stop_evidence(events,rows)[0]['passed'])

    def test_resume_before_actual_stop_is_rejected(self):
        events,rows=self.fixture();events[-1]['sim']=2.5
        self.assertFalse(stop_evidence(events,rows)[0]['passed'])


if __name__=='__main__':unittest.main()
