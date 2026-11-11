"""Moving-target safety must survive occlusion, stale input and path changes."""
import math
import unittest
import numpy as np
from robotx_task1.traffic import Tracker, Track, TrafficLease, StaticMap, clusters, corridor, conflict, traffic_grid, tracking_points


def surface(x=8., y=-6.):
    # Visible two faces of an unknown target, not a filled truth box.
    return np.concatenate([np.column_stack([np.full(21, x-1), np.linspace(y-1.5, y+1.5, 21)]),
                           np.column_stack([np.linspace(x-1, x+1, 15), np.full(15, y+1.5)])])


class TrafficTests(unittest.TestCase):
    def setUp(self):
        self.hull = np.array([[-2.8,-1.27],[2.55,-1.27],[2.55,1.27],[-2.8,1.27]])
        self.path = np.column_stack([np.arange(0,20,.1), np.zeros(200), np.zeros(200)])

    def test_observed_clusters_do_not_merge_separated_bodies(self):
        self.assertEqual(len(clusters(np.concatenate([surface(),surface(14.,-6.)]))),2)

    def test_stationary_returns_do_not_acquire_velocity(self):
        tracker = Tracker()
        for t in np.arange(0,4,.1):tracker.update(surface(),float(t))
        self.assertFalse(tracker.tracks[0].moving)

    def test_far_returns_only_continue_confirmed_near_field_tracks(self):
        track=Track(1,10.,np.array([[25.,-2.],[27.,1.]]),np.zeros(2),True)
        points=np.array([[10.,5.,1.],[26.,0.,1.],[30.,4.,1.],[45.,0.,1.]])
        np.testing.assert_equal(tracking_points(points,[0.,0.],[track],10.1),points[:2])
        track.moving=False
        np.testing.assert_equal(tracking_points(points,[0.,0.],[track],10.1),points[:1])

    def test_continuation_does_not_create_returns_or_clear_a_lost_target(self):
        track=Track(1,10.,np.array([[4.,-2.],[6.,1.]]),np.array([0.,.25]),True)
        self.assertEqual(len(tracking_points(np.empty((0,3)),[0.,0.],[track],40.)),0)
        swept=corridor(self.hull,[0.,0.,0.],self.path,6.)
        self.assertIsNotNone(conflict([track],40.,swept))

    def test_sliding_partial_view_of_static_wall_is_not_translation(self):
        track=Track(1,0.,np.array([[0.,0.],[3.,1.]]))
        for t in np.arange(0,5,.1):track.observe(np.array([[0.,0.],[3.+t*.4,1.]]),float(t))
        self.assertFalse(track.moving)

    def test_only_surveyed_occupied_cells_and_outside_map_are_excluded(self):
        grid=np.zeros((20,20),dtype=int);grid[5:8,5:8]=100
        mapped=StaticMap(grid,1.,[0.,0.,0.],padding=0.)
        points=np.array([[2.,3.,1.],[6.,6.,1.],[25.,1.,1.]])
        np.testing.assert_equal(mapped.unmapped(points),points[:1])

    def test_hold_rotation_disk_rejects_unknown_land_and_boundary(self):
        grid=np.zeros((40,40),dtype=int);grid[20,20]=100;grid[10,10]=-1
        mapped=StaticMap(grid,.5,[0.,0.,0.],padding=0.)
        self.assertTrue(mapped.disk_clear([5.,10.],2.))
        self.assertFalse(mapped.disk_clear([9.,10.],2.))
        self.assertFalse(mapped.disk_clear([4.,5.],2.))
        self.assertFalse(mapped.disk_clear([1.,10.],2.))
        rotated=StaticMap(grid,.5,[30.,-5.,math.pi/2],padding=0.)
        self.assertTrue(rotated.disk_clear([20.,0.],2.))
        self.assertFalse(rotated.disk_clear([20.,4.],2.))

    def test_measured_motion_and_partial_view_retains_front(self):
        tracker = Tracker()
        for t in np.arange(0,4,.1):tracker.update(surface(y=-6+.25*t),float(t))
        track = tracker.tracks[0]
        self.assertTrue(track.moving)
        self.assertAlmostEqual(track.velocity[1],.25,delta=.03)
        p = surface(y=-5.)
        tracker.update(p[p[:,1]<-4.5],4.)
        self.assertGreater(track.bounds[1,1],-3.6)

    def test_long_partial_view_does_not_shrink_to_cabin(self):
        tracker=Tracker()
        for t in np.arange(0,4,.1):tracker.update(surface(y=-6+.25*t),float(t))
        track=tracker.tracks[0]
        for t in np.arange(4,9,.1):
            p=surface(y=-6+.25*t);tracker.update(p[p[:,1]<-6+.25*t+.5],float(t))
        self.assertGreater(track.bounds[1,1],-6+.25*8.9+1.3)

    def test_stopped_partial_feature_does_not_extrapolate_the_body_forever(self):
        tracker=Tracker()
        for t in np.arange(0,4,.1):tracker.update(surface(y=-6+.25*t),float(t))
        x,y=np.meshgrid(np.linspace(7.7,8.3,5),np.linspace(-5.5,-4.5,8))
        for t in np.arange(4,20,.1):tracker.update(np.column_stack([x.ravel(),y.ravel()]),float(t))
        track=tracker.tracks[0]
        self.assertEqual(len(tracker.tracks),1)
        self.assertLess(np.linalg.norm(track.velocity),.02)
        self.assertGreaterEqual(track.bounds[1,1],-3.5)
        self.assertLess(track.bounds[1,1],-2.2)

    def test_disconnected_roof_does_not_leave_a_second_lost_target(self):
        tracker=Tracker();angles=np.linspace(0,2*math.pi,30,endpoint=False)
        for t in np.arange(0,6,.1):
            p=surface(y=-6+.25*t)
            if t>3:
                roof=np.column_stack([8.2+.3*np.cos(angles),-6+.25*t+.3*np.sin(angles)])
                p=np.concatenate([p,roof])
            tracker.update(p,float(t))
        self.assertEqual(len(tracker.tracks),1)
        self.assertTrue(tracker.tracks[0].moving)

    def test_new_bow_fragment_overlaps_but_extends_beyond_previous_envelope(self):
        tracker=Tracker();tracker.next_id=2
        tracker.tracks=[Track(1,10.,np.array([[7.,-7.5],[9.,-4.5]]),np.array([0.,.065]),True)]
        rear=np.concatenate([np.column_stack([np.full(15,7.),np.linspace(-7.3,-5.5,15)]),
                             np.column_stack([np.linspace(7.,9.,15),np.full(15,-7.3)])])
        x,y=np.meshgrid(np.linspace(7.1,8.,7),np.linspace(-4.8,-3.8,8))
        front=np.column_stack([x.ravel(),y.ravel()])
        tracker.update(np.concatenate([rear,front]),10.1)
        self.assertEqual([t.id for t in tracker.tracks],[1])
        self.assertGreater(tracker.tracks[0].bounds[1,1],-4.)

    def test_partial_view_center_jump_uses_envelope_overlap_for_identity(self):
        tracker=Tracker();tracker.next_id=2
        tracker.tracks=[Track(1,10.,np.array([[7.,-6.],[9.,-2.]]),np.array([0.,.25]),True)]
        x,y=np.meshgrid(np.linspace(8.,8.8,6),np.linspace(-2.6,-1.5,8))
        tracker.update(np.column_stack([x.ravel(),y.ravel()]),10.1)
        self.assertEqual([t.id for t in tracker.tracks],[1])

    def test_future_crossing_yields_before_current_contact_and_then_clears(self):
        swept = corridor(self.hull,[0,0,0],self.path,3.6)
        track = Track(1,10.,np.array([[5.,-7.5],[7.,-4.5]]),np.array([0.,.25]),True)
        risk = conflict([track],10.,swept)
        self.assertIsNotNone(risk)
        self.assertGreater(risk['future_s'],0.)
        track.bounds += [0,12]
        self.assertIsNone(conflict([track],10.,swept))

    def test_lost_moving_track_cannot_timeout_to_free_space(self):
        tracker = Tracker()
        tracker.tracks = [Track(1,10.,np.array([[4.,-2.],[6.,1.]]),np.array([0.,.25]),True)]
        tracker.update(np.empty((0,2)),16.)
        swept = corridor(self.hull,[0,0,0],self.path,3.6)
        risk = conflict(tracker.tracks,16.,swept)
        self.assertIsNotNone(risk)
        self.assertEqual(risk['reason'],'lost moving target')

    def test_far_track_is_retired_only_outside_observation_and_prediction_window(self):
        tracker=Tracker()
        tracker.tracks=[Track(1,10.,np.array([[30.,-2.],[32.,1.]]),np.zeros(2),True),
                        Track(2,10.,np.array([[30.,-2.],[32.,1.]]),np.array([-1.,0.]),True)]
        tracker.update(np.empty((0,2)),13.,np.array([0.,0.]))
        self.assertEqual(tracker.retired,[1])
        self.assertEqual([t.id for t in tracker.tracks],[2])

    def test_full_hull_and_turn_sweep_are_checked(self):
        swept = corridor(self.hull,[0,0,math.pi/2],None,3.6)
        track = Track(1,10.,np.array([[0,2.3],[.2,2.5]]),np.zeros(2),True)
        self.assertIsNotNone(conflict([track],10.,swept))

    def test_reordered_cloud_is_rejected(self):
        tracker = Tracker();tracker.update(surface(),5.)
        with self.assertRaises(ValueError):tracker.update(surface(),5.)

    def test_inferred_map_marks_future_body_and_clears_previous_region(self):
        track=Track(1,10.,np.array([[5.,-1.],[7.,1.]]),np.array([0.,.25]),True)
        grid=traffic_grid([track],10.,(100,100),.2,np.array([0.,-10.,0.]))
        self.assertEqual(grid[50,30],100)
        self.assertEqual(grid[70,30],100)
        track.bounds += [0.,8.];track.stamp=11.
        updated=traffic_grid([track],11.,grid.shape,.2,np.array([0.,-10.,0.]))
        self.assertEqual(updated[50,30],0)
        self.assertEqual(updated[90,30],100)

    def test_inferred_layer_contains_no_unobserved_static_geometry(self):
        grid=traffic_grid([],10.,(100,100),.2,np.array([0.,-10.,0.]))
        self.assertEqual(np.count_nonzero(grid),0)

    def test_lease_source_receipt_and_path_must_all_be_current(self):
        lease = TrafficLease()
        self.assertFalse(lease.permits(10.,100.,9.))
        self.assertTrue(lease.update(10.,9.,True,'clear',10.1,100.))
        self.assertTrue(lease.permits(10.2,100.1,9.))
        self.assertFalse(lease.permits(10.2,100.1,10.))
        self.assertFalse(lease.permits(10.5,100.1,9.))
        self.assertFalse(lease.permits(10.2,100.5,9.))
        self.assertFalse(lease.update(10.,9.,True,'replay',10.2,100.1))
        self.assertTrue(lease.update(10.1,9.,False,'conflict',10.2,100.1))
        self.assertFalse(lease.permits(10.2,100.2,9.))

    def test_resumption_requires_sustained_fresh_clearance(self):
        lease=TrafficLease()
        lease.update(10.,9.,False,'conflict',10.1,100.)
        for t in np.arange(10.1,11.3,.1):
            lease.update(float(t),9.,True,'clear',float(t)+.1,float(t)+90.)
        self.assertTrue(lease.clear_for(11.3,101.3))
        lease.update(11.3,9.,False,'conflict',11.4,101.3)
        self.assertFalse(lease.clear_for(11.4,101.4))
        lease.update(11.4,9.,True,'clear',11.5,101.4)
        self.assertFalse(lease.clear_for(11.5,101.5))


if __name__ == '__main__':unittest.main()
