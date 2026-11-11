"""Pure-logic checks for the step-1 survey measurements; no ROS, no hardware."""
import math
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from robotx_task1 import survey_core as core


def buoy_points(center, count=12, radius=0.2, seed=0):
    rng = np.random.default_rng(seed)
    angle = rng.uniform(0, 2*math.pi, count)
    return np.column_stack([center[0]+radius*np.cos(angle), center[1]+radius*np.sin(angle),
                            rng.uniform(0.0, 0.8, count)])


def window(time, boat, centers, **kwargs):
    points = np.vstack([buoy_points(c, seed=i, **kwargs) for i, c in enumerate(centers)]) \
        if centers else np.empty((0, 3))
    return core.Window(time, np.asarray(boat, float), core.candidates(core.cluster(points)))


class PoseAndTransformTests(unittest.TestCase):
    def track(self, rate=0.4):
        times = np.arange(0, 10.01, 0.05)
        positions = np.column_stack([0.5*times, np.zeros_like(times), np.full_like(times, 7.)])
        quaternions = Rotation.from_euler('z', rate*times).as_quat()
        return core.PoseTrack(times, positions, quaternions)

    def test_pose_interpolates_and_refuses_gaps_and_extrapolation(self):
        track = core.PoseTrack([0., 1., 1.2, 1.2, 1.1, 1.4], np.array(
            [[0, 0, 0], [1, 0, 0], [1.2, 0, 0], [9, 9, 9], [9, 9, 9], [1.4, 0, 0]], float),
            Rotation.from_euler('z', [0, .1, .12, 3, 3, .14]).as_quat(), max_gap=0.5)
        positions, rotations, valid = track.at([1.1, 1.3, 0.5, -0.1, 2.0])
        self.assertTrue(np.allclose(positions[:2, 0], [1.1, 1.3]))  # Repeated / old stamps dropped.
        self.assertAlmostEqual(rotations[0].as_euler('zyx')[0], 0.11, places=6)
        self.assertEqual(valid.tolist(), [True, True, False, False, False])

    def test_each_point_uses_the_pose_at_its_own_time(self):
        track = self.track()
        mount_r, mount_t = Rotation.from_euler('z', 0.3), np.array([0.2, -0.1, 0.4])
        target = np.array([9., 2., 7.6])  # 0.6 m above the base_link origin.
        times = np.linspace(4.0, 4.1, 21)
        positions, rotations, _ = track.at(times)
        body = rotations.inv().apply(target-positions)
        sensor = mount_r.inv().apply(body-mount_t)
        world, valid = core.to_world(sensor, times, track, mount_r, mount_t)
        self.assertTrue(valid.all())
        self.assertLess(np.abs(world[:, :2]-target[:2]).max(), 0.02)
        self.assertLess(np.abs(world[:, 2]-0.6).max(), 1e-6)  # Not the EKF altitude.
        # Treating the whole frame as one instant visibly smears the same target.
        frame, _ = core.to_world(sensor, np.full(21, 4.0), track, mount_r, mount_t)
        self.assertGreater(np.abs(frame[:, :2]-target[:2]).max(), 0.2)

    def test_points_outside_the_recorded_odometry_are_marked_invalid(self):
        world, valid = core.to_world(np.ones((2, 3)), np.array([5., 11.]), self.track(),
                                     Rotation.identity(), np.zeros(3))
        self.assertEqual(valid.tolist(), [True, False])
        self.assertEqual(core.to_world(np.empty((0, 3)), np.empty(0), self.track(),
                                       Rotation.identity(), np.zeros(3))[0].shape, (0, 3))

    def test_static_tree_chains_and_inverts_mounting_transforms(self):
        tree = core.StaticTree()
        tree.add('base_link', 'camera_link', [0.3, 0., 0.5], [0., 0., 0., 1.])
        tree.add('/camera_link', 'camera_color_optical_frame', [0., 0., 0.], [-0.5, 0.5, -0.5, 0.5])
        rotation, translation = tree.lookup('base_link', 'camera_color_optical_frame')
        self.assertTrue(np.allclose(rotation.as_matrix(), core.BODY_FROM_OPTICAL, atol=1e-9))
        self.assertTrue(np.allclose(translation, [0.3, 0., 0.5]))
        back_r, back_t = tree.lookup('camera_color_optical_frame', 'base_link')
        self.assertTrue(np.allclose(back_r.apply(translation)+back_t, 0., atol=1e-9))
        self.assertIsNone(tree.lookup('base_link', 'livox_frame'))


class ClusterAndLandmarkTests(unittest.TestCase):
    def test_adjacent_buoys_stay_separate_and_walls_are_not_candidates(self):
        wall = np.column_stack([np.linspace(20, 26, 200), np.full(200, 5.), np.full(200, 0.5)])
        points = np.vstack([buoy_points([10., 0.]), buoy_points([10., 1.5], seed=1), wall,
                            [[3., 3., 0.2], [3.05, 3., 0.2]]])
        clusters = core.cluster(points)
        self.assertEqual(len(clusters), 4)
        kept = core.candidates(clusters, min_points=5, max_extent=1.0)
        self.assertEqual(len(kept), 2)  # Wall too wide, two-point clutter too sparse.
        centers = sorted((c.center.tolist() for c in kept), key=lambda center: center[1])
        self.assertLess(math.dist(centers[0], [10., 0.]), 0.15)
        self.assertLess(math.dist(centers[1], [10., 1.5]), 0.15)
        self.assertTrue(all(c.count == 12 and 0.0 <= c.z_min <= c.z_max <= 0.8 for c in kept))
        self.assertEqual(core.cluster(np.empty((0, 3))), [])

    def test_landmarks_need_repeated_hits_and_trust_near_observations(self):
        windows = []
        for i in range(30):
            boat = [0.5*i, 0.]
            # Far observations are biased by 0.3 m, as a heading error would do.
            bias = 0.3 if i < 15 else 0.0
            centers = [[15.+bias, 0.8+bias], [15., -0.8]]
            if i in (3, 4):
                centers.append([8., 6.])  # Transient clutter, e.g. a wake.
            windows.append(window(0.5*i, boat, centers))
        landmarks = core.build_landmarks(windows, gate=0.5, min_windows=10)
        self.assertEqual([l['id'] for l in landmarks], ['L01', 'L02'])
        first = min(landmarks, key=lambda l: math.dist([l['x'], l['y']], [15., 0.8]))
        self.assertLess(math.dist([first['x'], first['y']], [15., 0.8]), 0.12)
        self.assertEqual(first['hits'], 30)
        self.assertAlmostEqual(first['nearest_range'], math.dist([14.5, 0.], [first['x'], first['y']]),
                               places=1)
        self.assertGreater(first['z_top'], 0.6)

    def test_gate_does_not_merge_buoys_at_the_field_gap(self):
        # 1.5 m clear water between 0.4 m buoys: centres 1.9 m apart.
        windows = [window(0.5*i, [0., 0.], [[6., 0.], [6., 1.9]]) for i in range(12)]
        self.assertEqual(len(core.build_landmarks(windows, gate=0.5, min_windows=10)), 2)


class RangeTableTests(unittest.TestCase):
    def test_detection_band_reports_far_limit_and_near_blind_zone(self):
        windows = []
        for i in range(200):
            x = 0.1*i  # Approach from 20 m to 0.1 m.
            seen = 2.5 <= 20.-x <= 12.
            windows.append(window(0.5*i, [x, 0.], [[20., 0.]] if seen else []))
        table = core.detection_table(windows, [20., 0.], gate=0.5, bin_width=1.0, max_range=25.)
        band = core.stable_band(table, core.detection_ok(min_rate=0.9, min_windows=6))
        self.assertEqual(band, dict(near=3.0, far=12.0))
        by_lo = {row['lo']: row for row in table}
        self.assertEqual(by_lo[5.0]['rate'], 1.0)
        self.assertEqual(by_lo[5.0]['median_points'], 12)
        self.assertEqual(by_lo[15.0]['rate'], 0.0)
        self.assertAlmostEqual(core.farthest_detection(table), 12.0, places=6)

    def test_bins_without_enough_windows_cannot_extend_the_band(self):
        table = [dict(lo=float(i), hi=i+1., windows=w, detected=w) for i, w in enumerate([10, 10, 2, 10])]
        for row in table:
            row['rate'] = 1.0
        band = core.stable_band(table, core.detection_ok(min_rate=0.9, min_windows=6))
        self.assertEqual(band, dict(near=0.0, far=2.0))
        self.assertIsNone(core.stable_band(table[2:3], core.detection_ok(0.9, 6)))


class CameraTests(unittest.TestCase):
    def camera(self):
        return core.Camera(600., 600., 320., 240., 640, 480,
                           Rotation.from_matrix(core.BODY_FROM_OPTICAL), np.array([0.3, 0., 0.5]))

    def test_projection_signs_follow_body_axes(self):
        camera = self.camera()
        uv, depth = camera.project([[10.3, 0., 0.5], [10.3, 1., 0.5], [10.3, 0., 1.5], [-5., 0., 0.5]])
        self.assertTrue(np.allclose(uv[0], [320., 240.]))
        self.assertAlmostEqual(uv[1][0], 320.-60.)  # Port side appears left.
        self.assertAlmostEqual(uv[2][1], 240.-60.)  # Higher appears up.
        self.assertTrue(np.allclose(depth[:3], 10.))
        self.assertLess(depth[3], 0.)

    def test_roi_scales_with_distance_and_rejects_out_of_view(self):
        camera = self.camera()
        near = camera.roi([5.3, 0., 0.5], 0.5, 0.5)
        far = camera.roi([20.3, 0., 0.5], 0.5, 0.5)
        self.assertEqual((near[2]-near[0], near[3]-near[1]), (60, 60))
        self.assertEqual((far[2]-far[0], far[3]-far[1]), (15, 15))
        tiny = camera.roi([200.3, 0., 0.5], 0.5, 0.5, min_px=6)
        self.assertGreaterEqual(tiny[2]-tiny[0], 6)
        self.assertIsNone(camera.roi([-5., 0., 0.5], 0.5, 0.5))   # Behind.
        self.assertIsNone(camera.roi([5.3, 9., 0.5], 0.5, 0.5))   # Outside the image.
        self.assertIsNone(camera.roi([0.5, 0., 0.5], 0.5, 0.5))   # Closer than min depth.

    def test_overlapping_rois_are_detected(self):
        self.assertTrue(core.rois_overlap((0, 0, 10, 10), (9, 9, 20, 20)))
        self.assertFalse(core.rois_overlap((0, 0, 10, 10), (10, 0, 20, 10)))
        self.assertEqual(core.overlapping({'a': (0, 0, 10, 10), 'b': (5, 5, 15, 15), 'c': (40, 0, 50, 10)}),
                         {'a', 'b'})

    def test_level_point_removes_vessel_attitude(self):
        rotation = Rotation.from_euler('zyx', [math.pi/2, 0.1, -0.05])
        body = core.level_to_body([3., 14.], 0.7, np.array([3., 4., 99.]), rotation)
        self.assertTrue(np.allclose(rotation.apply(body), [0., 10., 0.7]))


class ColorTests(unittest.TestCase):
    def roi(self, color, lit=16, size=12, background=(70, 80, 90)):
        image = np.empty((size, size, 3), np.uint8)
        image[:] = background
        image.reshape(-1, 3)[:lit] = color
        return image

    def test_lamp_colors_and_unlit_background(self):
        for label, color in (('red', (235, 30, 25)), ('red', (230, 20, 70)), ('green', (20, 230, 90)),
                              ('blue', (30, 60, 240))):
            with self.subTest(color=color):
                self.assertEqual(core.classify_roi(self.roi(color))['label'], label)
        self.assertEqual(core.classify_roi(self.roi((70, 80, 90)))['label'], 'none')
        self.assertEqual(core.classify_roi(np.empty((0, 0, 3), np.uint8))['label'], 'none')

    def test_overexposed_or_tiny_or_mixed_lamp_is_not_classified(self):
        white = core.classify_roi(self.roi((255, 255, 250)))
        self.assertEqual((white['label'], white['white']), ('none', 16))
        self.assertEqual(core.classify_roi(self.roi((235, 30, 25), lit=3))['label'], 'none')
        mixed = self.roi((235, 30, 25), lit=20)
        mixed.reshape(-1, 3)[10:20] = (20, 230, 90)
        result = core.classify_roi(mixed)
        self.assertEqual((result['label'], result['red'], result['green']), ('none', 10, 10))
        # Dim, desaturated daylight colors on the hull are not a lamp.
        self.assertEqual(core.classify_roi(self.roi((120, 90, 90)))['label'], 'none')

    def test_color_table_and_truth_criteria(self):
        samples = [(4.5, 'red')]*6+[(4.5, 'none')]*6+[(9.5, 'red')]*2+[(9.5, 'none')]*9+[(9.5, 'green')]
        table = core.color_table(samples, bin_width=1.0, max_range=12.)
        near = next(r for r in table if r['lo'] == 4.0)
        far = next(r for r in table if r['lo'] == 9.0)
        self.assertEqual((near['frames'], near['red'], near['none']), (12, 6, 6))
        self.assertTrue(core.color_ok('red')(near))      # 50 % duty of a flashing lamp.
        self.assertFalse(core.color_ok('red')(far))      # Too rarely seen and one wrong color.
        self.assertFalse(core.color_ok('blue_steady')(near))
        self.assertIsNone(core.color_ok('red')(dict(frames=3, red=3, green=0, blue=0, none=0)))
        steady = dict(frames=20, red=0, green=0, blue=19, none=1)
        self.assertTrue(core.color_ok('blue_steady')(steady))
        self.assertTrue(core.color_ok('off')(dict(frames=40, red=0, green=1, blue=0, none=39)))
        self.assertFalse(core.color_ok('off')(dict(frames=40, red=0, green=8, blue=0, none=32)))
        with self.assertRaises(ValueError):
            core.color_ok('purple')

    def test_color_topics_found_for_both_realsense_naming_schemes(self):
        image, info = 'sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo'
        old = {'/camera/color/image_raw': image, '/camera/color/camera_info': info,
               '/camera/aligned_depth_to_color/image_raw': image, '/camera/depth/image_rect_raw': image}
        self.assertEqual(core.find_color_topics(old), ('/camera/color/image_raw', '/camera/color/camera_info'))
        new = {'/camera/camera/color/image_raw': image, '/camera/camera/color/camera_info': info}
        self.assertEqual(core.find_color_topics(new),
                         ('/camera/camera/color/image_raw', '/camera/camera/color/camera_info'))
        self.assertEqual(core.find_color_topics({'/livox/lidar': 'sensor_msgs/msg/PointCloud2'}), (None, None))
        packed = {'/camera/color/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
                  '/camera/color/camera_info': info}
        self.assertEqual(core.find_color_topics(packed),
                         ('/camera/color/image_raw/compressed', '/camera/color/camera_info'))


if __name__ == '__main__':
    unittest.main()
