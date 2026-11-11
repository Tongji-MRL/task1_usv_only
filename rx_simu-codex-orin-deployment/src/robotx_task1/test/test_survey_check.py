"""Verdict logic of the pre-recording check; the ROS node itself is not started."""
import copy
import unittest

import numpy as np

from robotx_task1 import survey_check as check
from robotx_task1.survey_core import StaticTree

IMAGE, INFO = 'sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo'


def series(rate, lag, count=40):
    stamps = (np.arange(count)*1e9/rate).astype(np.int64)+10**18
    return dict(stamps=stamps.tolist(), receives=(stamps-int(lag*1e9)).tolist())


def healthy():
    tree = StaticTree()
    tree.add('base_link', 'livox_frame', [0.2, 0., 0.4], [0., 0., 0., 1.])
    tree.add('base_link', 'camera_link', [0.3, 0., 0.5], [0., 0., 0., 1.])
    tree.add('camera_link', 'camera_color_optical_frame', [0., 0., 0.], [-0.5, 0.5, -0.5, 0.5])
    return dict(
        topics={'/livox/lidar': 'sensor_msgs/msg/PointCloud2', '/usv/odom': 'nav_msgs/msg/Odometry',
                '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
                '/mavros/state': 'mavros_msgs/msg/State', '/camera/color/image_raw': IMAGE,
                '/camera/color/camera_info': INFO, '/scan': 'sensor_msgs/msg/LaserScan',
                '/rosout': 'rcl_interfaces/msg/Log'},
        lidar=dict(series(10, -0.11), frame='livox_frame',
                   fields=dict(x=7, y=7, z=7, intensity=7, tag=2, line=2, timestamp=8),
                   point_offsets=[0.0, 0.0996]),
        odom=dict(series(20, -0.02), frames=['odom', 'base_link']),
        image=dict(series(15, -0.04), topic='/camera/color/image_raw', encoding='rgb8',
                   width=640, height=480, frame='camera_color_optical_frame'),
        info=dict(fx=615.0, width=640, height=480),
        state=dict(connected=True, armed=False, mode='MANUAL'),
        gps=dict(fix_type=6, yaw=9000), static=tree)


def levels(results):
    return [level for level, _ in results]


def failing(results, word):
    return [text for level, text in results if level != 'PASS' and word in text]


class SurveyCheckTests(unittest.TestCase):
    def test_healthy_inputs_pass_and_list_topics_to_record(self):
        seen = healthy()
        results = check.evaluate(seen)
        self.assertEqual(set(levels(results)), {'PASS'}, results)
        topics, missing = check.record_topics(seen['topics'])
        self.assertEqual(missing, [])
        self.assertEqual(topics[:7], ['/livox/lidar', '/usv/odom', '/tf', '/tf_static', '/mavros/state',
                                      '/camera/color/image_raw', '/camera/color/camera_info'])
        self.assertIn('/scan', topics)
        self.assertNotIn('/rosout', topics)
        self.assertNotIn('/usv/cmd_vel', topics)

    def test_missing_mandatory_topics_are_named(self):
        topics, missing = check.record_topics({'/livox/lidar': 'sensor_msgs/msg/PointCloud2'})
        self.assertEqual(topics, ['/livox/lidar'])
        self.assertEqual(missing, ['/usv/odom', '/tf', '/tf_static', '/mavros/state',
                                   'color image (.../color/image_raw)', 'color camera_info'])

    def test_unsynchronized_lidar_clock_fails(self):
        for lag in (-1700000000.0, -0.8, 0.3):
            with self.subTest(lag=lag):
                seen = healthy()
                seen['lidar'].update(series(10, lag))
                self.assertTrue(failing(check.evaluate(seen), 'clock'))
                self.assertIn('FAIL', levels(check.evaluate(seen)))

    def test_wrong_point_format_fails(self):
        seen = healthy()
        del seen['lidar']['fields']['timestamp']
        seen['lidar']['point_offsets'] = None
        self.assertTrue(failing(check.evaluate(seen), 'xfer_format'))
        seen = healthy()
        seen['lidar']['point_offsets'] = [-0.4, 0.05]
        self.assertTrue(failing(check.evaluate(seen), 'per-point'))

    def test_absent_streams_fail_with_the_step_to_fix_them(self):
        for key, word in (('lidar', '/livox/lidar'), ('odom', '/usv/odom'), ('image', 'color image'),
                          ('state', '/mavros/state')):
            with self.subTest(key=key):
                seen = healthy()
                seen[key] = None
                results = check.evaluate(seen)
                self.assertTrue([t for level, t in results if level == 'FAIL' and word in t], results)

    def test_camera_problems(self):
        seen = healthy()
        seen['image']['encoding'] = 'yuv422_yuy2'
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'FAIL' and 'yuv422_yuy2' in t])
        seen = healthy()
        seen['info'] = dict(fx=0.0, width=640, height=480)
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'FAIL' and 'camera_info' in t])
        seen = healthy()
        seen['info']['width'] = 1280
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'FAIL' and 'camera_info' in t])
        seen = healthy()
        seen['image'].update(series(3, -0.04))
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'WARN' and 'Hz' in t])

    def test_missing_mounting_transforms_warn_but_recording_stays_possible(self):
        seen = healthy()
        seen['static'] = StaticTree()
        results = check.evaluate(seen)
        warnings = [t for l, t in results if l == 'WARN']
        self.assertEqual(len(warnings), 2)
        self.assertTrue(any('--lidar-xyz-rpy' in t for t in warnings))
        self.assertTrue(any('--camera-xyz-rpy' in t for t in warnings))
        self.assertNotIn('FAIL', levels(results))

    def test_vehicle_state_is_reported_never_changed(self):
        seen = healthy()
        seen['state'] = dict(connected=False, armed=False, mode='')
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'FAIL' and 'FCU' in t])
        seen = healthy()
        seen['odom']['frames'] = ['map', 'base_link']
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'FAIL' and 'map->base_link' in t])
        seen = healthy()
        seen['gps'] = dict(fix_type=5, yaw=0)
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'WARN' and 'RTK float' in t])
        seen = healthy()
        seen['gps'] = None
        self.assertTrue([t for l, t in check.evaluate(seen) if l == 'WARN' and 'not observable' in t])
        before = copy.deepcopy(healthy()['state'])
        seen = healthy()
        check.evaluate(seen)
        self.assertEqual(seen['state'], before)


if __name__ == '__main__':
    unittest.main()
