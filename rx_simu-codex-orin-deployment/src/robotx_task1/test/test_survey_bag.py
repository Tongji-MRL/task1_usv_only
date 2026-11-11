"""End-to-end survey analysis on synthetic recorded data; no ROS, no hardware."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import survey_scene as scene  # noqa: E402
from robotx_task1 import survey_bag as bag  # noqa: E402

try:
    import cv2
except ImportError:
    cv2 = None


def by_name(report):
    """Map the scene's buoy names to the landmark entries found by the analysis."""
    found = {}
    for name, xy in scene.BUOYS.items():
        nearest = min(report['lidar']['landmarks'], key=lambda l: math.dist((l['x'], l['y']), xy))
        if math.dist((nearest['x'], nearest['y']), xy) < 0.15:
            found[name] = nearest
    return found


class SurveyAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.first = bag.analyze(scene.Scene(), bag.Config())
        cls.names = by_name(cls.first)

    def test_lidar_range_and_landmarks_match_the_scene(self):
        report = self.first
        self.assertEqual(report['problems'], [])
        self.assertEqual(len(report['lidar']['landmarks']), 3)  # Hull return and quay wall excluded.
        self.assertEqual(set(self.names), set(scene.BUOYS))
        for name in ('red', 'green'):
            landmark = self.names[name]
            self.assertEqual(landmark['stable_band']['far'], 12.0)
            self.assertAlmostEqual(landmark['farthest_detection'], scene.LIDAR_LIMIT, delta=0.4)
            self.assertAlmostEqual(landmark['z_top'], scene.LAMP_Z, delta=0.15)
        self.assertEqual(report['headline']['lidar_stable_range_m']['min'], 12.0)
        self.assertEqual(report['lidar']['stamp_source'], 'header')
        self.assertEqual(report['lidar']['mount_source'], 'tf_static')
        self.assertEqual(report['lidar']['frames_without_point_time'], 0)
        # The operator identifies buoys from positions relative to the start pose.
        self.assertAlmostEqual(self.names['red']['ahead'], 20.0, delta=0.2)
        self.assertAlmostEqual(self.names['red']['left'], 1.2, delta=0.2)
        self.assertEqual(report['vehicle']['modes'], ['MANUAL'])
        self.assertEqual(report['vehicle']['gps_fix_types'], {'5': 5, '6': 55})
        self.assertTrue(report['vehicle']['gps_yaw_reported'])
        self.assertAlmostEqual(report['topics']['/livox/lidar']['rate_hz'], 10.0, delta=0.1)
        json.dumps(report)  # Entirely serializable.

    def test_color_range_needs_operator_truth_and_matches_the_scene(self):
        self.assertIsNone(self.first['headline']['camera_color_range_m']['min'])
        labels = {self.names[name]['id']: name for name in scene.BUOYS}
        report = bag.analyze(scene.Scene(), bag.Config(labels=labels))
        self.assertEqual(report['problems'], [])
        camera = report['camera']['landmarks']
        for name in ('red', 'green'):
            entry = camera[self.names[name]['id']]
            self.assertEqual(entry['truth'], name)
            self.assertEqual(entry['color_band']['far'], 8.0)
            lit = [row for row in entry['table'] if row['hi'] <= 8.0]
            self.assertTrue(all(0.35 <= row[name]/row['frames'] <= 0.65 for row in lit))
            self.assertTrue(all(row[name] == 0 for row in entry['table'] if row['lo'] >= 9.0))
        off = camera[self.names['off']['id']]
        self.assertTrue(all(row['none'] == row['frames'] for row in off['table']))
        self.assertEqual(report['headline']['camera_color_range_m']['min'], 8.0)
        self.assertEqual(report['camera']['mount_source'], 'tf_static')
        text = bag.format_report(report)
        self.assertIn('LiDAR stable range', text)
        self.assertIn('8.0', text)

    def test_unsynchronized_lidar_clock_is_reported_not_hidden(self):
        offset = -1_799_990_000*10**9  # Sensor counting from power-on.
        report = bag.analyze(scene.Scene(lidar_clock_offset_ns=offset), bag.Config())
        self.assertEqual(report['lidar']['landmarks'], [])
        self.assertTrue(any('clock' in p and '--lidar-stamp receive' in p for p in report['problems']))
        self.assertIsNone(report['headline']['lidar_stable_range_m']['min'])
        approximate = bag.analyze(scene.Scene(lidar_clock_offset_ns=offset),
                                  bag.Config(lidar_stamp='receive'))
        self.assertEqual(len(by_name(approximate)), 3)
        self.assertEqual(approximate['lidar']['stamp_source'], 'receive')
        self.assertTrue(any('livox_filter' in p for p in approximate['problems']))

    def test_mounting_transforms_are_required_not_guessed(self):
        report = bag.analyze(scene.Scene(duration=20.0, static_tf=False), bag.Config())
        self.assertEqual(report['lidar']['landmarks'], [])
        self.assertTrue(any('--lidar-xyz-rpy' in p for p in report['problems']))
        config = bag.Config(lidar_mount=scene.LIDAR_MOUNT+(0., 0., 0.),
                            camera_mount=scene.CAMERA_MOUNT+(0., 0., 0.))
        report = bag.analyze(scene.Scene(static_tf=False), config)
        self.assertEqual(report['problems'], [])
        self.assertEqual(len(by_name(report)), 3)
        self.assertEqual((report['lidar']['mount_source'], report['camera']['mount_source']),
                         ('argument', 'argument'))
        self.assertGreater(report['camera']['frames_used'], 100)

    def test_labels_are_validated_against_landmarks(self):
        with self.assertRaises(ValueError):
            bag.analyze(scene.Scene(duration=5.0), bag.Config(labels={'L01': 'purple'}))
        red = self.names['red']
        moved = {red['id']: dict(truth='red', x=red['x']+3.0, y=red['y']), 'L99': 'green'}
        report = bag.analyze(scene.Scene(), bag.Config(labels=moved))
        self.assertTrue(any(red['id'] in p and 'moved' in p for p in report['problems']))
        self.assertTrue(any('L99' in p for p in report['problems']))

    def test_outputs_and_crops_are_written(self):
        labels = {self.names['red']['id']: 'red'}
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            report = bag.analyze(scene.Scene(encoding='rgba8'),
                                 bag.Config(labels=labels, crops_dir=str(out/'crops')))
            bag.write_outputs(report, out)
            saved = json.loads((out/'report.json').read_text())
            self.assertEqual(saved['headline'], report['headline'])
            template = json.loads((out/'labels_template.json').read_text())
            self.assertEqual(template[self.names['red']['id']]['truth'], 'red')
            self.assertEqual(template[self.names['green']['id']]['truth'], '')
            crops = sorted((out/'crops').glob('*.png'))
            self.assertTrue(crops)
            self.assertTrue(all(p.read_bytes().startswith(b'\x89PNG\r\n\x1a\n') for p in crops))
            if cv2 is not None:
                lit = [p for p in crops if '_red_' in p.name]
                image = cv2.imdecode(np.frombuffer(lit[0].read_bytes(), np.uint8), cv2.IMREAD_COLOR)
                self.assertGreater(int((image[..., 2] > 200).sum()), 0)  # The red lamp is in view.


class DecodeTests(unittest.TestCase):
    def test_image_encodings_and_row_padding(self):
        expected = scene.Scene().image(0.0)
        for encoding in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            with self.subTest(encoding=encoding):
                message = next(scene.Scene(encoding=encoding).images())[2]
                self.assertTrue(np.array_equal(bag.image_rgb(message), expected))
        message.encoding = 'mono8'
        with self.assertRaisesRegex(ValueError, 'mono8'):
            bag.image_rgb(message)

    @unittest.skipIf(cv2 is None, 'OpenCV not installed')
    def test_compressed_image(self):
        expected = scene.Scene().image(0.0)
        ok, data = cv2.imencode('.png', expected[..., ::-1])
        decoded = bag.image_rgb(NS(header=None, format='png', data=data.tobytes()))
        self.assertTrue(ok and np.array_equal(decoded, expected))

    def test_livox_cloud_layout_and_missing_point_times(self):
        message = next(scene.Scene().clouds())[2]
        xyz, times = bag.cloud_arrays(message)
        self.assertEqual(xyz.shape, (message.width, 3))
        self.assertTrue(np.allclose(xyz[0], [-0.3, 0.0, -0.2]))
        self.assertTrue(np.all(times-scene.EPOCH_NS >= 0) and np.all(times-scene.EPOCH_NS <= 1e8))
        message.fields = [f for f in message.fields if f.name != 'timestamp']
        self.assertIsNone(bag.cloud_arrays(message)[1])
        message.fields = [f for f in message.fields if f.name != 'z']
        with self.assertRaises(ValueError):
            bag.cloud_arrays(message)

    def test_mount_argument_uses_static_transform_publisher_convention(self):
        rotation, translation = bag.mount_pose((1., 2., 3., 0., 0., math.pi/2))
        self.assertTrue(np.allclose(rotation.apply([1., 0., 0.]), [0., 1., 0.]))
        self.assertTrue(np.allclose(translation, [1., 2., 3.]))


if __name__ == '__main__':
    unittest.main()
