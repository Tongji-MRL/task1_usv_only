"""Survey tools in a real ROS 2 Humble environment; no hardware, no control.

Run on a separate ROS_DOMAIN_ID with the hardware environment sourced:
    python3 -m unittest discover -s src/robotx_task1/test_ros -p 'test_survey_ros.py' -v
Writes a synthetic bag with rosbag2_py and analyzes it through the same reader
that is used for field recordings, then starts the check node with no sensors.
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import rosbag2_py
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).parents[1]/'test'))
import survey_scene as scene  # noqa: E402
from robotx_task1 import survey_bag, survey_check  # noqa: E402

TYPES = {'/livox/lidar': PointCloud2, '/usv/odom': Odometry, '/tf_static': TFMessage,
         '/mavros/state': State, '/camera/color/image_raw': Image,
         '/camera/color/camera_info': CameraInfo}


def fill(target, source):
    """Copy a duck-typed message into a ROS message, field by field."""
    for name, value in vars(source).items():
        if isinstance(value, SimpleNamespace):
            fill(getattr(target, name), value)
        elif name == 'fields':
            target.fields = [PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                             for f in value]
        elif name == 'transforms':
            target.transforms = [fill(TransformStamped(), item) for item in value]
        elif name in ('k', 'd'):
            setattr(target, name, [float(v) for v in value])
        else:
            setattr(target, name, value)
    return target


class SurveyRosTests(unittest.TestCase):
    def test_bag_written_by_rosbag2_is_analyzed(self):
        source = scene.Scene(encoding='rgb8')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bag'
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('cdr', 'cdr'))
            for topic in TYPES:
                writer.create_topic(rosbag2_py.TopicMetadata(
                    name=topic, type=source.topics[topic], serialization_format='cdr'))
            for topic, receive, message in source.messages(set(TYPES)):
                writer.write(topic, serialize_message(fill(TYPES[topic](), message)), receive)
            del writer  # Closes the storage.
            out = Path(directory)/'out'
            self.assertIsInstance(survey_bag.open_bag(path), survey_bag.Rosbag2Bag)
            self.assertEqual(survey_bag.main([str(path), '--output', str(out), '--crops']), 0)
            report = json.loads((out/'report.json').read_text())
            self.assertEqual(report['problems'], [])
            self.assertEqual(len(report['lidar']['landmarks']), 3)
            self.assertEqual(report['headline']['lidar_stable_range_m']['min'], 12.0)
            self.assertEqual(report['vehicle']['modes'], ['MANUAL'])
            self.assertTrue(list((out/'crops').glob('*.png')))
            labels = json.loads((out/'labels_template.json').read_text())
            for entry in labels.values():
                entry['truth'] = 'red' if entry['left'] > 1 else 'green' if entry['left'] < -1 else 'off'
            (out/'labels.json').write_text(json.dumps(labels))
            self.assertEqual(survey_bag.main([str(path), '--output', str(out),
                                              '--labels', str(out/'labels.json')]), 0)
            report = json.loads((out/'report.json').read_text())
            self.assertEqual(report['headline']['camera_color_range_m']['min'], 8.0)

    def test_check_node_starts_and_fails_closed_without_sensors(self):
        with tempfile.TemporaryDirectory() as directory:
            result = Path(directory)/'check.json'
            self.assertEqual(survey_check.main(['--duration', '1', '--json', str(result)]), 1)
            saved = json.loads(result.read_text())
            self.assertFalse(saved['passed'])
            self.assertIn('/livox/lidar', saved['missing'])


if __name__ == '__main__':
    unittest.main()
