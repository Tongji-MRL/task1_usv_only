"""Analysis of a real rosbag2 directory without ROS (pip package 'rosbags').

This is the path used on a laptop. The rosbag2_py path used on the Orin is
covered by test_ros/test_survey_ros.py.
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import survey_scene as scene  # noqa: E402
from robotx_task1 import survey_bag as bag  # noqa: E402

try:
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
except ImportError:
    Writer = None


def build(store, name, source):
    """Typed message from a duck-typed one; absent fields take zero values."""
    def value(kind, args, given):
        if kind == 1:  # Base type.
            if args[0] == 'string':
                return given or ''
            return given if given is not None else (False if args[0] == 'bool' else 0)
        if kind == 2:
            return build(store, args, given)
        (sub_kind, sub_args), length = args
        if sub_kind == 1 and sub_args[0] != 'string':
            data = given if given is not None else [0]*length
            if isinstance(data, bytes):
                return np.frombuffer(data, np.uint8)
            return np.array(data, dtype=sub_args[0])
        return [value(sub_kind, sub_args, item) for item in (given or [])]
    return store.types[name](**{field: value(kind, args, getattr(source, field, None))
                                for field, (kind, args) in store.fielddefs[name][1]})


@unittest.skipIf(Writer is None, "pip package 'rosbags' not installed")
class RosbagsFileTests(unittest.TestCase):
    def test_recorded_directory_is_analyzed_from_the_command_line(self):
        store = get_typestore(Stores.ROS2_HUMBLE)
        store.register(get_types_from_msg(bag.STATE_MSG, 'mavros_msgs/msg/State'))
        source = scene.Scene(encoding='rgb8')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bag'
            with Writer(path, version=8) as writer:
                connections = {topic: writer.add_connection(topic, kind, typestore=store)
                               for topic, kind in source.topics.items() if kind in store.types}
                for topic, receive, message in source.messages(set(connections)):
                    kind = source.topics[topic]
                    writer.write(connections[topic], receive, store.serialize_cdr(
                        build(store, kind, message), kind))
            recorded = bag.RosbagsBag(path)
            self.assertEqual(recorded.topics['/livox/lidar'], 'sensor_msgs/msg/PointCloud2')
            out = Path(directory)/'out'
            self.assertEqual(bag.main([str(path), '--output', str(out), '--crops']), 0)
            report = json.loads((out/'report.json').read_text())
            self.assertEqual(report['problems'], [])
            self.assertEqual(len(report['lidar']['landmarks']), 3)
            self.assertEqual(report['headline']['lidar_stable_range_m']['min'], 12.0)
            self.assertEqual(report['vehicle']['modes'], ['MANUAL'])
            self.assertTrue(list((out/'crops').glob('*.png')))
            labels = json.loads((out/'labels_template.json').read_text())
            for name, entry in labels.items():
                entry['truth'] = 'red' if entry['left'] > 1 else 'green' if entry['left'] < -1 else 'off'
            (out/'labels.json').write_text(json.dumps(labels))
            self.assertEqual(bag.main([str(path), '--output', str(out), '--labels', str(out/'labels.json')]), 0)
            report = json.loads((out/'report.json').read_text())
            self.assertEqual(report['headline']['camera_color_range_m']['min'], 8.0)


if __name__ == '__main__':
    unittest.main()
