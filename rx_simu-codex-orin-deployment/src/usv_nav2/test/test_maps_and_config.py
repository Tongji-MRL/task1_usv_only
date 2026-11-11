import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from usv_nav2.make_water_map import rasterize, write_map


ROOT = Path(__file__).resolve().parents[1]


class MapTests(unittest.TestCase):
    def area(self):
        return dict(resolution=1.0, padding=1.0,
                    navigable_boundary=[[-2, -2], [2, -2], [2, 2], [-2, 2]],
                    exclusions=[[[0, 0], [1, 0], [1, 1], [0, 1]]])

    def test_boundary_and_exclusion_stay_occupied(self):
        width, height, res, origin, pixels = rasterize(self.area())
        self.assertEqual((width, height, origin), (6, 6, (-3.0, -3.0)))
        at = lambda x, y: pixels[int((y - origin[1]) / res) * width + int((x - origin[0]) / res)]
        self.assertEqual(at(-2.5, 0.5), 0)
        self.assertEqual(at(-1.5, -1.5), 254)
        self.assertEqual(at(0.5, 0.5), 0)
        self.assertEqual(at(1.5, 1.5), 254)

    def test_pgm_north_up_and_origin_metadata(self):
        with tempfile.TemporaryDirectory() as temp:
            prefix = Path(temp) / 'water'
            metadata_path, width, height = write_map(self.area(), prefix)
            metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
            self.assertEqual(metadata['origin'], [-3.0, -3.0, 0.0])
            raw = prefix.with_suffix('.pgm').read_bytes()
            header, size, maximum, image = raw.split(b'\n', 3)
            self.assertEqual(header, b'P5')
            self.assertEqual(size, b'6 6')
            self.assertEqual(maximum, b'255')
            self.assertEqual(len(image), width * height)
            # Exclusion is at +x,+y, not reflected to -y by PGM row ordering.
            self.assertEqual(image[2 * width + 3], 0)
            self.assertEqual(image[3 * width + 3], 254)
            with self.assertRaises(FileExistsError):
                write_map(self.area(), prefix)

    def test_bad_resolution_and_huge_map_rejected(self):
        for resolution in (0, -1, float('nan'), 0.00001):
            area = self.area()
            area['resolution'] = resolution
            with self.assertRaises(ValueError):
                rasterize(area)


class ConfigurationContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT / 'config/nav2_params.yaml').read_text(encoding='utf-8'))

    def test_planner_and_bridge_limits_agree(self):
        controller = self.config['controller_server']['ros__parameters']
        local = controller['FollowPath']
        bridge = self.config['rover_bridge']['ros__parameters']
        self.assertEqual(local['max_vel_x'], bridge['max_speed'])
        self.assertEqual(local['max_speed_xy'], bridge['max_speed'])
        self.assertEqual(local['max_vel_theta'], bridge['max_yaw_rate'])
        self.assertEqual(local['sim_period'], 1.0 / controller['controller_frequency'])
        self.assertEqual(local['min_vel_y'], 0.0)
        self.assertEqual(local['max_vel_y'], 0.0)
        self.assertEqual(local['vy_samples'], 1)
        self.assertIn('ObstacleFootprint', local['critics'])
        self.assertFalse(bridge['enable_control'])

    def test_both_costmaps_enforce_same_boat_and_boundary(self):
        footprints = []
        for name in ('local_costmap', 'global_costmap'):
            config = self.config[name][name]['ros__parameters']
            footprint = json.loads(config['footprint'])
            self.assertAlmostEqual(max(p[0] for p in footprint) - min(p[0] for p in footprint), 1.2)
            self.assertAlmostEqual(max(p[1] for p in footprint) - min(p[1] for p in footprint), 0.7)
            self.assertIn('static_layer', config['plugins'])
            self.assertLess(config['plugins'].index('static_layer'), config['plugins'].index('obstacle_layer'))
            self.assertEqual(config['obstacle_layer']['combination_method'], 1)
            self.assertFalse(config['obstacle_layer']['scan']['inf_is_valid'])
            footprints.append(footprint)
        self.assertEqual(*footprints)
        planner = self.config['planner_server']['ros__parameters']['GridBased']
        self.assertFalse(planner['allow_unknown'])

    def test_nav2_uses_relay_and_bridge_checks_raw_odometry(self):
        adapter = self.config['odom_tf']['ros__parameters']
        self.assertNotEqual(adapter['odom_topic'], adapter['output_odom_topic'])
        for node in ('bt_navigator', 'controller_server'):
            self.assertEqual(self.config[node]['ros__parameters']['odom_topic'],
                             adapter['output_odom_topic'])
        self.assertEqual(self.config['rover_bridge']['ros__parameters']['odom_topic'],
                         adapter['odom_topic'])

    def test_behavior_tree_plugins_and_goal_checker_match(self):
        libs = self.config['bt_navigator']['ros__parameters']['plugin_lib_names']
        tags = {'ComputePathToPose': 'nav2_compute_path_to_pose_action_bt_node',
                'ComputePathThroughPoses': 'nav2_compute_path_through_poses_action_bt_node',
                'FollowPath': 'nav2_follow_path_action_bt_node',
                'RateController': 'nav2_rate_controller_bt_node',
                'PipelineSequence': 'nav2_pipeline_sequence_bt_node',
                'RemovePassedGoals': 'nav2_remove_passed_goals_action_bt_node'}
        for path in (ROOT / 'behavior_trees').glob('*.xml'):
            root = ET.parse(path).getroot()
            for element in root.iter():
                if element.tag in tags:
                    self.assertIn(tags[element.tag], libs)
                if element.tag == 'FollowPath':
                    self.assertIn(element.attrib['goal_checker_id'],
                                  self.config['controller_server']['ros__parameters']['goal_checker_plugins'])


if __name__ == '__main__':
    unittest.main()
