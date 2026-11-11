"""Deployment configuration checks; no ROS nodes or hardware connections."""
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest

import yaml
from prepare_hardware import ROOT, generate, settings


class HardwareSetupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        cfg = self.root/'src/usv_nav2/config'
        cfg.mkdir(parents=True)
        shutil.copy(ROOT/'src/usv_nav2/config/nav2_params.yaml', cfg/'nav2_params.yaml')
        water = self.root/'water.yaml'
        water.write_text('image: water.pgm\n')
        self.env = dict(USV_FCU_URL='/dev/example:115200', USV_FCU_SYSID='1',
                        USV_LIDAR_FRAME='livox_frame',
                        USV_MARK_MIN_Z='0.1', USV_MARK_MAX_Z='1.5',
                        USV_MAP_LAT='1.3', USV_MAP_LON='103.8', USV_MAP_YAML=str(water),
                        USV_MAX_SPEED='0.2', USV_MAX_YAW_RATE='0.2')
        self.env['USV_FOOTPRINT'] = json.dumps([[1,.5],[1,-.5],[-1,-.5],[-1,.5]])
        self.env.update({'USV_LIDAR_'+k:'0' for k in ('X','Y','Z','ROLL','PITCH','YAW')})

    def test_missing_measured_values_do_not_write_partial_configuration(self):
        for name in ('USV_FOOTPRINT', 'USV_FCU_URL', 'USV_LIDAR_YAW', 'USV_MAP_LAT'):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, name):
                    generate(dict(self.env, **{name:''}), self.root)
        self.assertFalse((self.root/'runs/hardware_config').exists())

    def test_reject_nonfinite_limits_invalid_frame_and_invalid_hull(self):
        star = [[math.cos(i*4*math.pi/5), math.sin(i*4*math.pi/5)] for i in range(5)]
        bad = [dict(USV_MAX_SPEED='nan'), dict(USV_MAX_SPEED='0.5'),
               dict(USV_MAX_YAW_RATE='0'), dict(USV_MAP_LAT='91'),
               dict(USV_LIDAR_FRAME='base_link'), dict(USV_FCU_SYSID='0'),
               dict(USV_MARK_MAX_Z='0'), dict(USV_FOOTPRINT='[[0,0],[1,1],[2,2]]'),
               dict(USV_FOOTPRINT=json.dumps(star))]
        for change in bad:
            with self.subTest(change=change), self.assertRaises(ValueError):
                settings(dict(self.env, **change))

    def test_generated_profile_keeps_control_off_and_limits_consistent(self):
        files = generate(self.env, self.root)
        nav, lidar = [yaml.safe_load(Path(p).read_text()) for p in files]
        bridge = nav['rover_bridge']['ros__parameters']
        self.assertFalse(bridge['enable_control'])
        self.assertFalse(bridge['enable_stationkeeping'])
        self.assertFalse(bridge['require_task_permission'])
        self.assertEqual(nav['odom_tf']['ros__parameters']['expected_parent_frame'], 'odom')
        dwb = nav['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual((dwb['max_vel_x'],dwb['max_speed_xy'],dwb['max_vel_theta']), (.2,.2,.2))
        self.assertEqual(bridge['max_speed'], dwb['max_vel_x'])
        for name in ('local_costmap', 'global_costmap'):
            cost = nav[name][name]['ros__parameters']
            self.assertEqual(json.loads(cost['footprint']), json.loads(self.env['USV_FOOTPRINT']))
            self.assertEqual(cost['footprint_padding'], .2)
        self.assertEqual(lidar['livox_filter']['ros__parameters']['fixed_frame'], 'odom')

    def test_existing_field_configuration_is_preserved(self):
        out = self.root/'runs/hardware_config'
        out.mkdir(parents=True)
        (out/'livox_filter.yaml').write_text('field configuration\n')
        with self.assertRaises(FileExistsError):
            generate(self.env, self.root)
        self.assertFalse((out/'nav2.yaml').exists())
        self.assertEqual((out/'livox_filter.yaml').read_text(), 'field configuration\n')


if __name__ == '__main__':
    unittest.main()
