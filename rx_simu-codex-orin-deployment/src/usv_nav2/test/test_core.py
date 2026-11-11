import math
import random
import unittest

from usv_nav2.core import (VELOCITY_YAW_RATE_MASK, CommandGate, bounded_command,
                          multiply_quaternions, normalise_quaternion,
                          split_yaw_attitude, stamp_is_fresh)


class TransformTests(unittest.TestCase):
    def test_full_attitude_survives_yaw_split(self):
        random.seed(4)
        for _ in range(100):
            roll, pitch, yaw = (random.uniform(-0.6, 0.6), random.uniform(-0.5, 0.5),
                                random.uniform(-math.pi, math.pi))
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                 cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
            level, residual = split_yaw_attitude(q)
            self.assertAlmostEqual(level[2], sy)
            self.assertAlmostEqual(level[3], cy)
            for actual, expected in zip(multiply_quaternions(level, residual), q):
                self.assertAlmostEqual(actual, expected)

    def test_invalid_quaternions(self):
        for q in [(0, 0, 0, 0), (0, math.nan, 0, 1), (0, 0, math.inf, 1)]:
            with self.assertRaises(ValueError):
                normalise_quaternion(q)

    def test_clock_offset_and_stale_messages(self):
        self.assertTrue(stamp_is_fresh(9.9, 10.0, 0.5))
        self.assertFalse(stamp_is_fresh(9.0, 10.0, 0.5))
        self.assertFalse(stamp_is_fresh(12.0, 10.0, 0.5))
        self.assertFalse(stamp_is_fresh(0.0, 0.0, 0.5))


class ControlTests(unittest.TestCase):
    def ready_gate(self):
        gate = CommandGate()
        gate.update_state(True, True, 'GUIDED', 10.0)
        gate.update_command((0.3, 0.2), 10.0)
        gate.mark('odom', 10.0)
        gate.mark('scan', 10.0)
        return gate

    def test_mavlink_fields(self):
        self.assertEqual(VELOCITY_YAW_RATE_MASK, 1511)
        # Position/acceleration/yaw ignored; x/y velocity and yaw rate used.
        for bit in (1, 2, 4, 32, 64, 128, 256, 1024):
            self.assertTrue(VELOCITY_YAW_RATE_MASK & bit)
        for bit in (8, 16, 512, 2048):
            self.assertFalse(VELOCITY_YAW_RATE_MASK & bit)

    def test_keep_ros_yaw_sign_and_clip(self):
        self.assertEqual(bounded_command((0.3, 0, 0, 0, 0, 0.2), 0.4, 0.4), (0.3, 0.2))
        self.assertEqual(bounded_command((3, 0, 0, 0, 0, -2), 0.4, 0.4), (0.4, -0.4))

    def test_reject_unsupported_or_nonfinite_motion(self):
        for command in ((0.3, 0.2, 0, 0, 0, 0), (-0.2, 0, 0, 0, 0, 0),
                        (math.nan, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, math.inf)):
            with self.assertRaises(ValueError):
                bounded_command(command, 0.4, 0.4)

    def test_each_required_input_stops_on_timeout(self):
        for stale in ('command', 'odom', 'scan', 'state'):
            gate = self.ready_gate()
            for source in gate.received:
                if source != stale:
                    gate.received[source] = 13.0
            result, reason = gate.evaluate(13.0)
            self.assertEqual(result, (0.0, 0.0))
            self.assertIn(stale, reason)

    def test_valid_data_passes(self):
        self.assertEqual(self.ready_gate().evaluate(10.2), ((0.3, 0.2), 'ready'))

    def test_arming_requires_new_command(self):
        gate = CommandGate()
        gate.update_command((0.3, 0.0), 10.0)
        gate.mark('odom', 10.0)
        gate.mark('scan', 10.0)
        gate.update_state(True, True, 'GUIDED', 10.1)
        self.assertEqual(gate.evaluate(10.2)[0], (0.0, 0.0))
        gate.update_command((0.2, 0.0), 10.2)
        self.assertEqual(gate.evaluate(10.3)[0], (0.2, 0.0))

    def test_mode_change_and_invalid_input_drop_cached_command(self):
        gate = self.ready_gate()
        gate.update_state(True, True, 'MANUAL', 10.1)
        self.assertEqual(gate.evaluate(10.2)[0], (0.0, 0.0))
        gate.update_state(True, True, 'GUIDED', 10.2)
        self.assertEqual(gate.evaluate(10.3)[0], (0.0, 0.0))
        gate = self.ready_gate()
        gate.invalidate('command')
        self.assertEqual(gate.evaluate(10.1)[0], (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
