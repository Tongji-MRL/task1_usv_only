import math
import unittest
from usv_nav2.core import scan_ranges_valid


class ScanSafety(unittest.TestCase):
    def test_missing_and_corrupt_observations_are_not_fresh_safety_inputs(self):
        for values in ([], [math.nan]*8, [-math.inf]*8, [-1, 0.1, 31]):
            self.assertFalse(scan_ranges_valid(values, 0.3, 30))

    def test_no_hit_is_distinct_from_unknown(self):
        self.assertTrue(scan_ranges_valid([math.inf]*8, 0.3, 30))
        self.assertTrue(scan_ranges_valid([math.nan, 2.0], 0.3, 30))


if __name__ == '__main__':
    unittest.main()
