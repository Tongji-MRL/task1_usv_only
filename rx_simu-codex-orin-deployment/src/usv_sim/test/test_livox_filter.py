import math
import struct
import unittest
from types import SimpleNamespace as NS
from collections import deque
from unittest.mock import Mock, patch

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from usv_sim.cloud_filter import xyz_view
from usv_sim.livox_filter import point_times, unknown_empty_bins, transform_points, LivoxFilter


class LivoxFormatTests(unittest.TestCase):
    def cloud(self, times, big=False):
        msg=PointCloud2();msg.height=1;msg.width=len(times)
        msg.header.stamp.sec=10;msg.header.frame_id='mid360_frame'
        msg.point_step=26;msg.row_step=26*len(times);msg.is_bigendian=big
        msg.fields=[PointField(name=k,offset=o,datatype=t,count=1) for k,o,t in
                    [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]]
        msg.data=b''.join(struct.pack(('>' if big else '<')+'ffffBBd',5.,2.,1.,100.,0,1,t) for t in times)
        return msg

    def test_driver_layout_in_both_byte_orders(self):
        for big in (False,True):
            msg=self.cloud([10e9,10.08e9],big)
            times,base=point_times(msg)
            np.testing.assert_equal(times,[10e9,10.08e9])
            self.assertEqual(base,10e9)
            np.testing.assert_equal(xyz_view(msg)[0],[[5,2,1],[5,2,1]])

    def test_timestamp_mismatch_is_rejected_not_restamped(self):
        for times in ([9e9],[10.5e9],[math.nan]):
            with self.assertRaises(ValueError):point_times(self.cloud(times))

    def test_no_return_frame_is_empty_not_infinite_rays(self):
        times,base=point_times(self.cloud([]))
        self.assertEqual(len(times),0)
        out=LivoxFilter.cloud(self.cloud([]).header, np.empty((0,3)))
        self.assertEqual(out.width,0);self.assertEqual(len(out.data),0)
        self.assertEqual(out.header.stamp.sec,10)

    def test_empty_angles_remain_unknown(self):
        result=unknown_empty_bins([2.,math.inf,math.nan,-math.inf])
        self.assertEqual(result[0],2.)
        self.assertTrue(np.isnan(result[1:]).all())

    def test_rotated_mount_uses_rotation_and_translation(self):
        t=NS(transform=NS(rotation=NS(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5)),
                           translation=NS(x=1.,y=2.,z=3.)))
        np.testing.assert_allclose(transform_points([[1,0,0]],t),[[1,3,3]],atol=1e-8)

    def test_wait_for_tf_retains_original_measurement_and_expires_in_wall_time(self):
        msg=self.cloud([10e9])
        node=LivoxFilter.__new__(LivoxFilter)
        node.p={'input_timeout':.5};node.pending=deque([(msg,100.)])
        node.fresh=Mock(return_value=True)
        node.process_cloud=Mock(return_value=False)
        with patch('usv_sim.livox_filter.time.monotonic',return_value=100.1):
            node.process_pending()
        self.assertEqual(len(node.pending),1)
        self.assertEqual(msg.header.stamp.sec,10)
        with patch('usv_sim.livox_filter.time.monotonic',return_value=100.6):
            node.process_pending()
        self.assertEqual(len(node.pending),0)
        self.assertEqual(node.process_cloud.call_count,1)


if __name__=='__main__':unittest.main()
