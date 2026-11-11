import json
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from usv_sim.cloud_filter import CloudFilter, inside_polygon, xyz_view
from usv_sim.clearing_cloud import ray_directions
from collections import OrderedDict


class LidarProjectionTests(unittest.TestCase):
    def cloud(self, points):
        points=np.asarray(points,dtype='<f4')
        msg=PointCloud2();msg.header.frame_id='lidar';msg.header.stamp.sec=10
        msg.height=points.shape[0];msg.width=points.shape[1];msg.point_step=12;msg.row_step=12*msg.width
        msg.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
        msg.data=points.tobytes()
        return msg

    def node(self):
        node=CloudFilter.__new__(CloudFilter)
        node.polygon=[[-2,-1],[2,-1],[2,1],[-2,1]];node.offset=np.array([.7,-.3,1.8])
        node.clouds=OrderedDict();node.publisher=Mock();node.scan_publisher=Mock()
        node.clearing_publisher=None
        node.marking_publisher=None
        node.shape=(16,720)
        node.tf=Mock();node.tf.lookup_transform.return_value=NS(transform=NS(rotation=NS(x=0,y=0,z=0,w=1)))
        return node

    def test_remove_self_preserve_external_points_and_source_stamp(self):
        node=self.node();msg=self.cloud([[[0,0,0],[5,0,0],[math.inf]*3]])
        node.on_cloud(msg);output=node.publisher.publish.call_args.args[0]
        self.assertEqual(output.header.stamp.sec,10)
        values=xyz_view(output)
        self.assertTrue(np.isnan(values[0,0]).all())
        np.testing.assert_array_equal(values[0,1],[5,0,0])
        self.assertTrue(np.isinf(values[0,2]).all())

    def test_empty_projection_needs_observed_free_rays(self):
        node=self.node();points=np.full((16,720,3),np.inf,dtype=np.float32)
        points[:,330:390,:]=np.nan  # unobserved sector cannot clear an old obstacle
        node.clouds[(10,0)]=(points,'lidar')
        scan=LaserScan();scan.header.stamp.sec=10;scan.header.frame_id='base_stabilized'
        scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.range_max=20.
        scan.ranges=[math.inf]*720;scan.ranges[360]=3.
        node.on_scan(scan);output=node.scan_publisher.publish.call_args.args[0]
        self.assertTrue(math.isnan(output.ranges[350]))
        self.assertEqual(output.ranges[360],3.)
        self.assertEqual(output.ranges[100],math.inf)

    def test_clearing_cloud_keeps_source_header_and_does_not_invent_marking_hits(self):
        node=self.node();node.clearing_publisher=Mock();node.directions=ray_directions((1,3),-.25,.25)
        msg=self.cloud([[[0,0,0],[5,0,0],[math.inf]*3]]);msg.header.stamp.nanosec=123
        node.on_cloud(msg)
        marked=node.publisher.publish.call_args.args[0];cleared=node.clearing_publisher.publish.call_args.args[0]
        self.assertEqual(cleared.header,marked.header)
        self.assertEqual(cleared.header.stamp.sec,10);self.assertEqual(cleared.header.stamp.nanosec,123)
        self.assertTrue(np.isinf(xyz_view(marked)[0,2]).all())
        self.assertTrue(np.isnan(xyz_view(cleared)[0,0]).all())
        self.assertAlmostEqual(np.linalg.norm(xyz_view(cleared)[0,2]),20.,places=4)

    def test_voxel_marking_uses_stabilized_height_and_keeps_lidar_coordinates(self):
        node=self.node();node.marking_publisher=Mock();node.marking_height=(.25,3.)
        node.tf.lookup_transform.return_value.transform.translation=NS(x=.7,y=-.3,z=1.8)
        points=np.full((16,720,3),np.nan,dtype=np.float32)
        points[0,:4]=[[5,0,-1.4],[5,0,-1.7],[0,0,0],[6,0,3]]
        node.clouds[(10,123)]=(points,'lidar')
        scan=LaserScan();scan.header.stamp.sec=10;scan.header.stamp.nanosec=123;scan.header.frame_id='base_stabilized'
        scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.range_max=20.;scan.ranges=[math.inf]*720
        node.on_scan(scan);marked=node.marking_publisher.publish.call_args.args[0]
        self.assertEqual(marked.width,1);self.assertEqual(marked.header.frame_id,'lidar')
        self.assertEqual(marked.header.stamp,scan.header.stamp)
        np.testing.assert_allclose(xyz_view(marked)[0,0],[5,0,-1.4])

    def test_polygon_padding_is_geometric(self):
        polygon=[[-1,-1],[1,-1],[1,1],[-1,1]]
        self.assertEqual(inside_polygon(np.array([[0,0],[1.05,0],[2,0]]),polygon).tolist(),[True,True,False])


if __name__=='__main__':unittest.main()
