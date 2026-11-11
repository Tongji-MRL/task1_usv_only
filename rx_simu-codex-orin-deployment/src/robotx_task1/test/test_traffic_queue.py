"""Source-time queue selection under CPU / TF delays, without control outputs."""
from collections import deque
from types import SimpleNamespace as NS
import unittest
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformException
from robotx_task1.traffic_guard import TrafficGuard, seconds


def cloud(stamp):
    msg=PointCloud2();msg.header.frame_id='lidar'
    msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
    return msg


class QueueTests(unittest.TestCase):
    def guard(self,stamps,tf_until=99.):
        def lookup(target,source,stamp):
            t=stamp.nanoseconds/1e9
            if t>tf_until:raise TransformException('Matching FCU TF has not arrived')
            return NS(transform=(target,source,t))
        return NS(pending=deque(map(cloud,stamps),maxlen=5),tracker=NS(stamp=9.),
                  tf=NS(lookup_transform=lookup))

    def test_latest_ready_source_replaces_queued_old_samples(self):
        guard=self.guard([10.,10.1,10.2]);msg,transform,boat,skipped=TrafficGuard.ready_cloud(guard,10.3)
        self.assertEqual(seconds(msg.header.stamp),10.2)
        self.assertEqual(transform,('map','lidar',10.2));self.assertEqual(boat,('map','base_link',10.2))
        self.assertEqual(skipped,2);self.assertFalse(guard.pending)

    def test_newest_missing_tf_does_not_block_a_ready_observation(self):
        guard=self.guard([10.,10.1,10.2],tf_until=10.1)
        msg,_,_,_=TrafficGuard.ready_cloud(guard,10.3)
        self.assertEqual(seconds(msg.header.stamp),10.1)
        self.assertEqual([seconds(m.header.stamp) for m in guard.pending],[10.2])

    def test_expired_future_and_repeated_sources_cannot_be_used(self):
        guard=self.guard([9.,10.,10.2,11.]);guard.tracker.stamp=10.2
        self.assertIsNone(TrafficGuard.ready_cloud(guard,10.5));self.assertFalse(guard.pending)

    def test_waiting_for_source_tf_preserves_timestamp_and_expires(self):
        guard=self.guard([10.2],tf_until=10.1)
        self.assertIsNone(TrafficGuard.ready_cloud(guard,10.3))
        self.assertEqual(seconds(guard.pending[0].header.stamp),10.2)
        self.assertIsNone(TrafficGuard.ready_cloud(guard,10.6));self.assertFalse(guard.pending)


if __name__=='__main__':unittest.main()
