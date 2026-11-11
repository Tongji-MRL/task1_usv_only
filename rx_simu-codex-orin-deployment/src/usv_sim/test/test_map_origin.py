import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace
from usv_sim.map_origin import local_xy, MapOrigin


class OriginTests(unittest.TestCase):
    def test_equal_origin_and_observed_wind_drift(self):
        self.assertEqual(local_xy(-33.724223,150.679736),(0.,0.))
        x,y=local_xy(-33.7242275,150.6797341)
        self.assertAlmostEqual(x,-.176,delta=.002)
        self.assertAlmostEqual(y,-.499,delta=.002)

    def test_invalid_origin_rejected(self):
        with self.assertRaises(ValueError):local_xy(float('nan'),150.)
        with self.assertRaises(ValueError):local_xy(100.,150.)

    def test_lost_service_reply_cannot_block_origin_discovery_forever(self):
        request=Mock();request.done.return_value=False
        client=Mock();client.service_is_ready.return_value=True
        node=SimpleNamespace(origin=None,request=request,request_at=100.,client=client,get_logger=lambda:Mock())
        with patch('usv_sim.map_origin.time.monotonic',return_value=105.9):MapOrigin.ask_origin(node)
        client.call_async.assert_not_called()
        with patch('usv_sim.map_origin.time.monotonic',return_value=106.1):MapOrigin.ask_origin(node)
        request.cancel.assert_called_once()
        sent=client.call_async.call_args.args[0]
        self.assertEqual(sent.command,512);self.assertEqual(sent.param1,49.)
        node.origin=(0.,0.)
        with patch('usv_sim.map_origin.time.monotonic',return_value=120.):MapOrigin.ask_origin(node)
        self.assertEqual(client.call_async.call_count,1)
