"""Contract tests for the greenhouse bridge service handler.

These exercise the handler directly without spinning an executor. The shared
ROS context is bootstrapped once for the whole module so each test instantiates
its own fresh node and simulator.
"""

import unittest

import rclpy

from lupin_msgs.srv import GetTagReading

from lupin_greenhouse_bridge.bridge_node import GreenhouseBridgeNode


SECONDS_IN_DAY = 24 * 60 * 60


class TestBridgeContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = GreenhouseBridgeNode()

    def tearDown(self):
        self.node.destroy_node()

    def _call(self, tag_id):
        request = GetTagReading.Request()
        request.tag_id = tag_id
        response = GetTagReading.Response()
        return self.node._handle_get_tag_reading(request, response)

    def test_unknown_tag_returns_unknown_status(self):
        response = self._call('this-tag-does-not-exist')
        self.assertEqual(
            response.status, GetTagReading.Response.STATUS_UNKNOWN_TAG
        )
        self.assertNotEqual(response.error_message, '')

    def test_known_tag_returns_ok_with_named_readings(self):
        # Tag '1' is in the package default greenhouse with four sensors.
        response = self._call('1')
        self.assertEqual(response.status, GetTagReading.Response.STATUS_OK)
        self.assertEqual(response.error_message, '')
        self.assertEqual(response.reading.tag_id, '1')
        self.assertGreater(len(response.reading.readings), 0)
        for sensor_reading in response.reading.readings:
            self.assertNotEqual(sensor_reading.name, '')

    def test_sim_time_of_day_is_in_24h_window(self):
        response = self._call('1')
        self.assertEqual(response.status, GetTagReading.Response.STATUS_OK)
        self.assertGreaterEqual(response.reading.sim_time_of_day_seconds, 0.0)
        self.assertLess(
            response.reading.sim_time_of_day_seconds, float(SECONDS_IN_DAY)
        )


if __name__ == '__main__':
    unittest.main()
