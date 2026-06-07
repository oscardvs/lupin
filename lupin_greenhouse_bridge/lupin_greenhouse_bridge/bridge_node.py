"""ROS 2 wrapper around the mdp-greenhouse simulator.

Exposes ``~/get_tag_reading`` which returns a TagReading for a given tag id,
or STATUS_UNKNOWN_TAG if the tag is not in the loaded greenhouse config.
"""

import random
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from greenhouse_sim.simulator import GreenhouseSimulator

from lupin_msgs.msg import SensorReading, TagReading
from lupin_msgs.srv import GetTagReading


class GreenhouseBridgeNode(Node):
    """Wraps a single GreenhouseSimulator instance behind a ROS 2 service."""

    def __init__(self):
        super().__init__('greenhouse_bridge')

        self.declare_parameter('tag_file', '')
        self.declare_parameter('sim_config_file', '')
        self.declare_parameter('debug_time_of_day', -1.0)
        self.declare_parameter('speedup_factor', 0.0)
        self.declare_parameter('debug_seed', -1)

        # mdp-greenhouse's TagManager assigns the path arg directly to
        # self.file_path and later calls .exists() on it — passing a raw str
        # crashes with AttributeError. Still reproduces on 1.0.8; wrap in Path
        # here until upstream fixes.
        tag_file_str = self.get_parameter('tag_file').value
        cfg_file_str = self.get_parameter('sim_config_file').value
        tag_file = Path(tag_file_str) if tag_file_str else None
        cfg_file = Path(cfg_file_str) if cfg_file_str else None
        debug_tod = float(self.get_parameter('debug_time_of_day').value)
        speedup = float(self.get_parameter('speedup_factor').value)
        debug_seed = int(self.get_parameter('debug_seed').value)

        self._sim = GreenhouseSimulator(tag_file=tag_file, sim_config_file=cfg_file)
        self._tag_set = set(self._sim.tags())
        self._apply_overrides(debug_tod, speedup, debug_seed)

        # Mutually-exclusive callback group makes the threading model explicit:
        # at most one service handler runs at a time, so the simulator instance
        # is never touched concurrently. Holds even if the executor is later
        # swapped to multi-threaded.
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._service = self.create_service(
            GetTagReading,
            '~/get_tag_reading',
            self._handle_get_tag_reading,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'Greenhouse bridge ready: {len(self._tag_set)} tags, '
            f'sensors={self._sim.sensors()}, '
            f"debug_mode={self._sim.sim_config['time']['debug_mode']}"
        )

    def _apply_overrides(self, debug_tod, speedup, debug_seed):
        time_cfg = self._sim.sim_config['time']
        # Setting either debug param flips debug_mode on; otherwise leave
        # whatever was in the loaded config alone.
        if debug_tod >= 0.0 or debug_seed >= 0:
            time_cfg['debug_mode'] = True
            if debug_tod >= 0.0:
                time_cfg['debug_time_of_day'] = debug_tod
            if debug_seed >= 0:
                time_cfg['debug_seed'] = debug_seed
                # GreenhouseSimulator only seeds in __init__, so re-seed here.
                random.seed(debug_seed)
        if speedup > 0.0:
            time_cfg['speedup_factor'] = speedup

    def _handle_get_tag_reading(self, request, response):
        tag_id = request.tag_id
        if tag_id not in self._tag_set:
            response.status = GetTagReading.Response.STATUS_UNKNOWN_TAG
            response.error_message = (
                f"Tag '{tag_id}' is not in the loaded greenhouse "
                f'({len(self._tag_set)} tags known).'
            )
            return response

        wall_now = self.get_clock().now().to_msg()
        data = self._sim.get_sensor_data(tag_id)

        reading = TagReading()
        reading.tag_id = tag_id
        reading.stamp = wall_now
        reading.sim_time_of_day_seconds = float(self._sim.current_time())

        for name, value in data.items():
            if name == 'timestamp':
                continue
            sr = SensorReading()
            sr.name = name
            sr.value = float(value)
            reading.readings.append(sr)

        response.status = GetTagReading.Response.STATUS_OK
        response.error_message = ''
        response.reading = reading
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GreenhouseBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
