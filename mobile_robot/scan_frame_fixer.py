#!/usr/bin/env python3
"""scan_frame_fixer.py

Subscribes to /scan (published by ros_gz_bridge from Ignition Fortress).
Ignition gpu_lidar sets header.frame_id to its internal namespaced path:
    mobile_robot/base_footprint/laser
The ROS TF tree only knows 'laser_frame' (robot_state_publisher).
SLAM Toolbox and RViz2 drop every incoming scan because the frame_id
never resolves, producing the continuous 'queue is full' warning.

This node rewrites frame_id to 'laser_frame' (configurable via the
'target_frame' parameter) and republishes on /scan_fixed.
SLAM Toolbox and RViz2 must subscribe to /scan_fixed.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        self.declare_parameter('target_frame', 'laser_frame')
        self._target_frame = (
            self.get_parameter('target_frame').get_parameter_value().string_value
        )
        self.get_logger().info(
            f'scan_frame_fixer: rewriting /scan frame_id -> {self._target_frame}'
        )
        self._pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self._sub = self.create_subscription(
            LaserScan, '/scan', self._cb, 10
        )

    def _cb(self, msg: LaserScan):
        msg.header.frame_id = self._target_frame
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
