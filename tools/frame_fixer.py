import sys
import time
import rclpy

from argparse import ArgumentParser
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan


class FrameRelay(Node):
    def __init__(self, name, kind, source, target, frame):
        super().__init__(name)
        self.frame = frame
        self.kind = kind
        self.source = source
        self.last_sample = None
        msg_type = {'Image': Image, 'Imu': Imu, 'LaserScan': LaserScan}[kind]
        self.publisher = self.create_publisher(msg_type, target, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.subscription = self.create_subscription(msg_type, source, self.receive, qos_profile_sensor_data)
        self.watchdog = self.create_timer(10.0, self.check_input)
        self.get_logger().info(f'Waiting for {source}; publishing received samples on {target} with frame {frame}')
        
    def receive(self, msg):
        if self.last_sample is None:
            self.get_logger().info(f'First sample received on {self.source}')
        self.last_sample = time.monotonic()
        msg.header.frame_id = self.frame
        if self.kind == 'Imu':
            msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg.angular_velocity_covariance = [2.5e-5, 0.0, 0.0, 0.0, 2.5e-5, 0.0, 0.0, 0.0, 2.5e-5]
            msg.linear_acceleration_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        self.publisher.publish(msg)

    def check_input(self):
        if self.last_sample is None:
            self.get_logger().warning(f'No samples on {self.source}; check the sensor and bridge. No synthetic samples published.')
        elif time.monotonic() - self.last_sample > 10.0:
            self.get_logger().warning(f'Stale input on {self.source}; check source or paused simulation.')


if __name__ == '__main__':
    args = ArgumentParser()
    args.add_argument('name', help='Node name')
    args.add_argument('kind', help='Message type: Image, Imu, LaserScan')
    args.add_argument('source', help='Input topic name')
    args.add_argument('target', help='Output topic name')
    args.add_argument('frame', help='Frame name to set in header')
    parsed_args = args.parse_args()
    
    rclpy.init()
    node = FrameRelay(
        parsed_args.name, parsed_args.kind, parsed_args.source, parsed_args.target, parsed_args.frame
    )
    try:     
        rclpy.spin(node)
    except KeyboardInterrupt:     
        pass
    finally:     
        node.destroy_node()     
        if rclpy.ok():         
            rclpy.shutdown()
