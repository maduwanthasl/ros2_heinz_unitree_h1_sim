#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SimpleJointMover(Node):
    def __init__(self):
        super().__init__('simple_joint_mover')

        # ✅ joint topic from your `ros2 topic list`
        joint_topic = '/h1/right_shoulder_pitch_joint/cmd_pos'

        self.publisher_ = self.create_publisher(Float64, joint_topic, 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.t = 0.0
        self.get_logger().info(f'Publishing commands to: {joint_topic}')

    def timer_callback(self):
        # Smooth sine motion between -0.3 and +0.3 rad
        amplitude = 0.3
        speed = 0.5  # frequency scaling

        msg = Float64()
        msg.data = amplitude * math.sin(self.t * speed)

        self.publisher_.publish(msg)
        self.t += 0.02


def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
