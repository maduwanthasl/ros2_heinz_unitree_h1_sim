#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class HandshakeMotion(Node):
    def __init__(self):
        super().__init__('handshake_motion')

        # Joint topics (from ros2 topic list)
        self.joint_topics = {
            'shoulder_pitch': '/h1/right_shoulder_pitch_joint/cmd_pos',
            'shoulder_roll':  '/h1/right_shoulder_roll_joint/cmd_pos',
            'shoulder_yaw':   '/h1/right_shoulder_yaw_joint/cmd_pos',
            'elbow':          '/h1/right_elbow_joint/cmd_pos',
            'left_elbow':     '/h1/left_elbow_joint/cmd_pos'
        }

        # Publishers
        self.pubs = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in self.joint_topics.items()
        }

        # Neutral pose (roughly default standing)
        self.rest_pose = {
            'shoulder_pitch': 0.0,
            'shoulder_roll':  0.0,
            'shoulder_yaw':   0.0,
            'elbow':          0.0,
            'left_elbow':     0.0
        }

        # Handshake base pose: arm forward, slightly out, elbow bent
        self.shake_pose = {
            'shoulder_pitch': -0.8,   # forward
            'shoulder_roll':  0.2,   # out to the side (tune sign if needed)
            'shoulder_yaw':   0.0,
            'elbow':          1.0,   # bent
            'left_elbow':     1.3
        }

        # Simple state machine
        self.phase = 'move_to_shake'   # 'move_to_shake' -> 'shake' -> 'back_to_rest'
        self.alpha = 0.0               # interpolation factor [0,1]
        self.t = 0.0                   # time for sine during shaking
        self.shake_counter = 0         # how many cycles of shaking

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info('Handshake motion node started.')

    # Linear interpolation and publish
    def interpolate_and_publish(self, from_pose, to_pose, alpha):
        for name in self.joint_topics.keys():
            start = from_pose[name]
            end = to_pose[name]
            value = (1.0 - alpha) * start + alpha * end

            msg = Float64()
            msg.data = value
            self.pubs[name].publish(msg)

    def timer_callback(self):
        dt = 0.02

        if self.phase == 'move_to_shake':
            # interpolate from rest to handshake pose
            self.interpolate_and_publish(self.rest_pose, self.shake_pose, self.alpha)
            self.alpha += 0.01  # speed of going into handshake pose

            if self.alpha >= 1.0:
                self.alpha = 1.0
                self.phase = 'shake'
                self.t = 0.0
                self.shake_counter = 0
                self.get_logger().info('Reached handshake pose. Starting shake...')

        elif self.phase == 'shake':
            # Keep all joints at shake_pose, except elbow which oscillates
            base_elbow = self.shake_pose['elbow']

            # small oscillation around elbow to imitate shaking
            amplitude = 0.25   # radians
            frequency = 3.0    # Hz-ish (tuned via dt)
            elbow_offset = amplitude * math.sin(2.0 * math.pi * frequency * self.t)

            # publish shoulder joints as base
            for name in ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw']:
                msg = Float64()
                msg.data = self.shake_pose[name]
                self.pubs[name].publish(msg)

            # publish elbow with oscillation
            elbow_msg = Float64()
            elbow_msg.data = base_elbow + elbow_offset
            self.pubs['elbow'].publish(elbow_msg)

            # track time and cycle count
            self.t += dt

            # count shakes by zero-crossings of sine
            if abs(elbow_offset) < 1e-3:  # near middle
                self.shake_counter += 1

            # after some shakes, go back to rest
            if self.shake_counter > 12:   # ~6 up-down cycles
                self.phase = 'back_to_rest'
                self.alpha = 0.0
                self.get_logger().info('Finished shaking. Moving back to rest pose.')

        elif self.phase == 'back_to_rest':
            # interpolate back to rest
            self.interpolate_and_publish(self.shake_pose, self.rest_pose, self.alpha)
            self.alpha += 0.01

            if self.alpha >= 1.0:
                self.alpha = 1.0
                # hold rest pose
                self.interpolate_and_publish(self.rest_pose, self.rest_pose, 0.0)
                # You can either stop node or loop again:
                # self.phase = 'move_to_shake'
                # self.alpha = 0.0

                # For now, just stay in rest pose
                self.get_logger().info('Returned to rest pose. Handshake complete.')
                # If you want node to exit automatically:
                # rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HandshakeMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
