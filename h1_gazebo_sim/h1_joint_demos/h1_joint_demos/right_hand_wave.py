#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class RightHandWave(Node):
    def __init__(self):
        super().__init__('right_hand_wave')

        # Joint topics (from your ros2 topic list)
        self.joint_topics = {
            'shoulder_pitch': '/h1/right_shoulder_pitch_joint/cmd_pos',
            'shoulder_roll':  '/h1/right_shoulder_roll_joint/cmd_pos',
            'shoulder_yaw':   '/h1/right_shoulder_yaw_joint/cmd_pos',
            'elbow':          '/h1/right_elbow_joint/cmd_pos',
            'left_elbow':          '/h1/left_elbow_joint/cmd_pos',
        }

        # Publishers
        self.pubs = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in self.joint_topics.items()
        }

        # Neutral pose (standing)
        self.rest_pose = {
            'shoulder_pitch': 0.0,
            'shoulder_roll':  0.0,
            'shoulder_yaw':   0.0,
            'elbow':          0.0,
            'left_elbow':          0.0,
        }

        # Base wave pose: arm raised and elbow bent
        self.wave_base_pose = {
            'shoulder_pitch': 0.1,   # arm up in front
            'shoulder_roll':  -1.5,   # slight outwards
            'shoulder_yaw':   -1.5,   # will be oscillated
            'elbow':          -0.43,   # bent
            'left_elbow':      1.3,
        }

        # Simple state machine
        self.phase = 'move_to_wave'   # 'move_to_wave' -> 'wave' -> 'back_to_rest'
        self.alpha = 0.0              # interpolation [0,1]
        self.t = 0.0                  # time for sine wave
        self.wave_cycles = 0          # count of waves

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info('Right hand wave node started.')

    def interpolate_and_publish(self, pose_from, pose_to, alpha):
        """Interpolate between two poses and publish joint commands."""
        for name in self.joint_topics.keys():
            start = pose_from[name]
            end = pose_to[name]
            value = (1.0 - alpha) * start + alpha * end

            msg = Float64()
            msg.data = value
            self.pubs[name].publish(msg)

    def timer_callback(self):
        dt = 0.02
        step = 0.01  # interpolation speed

        if self.phase == 'move_to_wave':
            # Smoothly go from rest to the wave base pose
            self.interpolate_and_publish(self.rest_pose, self.wave_base_pose, self.alpha)
            self.alpha += step

            if self.alpha >= 1.0:
                self.alpha = 1.0
                self.phase = 'wave'
                self.t = 0.0
                self.wave_cycles = 0
                self.get_logger().info('Reached wave pose. Starting to wave...')

        elif self.phase == 'wave':
            # Keep shoulder_pitch, roll, yaw at base pose
            for name in ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw']:
                msg = Float64()
                msg.data = self.wave_base_pose[name]
                self.pubs[name].publish(msg)

            # Oscillate elbow (bend and straighten)
            amplitude = 0.4   # rad (~23 degrees each side)
            frequency = 2.0   # waves per second approx

            elbow = self.wave_base_pose['elbow'] + amplitude * math.sin(2.0 * math.pi * frequency * self.t)
            elbow_msg = Float64()
            elbow_msg.data = elbow
            self.pubs['elbow'].publish(elbow_msg)

            self.t += dt

            # count waves roughly by zero crossings
            elbow_offset = amplitude * math.sin(2.0 * math.pi * frequency * self.t)
            if abs(elbow_offset) < 1e-3:
                self.wave_cycles += 1

            # after some cycles, go back to rest
            if self.wave_cycles > 12:  # ~6 full left-right waves
                self.phase = 'back_to_rest'
                self.alpha = 0.0
                self.get_logger().info('Finished waving. Moving back to rest pose.')

        elif self.phase == 'back_to_rest':
            self.interpolate_and_publish(self.wave_base_pose, self.rest_pose, self.alpha)
            self.alpha += step

            if self.alpha >= 1.0:
                self.alpha = 1.0
                # Hold rest pose
                self.interpolate_and_publish(self.rest_pose, self.rest_pose, 0.0)
                self.get_logger().info('Returned to rest pose. Wave complete.')
                # You can loop again by uncommenting:
                # self.phase = 'move_to_wave'
                # self.alpha = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = RightHandWave()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
