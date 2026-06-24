#!/usr/bin/env python3
"""SpaceMouse HID driver -> ROS2.

Reads the SpaceMouse in a dedicated thread (keeps the HID buffer fresh), auto-
calibrates the resting bias, and publishes a calibrated 6-DOF Twist plus buttons.
All topics/rates are parameters so the same node serves any robot.
"""
import collections
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray

import pyspacemouse


class SpaceMouseDriver(Node):
    def __init__(self):
        super().__init__('spacemouse_driver')

        # --- parameters ---
        self.raw_topic = self.declare_parameter('raw_topic', '/spacemouse/raw_command').value
        self.button_topic = self.declare_parameter('button_topic', '/spacemouse/buttons').value
        self.rate_hz = self.declare_parameter('rate_hz', 100.0).value
        self.static_eps = self.declare_parameter('static_eps', 0.5).value
        self.calib_samples = int(self.declare_parameter('calib_samples', 5).value)

        # --- hardware ---
        self.dev = pyspacemouse.open()
        if self.dev:
            self.get_logger().info('SpaceMouse opened.')
        else:
            self.get_logger().error('Failed to open SpaceMouse.')
            return

        # --- state ---
        self.latest_state = None
        self.bias = np.zeros(6, dtype=np.float32)
        self.bias_set = False
        self.static_buf = collections.deque(maxlen=self.calib_samples)

        self.running = True
        self.read_thread = threading.Thread(target=self._hardware_read_loop, daemon=True)
        self.read_thread.start()

        self.raw_pub = self.create_publisher(Twist, self.raw_topic, 10)
        self.button_pub = self.create_publisher(Int8MultiArray, self.button_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_callback)

    def _hardware_read_loop(self):
        """Drain the HID buffer so we always publish the freshest physical signal."""
        while self.running:
            state = self.dev.read()
            if state:
                self.latest_state = state
            time.sleep(0.001)

    def publish_callback(self):
        if self.latest_state is None:
            return
        state = self.latest_state

        raw_action = np.array([state.y, state.x, state.z,
                               state.roll, state.pitch, state.yaw], dtype=np.float32)
        buttons = state.buttons

        # auto-calibrate resting bias once the device is held still
        if not self.bias_set:
            if np.linalg.norm(raw_action) < self.static_eps:
                self.static_buf.append(raw_action.copy())
                if len(self.static_buf) == self.static_buf.maxlen:
                    self.bias = np.mean(np.stack(self.static_buf), axis=0)
                    self.bias_set = True
                    self.get_logger().info('>>> bias calibration done <<<')
            return

        cal = raw_action - self.bias

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = map(float, cal[0:3])
        msg.angular.x, msg.angular.y, msg.angular.z = map(float, cal[3:6])
        self.raw_pub.publish(msg)

        btn = Int8MultiArray()
        btn.data = [int(b) for b in buttons]
        self.button_pub.publish(btn)

    def destroy_node(self):
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
