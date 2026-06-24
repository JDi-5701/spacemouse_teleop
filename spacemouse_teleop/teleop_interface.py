#!/usr/bin/env python3
"""SpaceMouse teleoperation interface (robot-agnostic, two output modes).

The compliance reference ("equilibrium") is what you command — never the measured
pose. Two ways to deliver it, selected by `output_mode`:

  twist: publish a 6-DOF velocity Twist (~/cmd_twist). The CONTROLLER owns and
         integrates the equilibrium (e.g. the Franka admittance_node). Cleanest;
         matches KUKA-style controller-owned equilibrium. Force limiting lives in
         the controller.

  pose:  integrate the SpaceMouse deltas onto an equilibrium pose HERE (initialized
         once from the measured pose) and publish an absolute PoseStamped. For
         controllers that take an absolute equilibrium pose (e.g. KUKA/FRI bridge).
         Optional force limiting blocks motion into a contact.

All topics/scales/signs/frame are parameters -> same code for Franka and KUKA.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R


class TeleopInterface(Node):
    def __init__(self):
        super().__init__('teleop_interface')

        self.output_mode = self.declare_parameter('output_mode', 'twist').value  # 'twist'|'pose'
        self.sm_sub_topic = self.declare_parameter('spacemouse_topic', '/spacemouse/raw_command').value
        # scaling: SpaceMouse units -> m/s and rad/s
        self.lin_scale = self.declare_parameter('lin_scale', 0.02).value
        self.ang_scale = self.declare_parameter('ang_scale', 0.0001).value
        self.sm_sign = np.array(
            self.declare_parameter('sm_axis_sign', [-1.0, -1.0, -1.0, -1.0, 1.0, 1.0]).value)
        # deadzone (SpaceMouse units): below this an axis is treated as 0, so resting
        # noise does not drift the equilibrium.
        self.deadzone = self.declare_parameter('deadzone', 0.05).value

        self.create_subscription(Twist, self.sm_sub_topic, self.sm_callback, 10)
        self.latest_sm = np.zeros(6)

        if self.output_mode == 'twist':
            self.cmd_twist_topic = self.declare_parameter(
                'cmd_twist_topic', '/admittance_node/cmd_twist').value
            self.twist_pub = self.create_publisher(Twist, self.cmd_twist_topic, 10)
            self.get_logger().info(f'Teleop [twist] -> {self.cmd_twist_topic} '
                                   '(controller owns the equilibrium)')
        else:  # pose mode (client owns the equilibrium)
            self.pose_sub_topic = self.declare_parameter('pose_topic', '/world/tcp_pose').value
            self.cmd_pub_topic = self.declare_parameter('cmd_topic', '/world/tcp_command').value
            self.reset_sub_topic = self.declare_parameter('reset_topic', '/reset_teleop').value
            self.ft_sub_topic = self.declare_parameter('force_topic', '/world/ft_data').value
            self.frame_id = self.declare_parameter('frame_id', 'world_frame').value
            self.force_limit = self.declare_parameter('force_limit', 20.0).value
            self.enable_force_limit = self.declare_parameter('enable_force_limit', True).value
            self.target_hz = self.declare_parameter('target_hz', 100.0).value
            self.dt = 1.0 / self.target_hz

            self.curr_pos = None
            self.curr_quat = None
            self.cmd_pos = None      # the virtual equilibrium (integrated client-side)
            self.cmd_quat = None
            self.latest_force = np.zeros(3)
            self.stop_publishing = False

            self.create_subscription(PoseStamped, self.pose_sub_topic, self.pose_callback, 10)
            self.create_subscription(Bool, self.reset_sub_topic, self.reset_callback, 10)
            self.create_subscription(WrenchStamped, self.ft_sub_topic, self.ft_callback, 10)
            self.cmd_pub = self.create_publisher(PoseStamped, self.cmd_pub_topic, 10)
            self.timer = self.create_timer(self.dt, self.pose_loop)
            self.get_logger().info(f'Teleop [pose] {self.pose_sub_topic} -> {self.cmd_pub_topic} '
                                   f'(client owns the equilibrium, frame={self.frame_id})')

    def sm_callback(self, msg):
        raw = np.array([msg.linear.x, msg.linear.y, msg.linear.z,
                        msg.angular.x, msg.angular.y, msg.angular.z])
        raw[np.abs(raw) < self.deadzone] = 0.0   # kill resting noise
        self.latest_sm = self.sm_sign * raw
        if self.output_mode == 'twist':
            # publish a velocity twist (tool frame); the controller integrates it
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.linear.z = (self.latest_sm[0:3] * self.lin_scale).tolist()
            tw.angular.x, tw.angular.y, tw.angular.z = (self.latest_sm[3:6] * self.ang_scale).tolist()
            self.twist_pub.publish(tw)

    # ---------------- pose mode ----------------
    def ft_callback(self, msg):
        self.latest_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def reset_callback(self, msg):
        if msg.data:
            self.stop_publishing = True
            if self.curr_pos is not None:
                self.cmd_pos = self.curr_pos.copy()
                self.cmd_quat = self.curr_quat.copy()
        else:
            self.stop_publishing = False

    def pose_callback(self, msg):
        self.curr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.curr_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                   msg.pose.orientation.z, msg.pose.orientation.w])
        if self.cmd_pos is None:   # init equilibrium ONCE; afterwards only SpaceMouse moves it
            self.cmd_pos = self.curr_pos.copy()
            self.cmd_quat = self.curr_quat.copy()

    def pose_loop(self):
        if self.cmd_pos is None or self.stop_publishing:
            return
        r_base = R.from_quat(self.cmd_quat)          # equilibrium orientation
        delta_lin = self.latest_sm[0:3] * self.lin_scale * self.dt
        if self.enable_force_limit:
            for i in range(3):
                f_val, v_val = self.latest_force[i], delta_lin[i]
                if f_val < -self.force_limit and v_val > 0:
                    delta_lin[i] = 0.0
                elif f_val > self.force_limit and v_val < 0:
                    delta_lin[i] = 0.0
        self.cmd_pos += r_base.apply(delta_lin)
        delta_rot = self.latest_sm[3:6] * self.ang_scale * self.dt
        if np.linalg.norm(delta_rot) > 1e-9:
            self.cmd_quat = (r_base * R.from_rotvec(delta_rot)).as_quat()

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.cmd_pos.tolist()
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = self.cmd_quat.tolist()
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
