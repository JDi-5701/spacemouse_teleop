#!/usr/bin/env python3
"""SpaceMouse teleoperation interface (robot-agnostic).

Integrates SpaceMouse 6-DOF deltas onto a smoothed measured TCP pose and publishes
a target pose command. Optional force limiting blocks motion into a contact. Same
control logic that was validated on the KUKA (FRI); every topic/scale/sign/frame is
a parameter so it also drives the Franka admittance node — just pick the config.
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

        # --- topics ---
        self.pose_sub_topic = self.declare_parameter('pose_topic', '/world/tcp_pose').value
        self.cmd_pub_topic = self.declare_parameter('cmd_topic', '/world/tcp_command').value
        self.sm_sub_topic = self.declare_parameter('spacemouse_topic', '/spacemouse/raw_command').value
        self.reset_sub_topic = self.declare_parameter('reset_topic', '/reset_teleop').value
        self.ft_sub_topic = self.declare_parameter('force_topic', '/world/ft_data').value
        self.frame_id = self.declare_parameter('frame_id', 'world_frame').value

        # --- tuning ---
        self.force_limit = self.declare_parameter('force_limit', 20.0).value
        self.enable_force_limit = self.declare_parameter('enable_force_limit', True).value
        self.lin_scale = self.declare_parameter('lin_scale', 0.02).value
        self.ang_scale = self.declare_parameter('ang_scale', 0.0001).value
        self.target_hz = self.declare_parameter('target_hz', 15.0).value
        # per-axis sign/mapping of the SpaceMouse twist [lx,ly,lz,ax,ay,az]
        self.sm_sign = np.array(
            self.declare_parameter('sm_axis_sign', [-1.0, -1.0, -1.0, -1.0, 1.0, 1.0]).value)
        self.dt = 1.0 / self.target_hz

        # --- state ---
        self.curr_pos = None
        self.curr_quat = None
        self.cmd_pos = None
        self.cmd_quat = None
        self.latest_sm = np.zeros(6)
        self.latest_force = np.zeros(3)
        self.is_active = False
        self.stop_publishing = False

        # --- ROS interfaces ---
        self.create_subscription(PoseStamped, self.pose_sub_topic, self.pose_callback, 10)
        self.create_subscription(Twist, self.sm_sub_topic, self.sm_callback, 10)
        self.create_subscription(Bool, self.reset_sub_topic, self.reset_callback, 10)
        self.create_subscription(WrenchStamped, self.ft_sub_topic, self.ft_callback, 10)
        self.cmd_pub = self.create_publisher(PoseStamped, self.cmd_pub_topic, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'Teleop started. pose<-{self.pose_sub_topic}  cmd->{self.cmd_pub_topic}  '
            f'force<-{self.ft_sub_topic}  frame={self.frame_id}')

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
        # Initialize the virtual equilibrium ONCE from the measured pose. After this it is
        # moved ONLY by the SpaceMouse — NOT re-based on the measured pose — so a hand push
        # does not drag the equilibrium (the robot yields, then returns to it on release).
        if self.cmd_pos is None:
            self.cmd_pos = self.curr_pos.copy()
            self.cmd_quat = self.curr_quat.copy()

    def sm_callback(self, msg):
        raw = np.array([msg.linear.x, msg.linear.y, msg.linear.z,
                        msg.angular.x, msg.angular.y, msg.angular.z])
        self.latest_sm = self.sm_sign * raw
        self.is_active = np.linalg.norm(self.latest_sm) > 1e-4

    def control_loop(self):
        if self.cmd_pos is None:          # not initialized yet (no measured pose received)
            return
        if self.stop_publishing:          # held (e.g. after reset) -> don't move/publish
            return

        # The command IS the virtual equilibrium; SpaceMouse deltas accumulate ON IT
        # (not on the measured pose), so external pushes don't drag it.
        r_base = R.from_quat(self.cmd_quat)
        if self.is_active:
            delta_lin = self.latest_sm[0:3] * self.lin_scale * self.dt

            if self.enable_force_limit:
                for i in range(3):
                    f_val = self.latest_force[i]
                    v_val = delta_lin[i]
                    if f_val < -self.force_limit and v_val > 0:
                        delta_lin[i] = 0.0
                    elif f_val > self.force_limit and v_val < 0:
                        delta_lin[i] = 0.0

            self.cmd_pos += r_base.apply(delta_lin)

            delta_rot = self.latest_sm[3:6] * self.ang_scale * self.dt
            if np.linalg.norm(delta_rot) > 1e-9:
                r_delta = R.from_rotvec(delta_rot)
                self.cmd_quat = (r_base * r_delta).as_quat()

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
