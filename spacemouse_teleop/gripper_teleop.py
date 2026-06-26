#!/usr/bin/env python3
"""SpaceMouse buttons -> Franka gripper (open / close).

Subscribes the driver's button topic (Int8MultiArray) and maps two buttons to the
franka_gripper actions on a RISING EDGE only (press, not hold):

  open_button  -> Move  (open to open_width)   : fast, no force
  close_button -> Grasp (close with force)     : keeps clamping force on the object

Edge detection means one press = one action; holding the button does nothing extra.
A new press while an action is still running is ignored (busy guard) so we never
queue up goals. All topics / indices / grasp params are ROS parameters, so the same
node works for any gripper that exposes franka_msgs Move/Grasp actions.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int8MultiArray, Float64

from franka_msgs.action import Move, Grasp


class GripperTeleop(Node):
    def __init__(self):
        super().__init__('gripper_teleop')

        # --- topics / button mapping ---
        self.button_topic = self.declare_parameter('button_topic', '/spacemouse/buttons').value
        self.open_button = int(self.declare_parameter('open_button', 0).value)
        self.close_button = int(self.declare_parameter('close_button', 1).value)
        # gripper action namespace (the franka_gripper node name, optionally namespaced)
        gripper_ns = self.declare_parameter('gripper_ns', '/franka_gripper').value.rstrip('/')

        # --- open (Move) params ---
        self.open_width = float(self.declare_parameter('open_width', 0.08).value)   # FR3 max ~0.08 m
        self.open_speed = float(self.declare_parameter('open_speed', 0.1).value)    # m/s

        # --- close (Grasp) params ---
        self.grasp_width = float(self.declare_parameter('grasp_width', 0.0).value)  # target gap [m]
        self.grasp_speed = float(self.declare_parameter('grasp_speed', 0.1).value)  # m/s
        self.grasp_force = float(self.declare_parameter('grasp_force', 40.0).value)  # N
        # epsilon = how far off grasp_width the final width may be and still "succeed"
        self.grasp_eps_inner = float(self.declare_parameter('grasp_epsilon_inner', 0.08).value)
        self.grasp_eps_outer = float(self.declare_parameter('grasp_epsilon_outer', 0.08).value)

        self.move_client = ActionClient(self, Move, f'{gripper_ns}/move')
        self.grasp_client = ActionClient(self, Grasp, f'{gripper_ns}/grasp')

        # --- latched gripper COMMAND for data recording -----------------------------
        # The gripper itself is action-driven and slow (~2 s), but the *commanded* state is
        # instantaneous and binary. Publish it as a single normalized scalar (open/close) at a
        # steady rate so a LeRobot recorder can sample it as the gripper action dim (pi0.5
        # convention: one gripper value appended to the action vector).
        self.cmd_topic = self.declare_parameter('gripper_command_topic', '/gripper_command').value
        self.open_value = float(self.declare_parameter('gripper_open_value', 1.0).value)
        self.close_value = float(self.declare_parameter('gripper_close_value', 0.0).value)
        cmd_rate = float(self.declare_parameter('gripper_command_rate', 30.0).value)
        # assume the gripper starts open (typical after homing); updated on first press
        self._gripper_cmd = self.open_value
        self.cmd_pub = self.create_publisher(Float64, self.cmd_topic, 10)
        self.create_timer(1.0 / max(1.0, cmd_rate), self._publish_cmd)

        self.prev = {}          # button index -> last raw value (for rising-edge)
        self.busy = False       # one gripper action at a time

        self.create_subscription(Int8MultiArray, self.button_topic, self.button_callback, 10)
        self.get_logger().info(
            f'gripper_teleop: button[{self.open_button}]=open -> {gripper_ns}/move, '
            f'button[{self.close_button}]=close -> {gripper_ns}/grasp; '
            f'command -> {self.cmd_topic} (open={self.open_value}, close={self.close_value})')

    def _publish_cmd(self):
        """Continuously publish the latched gripper command so the recorder always has it."""
        self.cmd_pub.publish(Float64(data=float(self._gripper_cmd)))

    def _rising_edge(self, buttons, idx):
        """True only on the 0->1 transition of button idx."""
        if idx >= len(buttons):
            return False
        cur = int(buttons[idx])
        prev = self.prev.get(idx, 0)
        self.prev[idx] = cur
        return prev == 0 and cur == 1

    def button_callback(self, msg):
        buttons = list(msg.data)
        open_pressed = self._rising_edge(buttons, self.open_button)
        close_pressed = self._rising_edge(buttons, self.close_button)

        if self.busy or not (open_pressed or close_pressed):
            return
        if open_pressed:
            self.send_open()
        elif close_pressed:
            self.send_close()

    def send_open(self):
        if not self.move_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('move action server unavailable')
            return
        goal = Move.Goal()
        goal.width = self.open_width
        goal.speed = self.open_speed
        self._gripper_cmd = self.open_value      # latch commanded state for recording
        self.get_logger().info(f'OPEN -> width={self.open_width:.3f} speed={self.open_speed:.3f}')
        self._send(self.move_client, goal)

    def send_close(self):
        if not self.grasp_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('grasp action server unavailable')
            return
        goal = Grasp.Goal()
        goal.width = self.grasp_width
        goal.speed = self.grasp_speed
        goal.force = self.grasp_force
        goal.epsilon.inner = self.grasp_eps_inner
        goal.epsilon.outer = self.grasp_eps_outer
        self._gripper_cmd = self.close_value     # latch commanded state for recording
        self.get_logger().info(f'CLOSE -> width={self.grasp_width:.3f} force={self.grasp_force:.1f}')
        self._send(self.grasp_client, goal)

    def _send(self, client, goal):
        self.busy = True
        future = client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('gripper goal rejected')
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        res = future.result().result
        ok = getattr(res, 'success', True)
        if not ok:
            self.get_logger().warn(f'gripper action failed: {getattr(res, "error", "")}')
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = GripperTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
