# spacemouse_teleop

Robot-agnostic SpaceMouse teleoperation for ROS 2. Two nodes, fully configured by YAML:

- **`spacemouse_driver`** — reads the SpaceMouse HID, auto-calibrates the resting
  bias, publishes a calibrated 6-DOF `Twist` (`raw_topic`) + buttons.
- **`teleop_interface`** — maps the SpaceMouse to a compliance-reference command, in
  one of two modes (`output_mode`):
  - **twist**: publish a 6-DOF velocity `Twist` (`~/cmd_twist`); the CONTROLLER owns and
    integrates the equilibrium (Franka `admittance_node`). Force limiting in controller.
  - **pose**: integrate the equilibrium client-side (initialized once from the measured
    pose) and publish an absolute `PoseStamped`; for controllers taking an absolute
    equilibrium pose (KUKA/FRI). Optional force limiting blocks motion into contact.

The reference (equilibrium) is what you command — never the measured pose — so a hand
push does not drag it. Everything robot-specific (mode, topics, frame, scales, axis
signs, force limit) lives in `config/<robot>.yaml`, so the same code drives both.

## Dependencies
```bash
pip install pyspacemouse        # also needs system hidapi (libhidapi-hidraw0)
# rclpy, numpy, scipy, geometry_msgs, std_msgs, sensor_msgs
```

## Build
```bash
colcon build --packages-select spacemouse_teleop
source install/setup.bash
```

## Run
```bash
# Franka built-in Cartesian impedance (default) — pose mode -> cartesian_impedance_node:
ros2 launch spacemouse_teleop teleop.launch.py robot:=franka_impedance

# Franka admittance node — twist mode -> admittance_node:
ros2 launch spacemouse_teleop teleop.launch.py robot:=franka_admittance

# KUKA (original setup):
ros2 launch spacemouse_teleop teleop.launch.py robot:=kuka

# or a custom config file:
ros2 launch spacemouse_teleop teleop.launch.py config:=/path/to/my.yaml
```

For Franka, run the controller node first
(`ros2 launch franka_cartesian_impedance_node admittance.launch.py`); the teleop
interface then publishes to `/admittance_node/target_pose` and reads
`/admittance_node/current_pose` + `/admittance_node/ext_wrench`.

## Config keys (`teleop_interface`)
| key | meaning |
|-----|---------|
| `pose_topic` / `cmd_topic` | measured pose in / target pose out |
| `force_topic` | external wrench in (for force limiting) |
| `frame_id` | frame stamped on the command |
| `lin_scale` / `ang_scale` | SpaceMouse → m,rad per step |
| `target_hz` | control/publish rate |
| `alpha_pos` / `alpha_quat` | measured-pose smoothing (0..1) |
| `sm_axis_sign` | per-axis sign/mapping `[lx,ly,lz,ax,ay,az]` |
| `force_limit` / `enable_force_limit` | block motion into contact above N |
