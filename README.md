# spacemouse_teleop

Robot-agnostic SpaceMouse teleoperation for ROS 2. Two nodes, fully configured by YAML:

- **`spacemouse_driver`** — reads the SpaceMouse HID, auto-calibrates the resting
  bias, publishes a calibrated 6-DOF `Twist` (`raw_topic`) + buttons.
- **`teleop_interface`** — integrates the SpaceMouse deltas onto a smoothed measured
  TCP pose and publishes a target pose command. Optional force limiting blocks motion
  into a contact. (Same control logic validated on the KUKA/FRI.)

Everything robot-specific (topic names, frame, scales, axis signs, force limit) lives
in `config/<robot>.yaml`, so the same code drives the Franka and the KUKA.

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
# Franka (default) — wires to admittance_node topics:
ros2 launch spacemouse_teleop teleop.launch.py robot:=franka

# KUKA (original setup):
ros2 launch spacemouse_teleop teleop.launch.py robot:=kuka

# or a custom config file:
ros2 launch spacemouse_teleop teleop.launch.py config:=/path/to/my.yaml
```

For Franka, run the admittance node first
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
