# ps4_franka_control

ROS 2 **ament_python** package that teleoperates a **Franka Panda** end-effector using a **PS4 controller**, interfacing with **MoveIt Servo** for real-time Cartesian motion and using **Franka gripper actions** for reliable open/close control.

This repository implements **CPS – Deliverable Part 1**:

> Implement a ROS 2 node that reads from a joystick/gamepad node and interfaces with MoveIt Servo to remote control the end-effector pose of the Franka arm.

---

## What this package provides

### Nodes (console scripts)

Installed executables (defined in `setup.py`):

- `unified_ps4_franka_control` — **arm + gripper** in one node (recommended)

(Your repo may also include separate arm-only / gripper-only scripts depending on your branch history, but this README documents the unified node shown below.)

---

## Architecture

**PS4 controller → `joy_node` → `unified_ps4_franka_control` → MoveIt Servo → Franka controllers**

Arm (MoveIt Servo)
- Subscribes: `/joy` (`sensor_msgs/msg/Joy`)
- Publishes: `/servo_node/delta_twist_cmds` (`geometry_msgs/msg/TwistStamped`)
- Calls: `/servo_node/switch_command_type` (`moveit_msgs/srv/ServoCommandType`) to set command type **TWIST**

Gripper (Franka actions)
- Action client: `/panda_gripper/move` (`franka_msgs/action/Move`) for **open**
- Action client: `/panda_gripper/grasp` (`franka_msgs/action/Grasp`) for **close**

---

## Requirements

### Software
- ROS 2 (this repo targets **Jazzy**; compatible with Humble if your MoveIt/Franka stack matches)
- MoveIt 2 + MoveIt Servo (`moveit_servo`)
- `joy` package (`joy_node`)
- Franka stack providing:
  - `franka_msgs/action/Move`
  - `franka_msgs/action/Grasp`
- A working Franka + MoveIt setup (real robot or simulation)

### Hardware
- PS4 controller connected via USB or Bluetooth

---

## Installation & build

### 1) Create a workspace and clone

```bash
mkdir -p ~/cps_ws/src
cd ~/cps_ws/src
git clone https://github.com/dianareyes11/ps4_franka_control.git
```

### 2) Install dependencies

```bash
cd ~/cps_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3) Build

```bash
cd ~/cps_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running (typical 4-terminal workflow)

> You must already have a working Franka + MoveIt + Servo setup. This package provides the joystick-to-servo bridge (and gripper actions).

### Terminal A — Start Franka + MoveIt
Launch your lab’s Franka bringup (real or sim) and your MoveIt configuration.

✅ Goal: robot is visible in RViz and MoveIt is running.

### Terminal B — Start MoveIt Servo
Start Servo using your MoveIt configuration (launch varies by lab).

This node will call `/servo_node/switch_command_type`, but Servo still needs to be running. Check:

```bash
ros2 node list | grep servo
ros2 service list | grep switch_command_type
```

### Terminal C — Start the joystick driver

```bash
ros2 run joy joy_node
```

Verify:

```bash
ros2 topic echo /joy
```

### Terminal D — Run the unified controller

```bash
ros2 run ps4_franka_control unified_ps4_franka_control
```

---

## Controls (exact mapping from the node)

### Buttons

- **CIRCLE** (button index **1**): configure MoveIt Servo to accept **TWIST** commands  
  (calls `/servo_node/switch_command_type` once)
- **L1** (button index **4**): toggle **Translation** ↔ **Rotation** mode
- **TRIANGLE** (button index **3**): toggle gripper **OPEN/CLOSE**

> The node uses rising-edge detection (button press events), so holding a button does not continuously toggle.

### Axes → End-effector commands

The node reads these joystick axes (PS4 layout as reported by `joy_node`):

- `axes[0]` = left stick **X**
- `axes[1]` = left stick **Y**
- `axes[3]` = right stick **X**
- `axes[2]` = **L2 trigger**
- `axes[5]` = **R2 trigger**

Triggers are converted from `[-1, 1]` to `[0, 1]` and combined into a single **Z** control:
- `z_control = R2 - L2`

All sticks use a dead-zone filter to prevent drift.

---

### Translation mode (default)

| Control | Command |
|---|---|
| Left stick Y | linear **Y** |
| Left stick X | linear **X** |
| R2 − L2 | linear **Z** |
| Right stick X | angular **Z** (yaw) |

### Rotation mode

| Control | Command |
|---|---|
| Left stick Y | angular **Y** (pitch) |
| Left stick X | angular **X** (roll) *(sign inverted in code)* |
| R2 − L2 | linear **Z** |
| Right stick X | angular **Z** (yaw) |

---

## Parameters / constants used in code

### Servo command frame
The node publishes `TwistStamped` with:

- `header.frame_id = "panda_link0"`

### Scaling and dead-zone
- `linear_scale = 0.1` (m/s)
- `angular_scale = 0.6` (rad/s)
- `dead_zone = 0.15`

### Gripper settings
- Open width: `0.08` m
- Speed: `0.05` m/s
- Force (close): `1.0` N
- Epsilon: `inner = inf`, `outer = inf` (no tight tolerance)

---

## ROS interfaces (exact)

### Subscribes
- `/joy` — `sensor_msgs/msg/Joy`

### Publishes
- `/servo_node/delta_twist_cmds` — `geometry_msgs/msg/TwistStamped`

### Service clients
- `/servo_node/switch_command_type` — `moveit_msgs/srv/ServoCommandType`  
  Requests: `command_type = TWIST`

### Action clients
- `/panda_gripper/move` — `franka_msgs/action/Move` (open)
- `/panda_gripper/grasp` — `franka_msgs/action/Grasp` (close)

---

## Quick debugging checklist

### Joystick input
```bash
ros2 topic echo /joy
```

### Servo command stream
```bash
ros2 topic echo /servo_node/delta_twist_cmds
```

### Servo service present?
```bash
ros2 service list | grep switch_command_type
```

### Gripper action servers present?
```bash
ros2 action list | grep panda_gripper
```

If the gripper servers are not available, the node will print warnings at startup and gripper toggles will fail.

---

## Safety notes

- Motion is **velocity-based** and stops when commands stop.
- MoveIt Servo is expected to enforce collision checking and limits (per your Servo configuration).
- Always keep the physical emergency stop accessible when testing on real hardware.

---


---

## Note on Franka controller parameter (nonzero velocity issue)

During development, an issue was encountered where the robot would reject or abruptly stop Servo commands due to a **nonzero velocity at trajectory end** constraint enforced by the Franka arm controller.

To resolve this, the following ROS 2 parameter must be set on the controller node:

- **Node**: `/panda_arm_controller`  
- **Parameter**: `allow_nonzero_velocity_at_trajectory_end`  
- **Value**: `true`

You can check the current value with:
```bash
ros2 param get /panda_arm_controller allow_nonzero_velocity_at_trajectory_end
```

And set it manually (if needed) with:
```bash
ros2 param set /panda_arm_controller allow_nonzero_velocity_at_trajectory_end true
```

In our setup, this parameter was later **set directly in the launch file**, so it does not need to be manually configured every time.  
However, if Servo motion does not behave as expected, this parameter should be one of the first things to verify.

## Reference

- MoveIt Servo tutorial:
  https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html
