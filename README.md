# hardwarebot

ROS 2 bring‑up for a 2‑DoF manipulator that talks to an Arduino running GRBL via a lightweight serial bridge.  
Jog from RViz2 / MoveIt with **no code edits**. Tested on **Windows 10/11 + WSL2** using a tiny TCP↔COM relay on Windows and a `socat` PTY in WSL.

---

## Actuated joints & GRBL axis mapping

| URDF joint   | Type     | GRBL axis | Frame sent                       |
|--------------|----------|-----------|----------------------------------|
| `Revolute22` | rotary   | **Y**     | `~Y <steps>\n`                   |
| `Revolute27` | rotary   | **X**     | `~X <steps>\n`                   |
| `Slider28`   | linear   | **Z**     | `~Z <steps>\n`                   |
| `Slider30`   | rotary   | **A**     | `~A <steps>\n`                   |

> The hardware plugin multiplies commanded joint values by your calibration factors:
> - **rotary** joints: `steps = position_rad * steps_per_rad`
> - **linear** joints: `steps = position_m * steps_per_mm`  
>   Note that ROS prismatic commands are in **meters**. If your mechanics are “200 steps/mm”, set `steps_per_mm` to `200 * 1000 = 200000` to account for meters.

On activation the plugin also sends an initial feed command: `~F<feed>\n`.

---

## Features

- RViz2/MoveIt jogging of two groups (`bot_grip`, `top_grip`) with `/joint_states` published (open‑loop command echo).
- Minimal **ros2_control** hardware plugin: `hardwarebot_system` (position interfaces).
- Windows↔WSL serial bridging with a small Python relay + `socat` virtual PTY.
- All routine calibration via **Xacro/YAML** (port, baud, steps/deg/mm, feed).

---

## Repository layout


hardwarebot/

├── CMakeLists.txt

├── config

├── description

├── hardware

├── launch

├── meshes

├── moveit

├── package.xml

└── worlds
---

## Requirements

- **WSL2 (Ubuntu) + ROS 2 Rolling** with:
  - `ros2_control`, `controller_manager`, `joint_trajectory_controller`
  - `robot_state_publisher`, `xacro`
  - MoveIt 2 (for RViz MotionPlanning/Joints panels)
- **Windows host** with Python 3 (`pyserial`) for the TCP↔COM relay
- **WSL**: `socat` (`sudo apt install socat`)

---

## Build (WSL2)

``bash
cd ~/ros            # your colcon workspace
colcon build --packages-select hardwarebot
source install/setup.bash

Quick start
1) Windows: run the serial bridge (COM ↔ TCP)
Create scripts/serial_tcp_bridge.py on Windows side:

You should see:

Opened COM3 @ 115200
Listening on 0.0.0.0:9000 …

2) WSL2: create a PTY and connect to the bridge

mkdir -p ~/serial
socat -d -d pty,raw,echo=0,link=$HOME/serial/ttyARD tcp:127.0.0.1:9000
# Keep this terminal open; you'll see "starting data transfer loop …" when connected.

3) WSL2: launch bring‑up (controllers + MoveIt/RViz)

source ~/ros/install/setup.bash
ros2 launch hardwarebot bringup_real.launch.py start_moveit:=true
When RViz opens, use the MotionPlanning or Joints panel to jog bot_grip and top_grip.

Configuration & calibration

1) Serial / steps / feed parameters (Xacro → plugin)
File: description/ros2_control.xacro

port (e.g., /home/<user>/serial/ttyARD)

baud (e.g., 115200)

cmd_prefix (default ~)

steps_per_rad (rotary joints)

steps_per_mm (linear joints; ROS uses meters → scale accordingly)

default_feed (mm/min)

2) Controllers (ROS 2 & MoveIt)
config/trajectory_controller.yaml — two JointTrajectoryControllers:

bot_grip_controller → joints: Revolute22, Slider30

top_grip_controller → joints: Revolute27, Slider28

moveit/moveit_controllers.yaml — binds MoveIt to those controllers. moveit_controllers_yaml

moveit/kinematics.yaml — KDL solver settings per group. kinematics_yaml

Ensure the joint lists match between the URDF/SRDF, ros2_control interfaces, and controller YAML.

How it works (end‑to‑end)
RViz / MoveIt ─▶ JointTrajectoryController ─▶ controller_manager ─▶ HardwarebotSystem (write)
          ▲                                                           │
          └────────────── /joint_states ◄────────── HardwarebotSystem (read: echo)
WSL PTY: /home/<user>/serial/ttyARD ◄──── socat ◄──── TCP:9000 ◄── Windows bridge ◄── COM3 ◄── Arduino/GRBL


Known limitations
Open‑loop: read() echoes commanded positions to /joint_states; there are no encoders yet.

Custom frames: the plugin currently emits ~X/~Y/~Z/~A frames, not GRBL $J jog or streamed G‑code.

WSL bridge: relies on socat + Python relay, native USB pass‑through (usbipd) is an alternative.

Files you will likely tune first
description/ros2_control.xacro — serial path, baud, steps per unit, feed.

config/trajectory_controller.yaml — which joints each controller exposes.

description/robot.srdf — MoveIt groups (bot_grip: Revolute22, Slider30; top_grip: Revolute27, Slider28).

hardware/hardwarebot_system.cpp — axis mapping, frame format, feed initialization.
