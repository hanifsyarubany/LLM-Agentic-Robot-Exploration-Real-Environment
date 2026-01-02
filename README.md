# LLM-Agentic Exploration Robot (Real-World) — Navigation + Object Pickup

This repository contains the **real-environment** implementation of our **LLM-based agentic exploration** system for a mobile manipulator robot that navigates a corridor-style “shopping” layout, builds/uses semantic cues (e.g., store icons / signboards / AprilTags), and executes **end-to-end tasks from user instruction to object pickup**.

---

## Real-World Testbed

<p align="center">
  <img src="figures/real_map.jpg" width="900"/>
  <br/>
  <em><b>Figure 1.</b> Real-world corridor testbed used for evaluation, containing store entrances/signboards and junctions for exploration and task execution.</em>
</p>

---

## Hardware Overview (Two-Computer Robot)

Our robot uses **two onboard compute units**:
- **Jetson Nano**: runs perception (YOLO), navigation/decision stack, and publishes motion/action commands.
- **ArmPi Pro**: runs the arm/gripper stack and bridges base/arm control (executes grasp routines and receives velocity commands from Jetson).

<p align="center">
  <img src="figures/robot_camera_functions.png" width="900"/>
  <br/>
  <em><b>Figure 2.</b> Sensor and camera responsibilities. The USB camera supports signboard/store-icon perception and store-entry alignment, while the RealSense RGB-D supports local costmap perception, AprilTag detection, and grasp-target tracking.</em>
</p>

---

## Motion & Behavior Primitives

The system is modular: high-level decisions (LLM policy) trigger low-level ROS “skills” such as AprilTag approaching, local-costmap obstacle avoidance, and store-entry alignment.

<p align="center">
  <img src="figures/specific_strategies.png" width="900"/>
  <br/>
  <em><b>Figure 3.</b> Representative real-world motion strategies used for reliable execution: AprilTag approaching (PID alignment), robot re-positioning, and local costmap-based obstacle avoidance / corridor centering.</em>
</p>

---

## Tech Stack

**Core**
- ROS (catkin workspace)
- Python 3

**Perception**
- Ultralytics **YOLO** (exported to **TensorRT engine** on Jetson)
- Dual-camera setup: **RealSense RGB-D** + **USB RGB**

**Localization / Cues**
- **AprilTag** detection (for anchoring and approach behaviors)

**Control**
- Modular ROS nodes (navigation primitives + grasping routines)
- Two-compute ROS networking (Jetson ↔ ArmPi)

---

## Running the Real Robot System

#### **1) ArmPi Pro Setup (Arm + Bridge)**
###### (a) Build catkin workspace
```bash
cd armpi_pro
rm -rf build devel
catkin_make
source devel/setup.bash
```
###### (b) Start function launcher
```bash
roslaunch armpi_pro_bringup start_functions.launch
```
###### (c) Start the main ArmPi programs
Open a new terminal (still on ArmPi) and run:
```bash
cd armpi_pro/src/armpi_pro_demo

# Receives cmd_vel from Jetson and converts it to the ArmPi velocity interface
python3 cmd_vel_to_velocity.py

# Arm + gripper control (predefined grasp sequences)
python3 kinematics_demo.py
```
Tip: If you prefer, run these in separate terminals so logs are easier to read.
