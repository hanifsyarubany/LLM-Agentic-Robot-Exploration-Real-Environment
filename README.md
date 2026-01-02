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

## Repository Notes (Image paths with spaces)

If you reference files with spaces in GitHub Markdown (e.g., `figures/simulated map.png`), you have two clean options:

1) **Rename files** to avoid spaces (recommended):  
`simulated map.png` → `simulated_map.png`

2) **URL-encode spaces** as `%20` in Markdown/HTML:  
`figures/simulated%20map.png`

Example:
```md
![map](figures/simulated%20map.png)
