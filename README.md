# 🚗 Autonomous Line Following & Obstacle Avoidance Robot (ROS2 + Gazebo)

## 📌 1. Introduction

This project focuses on designing and simulating an autonomous robot that can:

- Follow a predefined path (line) using **vision-based detection**
- Detect obstacles using **LiDAR sensor data**
- Switch between behaviors dynamically to ensure safe navigation  

The system is implemented using **ROS 2 (Robot Operating System)** and simulated in **Gazebo**, with visualization in **RViz**.

The proposed solution integrates **computer vision** and **sensor-based navigation**, allowing the robot to:
- Follow lines  
- Handle interruptions (line breaks)  
- Avoid obstacles  
- Re-align with the path  

This makes the system robust and practical for real-world applications.

---

## 🎯 2. Objective

### 2.1 Goals
- Develop an autonomous robot capable of accurate **line following**
- Implement real-time **obstacle detection** using LiDAR
- Design a switching control mechanism between:
  - Line following mode  
  - Obstacle avoidance mode  
- Ensure the robot can recover from **line loss (line breaks)**

---

### 2.2 Expected Outcomes
- Accurate detection and tracking of line using camera input  
- Reliable obstacle detection and avoidance  
- Smooth transition between navigation modes  
- Successful simulation in Gazebo with visualization in RViz  

---

## ⚙️ 3. Methodology

### 🔄 System Overview
The system is divided into multiple ROS nodes working together:

- **Camera Processing Node** → detects the line  
- **LiDAR Processing Node** → detects obstacles  
- **Control Node** → decides robot motion  
- **Switching Logic** → toggles between behaviors  

---
## Architecture

```
Camera (/camera/image_raw)
│
▼
[line_detector] ──► /line_error ──────────────────────┐
▼
LiDAR (/scan)                                   [robot_controller] ──► /cmd_vel
│                                                ▲
▼                                                │
[obstacle_detector] ──► /obstacle_detected ─────────────┘
```

---
## 🚀 Key Highlights
- Real-time vision + LiDAR sensor fusion  
- Finite State Machine (FSM) based navigation  
- Robust handling of line loss and recovery  
- Autonomous obstacle avoidance with rejoining logic  
- Fully simulation-based (ROS 2 + Gazebo)

--- 

## Worlds

| World File | Launch File | Description |
|---|---|---|
| `line_world.world` | `line_follower.launch.py` | Basic straight/curved line track |
| `figure8_world.world` | `figure8.launch.py` | Figure-8 loop track |
| `complex_world.world` | `complex_world.launch.py` | Track with intersections and obstacles |

---

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04 recommended)
- **Gazebo Classic** (comes with ROS 2 Humble desktop install)
- **TurtleBot3 packages:**
```bash
  sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
```
- **Python dependencies:**
```bash
  sudo apt install ros-humble-cv-bridge python3-opencv
```

---

## Installation

```bash
# 1. Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src

# 2. Copy / clone this package
cp -r /path/to/line_follower .

# 3. Build the workspace
cd ~/ros2_ws
colcon build --packages-select line_follower

# 4. Source the workspace
source install/setup.bash
```

---

## Usage

### Launch the basic line-following world

```bash
ros2 launch line_follower line_follower.launch.py
```

### Launch the figure-8 world

```bash
ros2 launch line_follower figure8.launch.py
```

### Launch the complex world (with obstacles)

```bash
ros2 launch line_follower complex_world.launch.py
```

### View the debug camera feed

```bash
ros2 run rqt_image_view rqt_image_view /line_debug_image
```

### Monitor the line error

```bash
ros2 topic echo /line_error
```

### Monitor obstacle detection

```bash
ros2 topic echo /obstacle_detected
```

---

## Configuration

All tuning constants are defined as class-level attributes in each node and can be modified directly:

**`line_detector.py`**
- Threshold value (`80`) — adjust for different lighting conditions.
- ROI crop fraction (`0.6`) — how much of the frame bottom is used.
- Minimum contour area (`150`) — filters out noise.

**`obstacle_detector.py`**
- `OBSTACLE_THRESHOLD = 0.5` m — stop/avoid distance.
- `FRONTAL_ARC_DEG = 30` — half-angle of the detection cone.

**`robot_controller.py`**
- Speed and gain constants at the top of the `RobotController` class.
- State durations (`COAST_DURATION`, `DODGE_DURATION`, `PASS_DURATION`).

---

## Package Structure
```
line_follower/
├── line_follower/
│   ├── __init__.py
│   ├── line_detector.py       # Camera-based line detection node
│   ├── obstacle_detector.py   # LiDAR-based obstacle detection node
│   └── robot_controller.py    # State-machine controller node
├── launch/
│   ├── line_follower.launch.py   # Basic line world
│   ├── figure8.launch.py         # Figure-8 world
│   └── complex_world.launch.py   # Complex world with obstacles
├── worlds/
│   ├── line_world.world
│   ├── figure8_world.world
│   ├── complex_world.world
│   └── ...
├── resource/
│   └── line_follower
├── package.xml
└── setup.py
```
---
