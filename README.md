# MTRN4231 Klotski Solver

A ROS2-based robotic system for solving the Klotski sliding puzzle using computer vision, path planning, and robotic manipulation.

## 📋 Overview

This project implements an automated Klotski puzzle solver that:

- **Senses**: Uses computer vision to detect the current puzzle state
- **Plans**: Generates optimal move sequences to reach the goal configuration
- **Acts**: Controls a robotic arm to physically manipulate puzzle pieces
- **Monitors**: Provides a web-based dashboard for real-time control and visualization

## 🚀 Quick Start

### Prerequisites

#### System Requirements

- **ROS2 Humble**
- **Node.js 18+** and npm
- **Python 3.8+**
- **OpenCV** (for computer vision)
- **Camera** (USB webcam or built-in)
- **Robotic Arm** (UR5e)
- **Klotski Puzzle Board and Pieces**

#### Required ROS2 Packages

Install the following ROS2 packages:

```bash
# Core ROS2 packages
sudo apt install ros-humble-rclpy
sudo apt install ros-humble-std-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-action-msgs
sudo apt install ros-humble-builtin-interfaces

# TF2 for coordinate transforms
sudo apt install ros-humble-tf2-ros

# ROS Bridge for web interface
sudo apt install ros-humble-rosbridge-server

# Camera support
sudo apt install ros-humble-v4l2-camera

# UR5e Robot Driver and MoveIt Integration
sudo apt install ros-humble-ur-robot-driver
sudo apt install ros-humble-ur-moveit-config

# Launch system
sudo apt install ros-humble-launch
sudo apt install ros-humble-launch-ros

# Build tools
sudo apt install ros-humble-ament-cmake
```

### 1. Clone and Build ROS Workspace

```bash
cd ~/mtrn4231-klotski-solver
colcon build
source install/setup.bash
```

### 2. Install Dashboard Dependencies

```bash
cd dashboard_app
npm install
```

### 3. Launch the System

#### Terminal 1: ROS Backend

```bash
# Launch core ROS nodes
ros2 launch launch/demo.launch.py

# Or launch individually:
ros2 run pkg_brain task_brain        # Task orchestrator
ros2 launch rosbridge_server rosbridge_websocket_launch.xml  # Web bridge
```

#### Terminal 2: Camera (if using USB camera)

```bash
# Start camera node
./camera.sh
# Or: ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"
```

#### Terminal 3: Dashboard

```bash
cd dashboard_app
npm run dev
```

The dashboard will be available at: <http://localhost:3000>

## 📦 Package Structure

### `klotski_interfaces/`

Custom ROS2 message and service definitions:

- **Messages**: `Board`, `BoardState`, `Piece`, `Cell`, `Move`, etc.
- **Services**: `SolveBoard` for path planning requests
- **Actions**: `MovePiece` for robotic manipulation

### `pkg_brain/`

**Task Orchestrator** - Coordinates the entire solving process:

```python
# Main responsibilities:
- Subscribe: /ui/cmd, /board_state, /ui/goal
- Publish: /ui/events
- Services: /plan/solve
- Actions: /move_piece
```

### `pkg_sense/`

**Computer Vision Module** (To be implemented):

- Camera calibration and image processing
- Puzzle piece detection and tracking
- Board state estimation
- Publishes to `/board_state`

### `pkg_plan/`

**Path Planning Module** (To be implemented):

- Klotski puzzle solving algorithms
- Move sequence optimization
- Collision avoidance
- Serves `/plan/solve` requests

### `pkg_manipulation/`

**Robot Control Module** (To be implemented):

- Robotic arm motion planning
- Grasp planning and execution
- Safety monitoring
- Provides `/move_piece` action server

### `dashboard_app/`

**Web Interface** - Next.js application with:

- Real-time ROS integration via rosbridge
- Interactive puzzle editor
- System monitoring and control
- Built with React + TypeScript + Tailwind CSS

## 🔧 Development Setup

### Building Individual Packages

```bash
# Build specific packages
colcon build --packages-select klotski_interfaces pkg_brain

# Build with debug info
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
rm -rf build/ install/ log/
colcon build
```

### ROS2 Development Commands

```bash
# Check running nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /ui/events
ros2 topic hz /board_state

# Service testing
ros2 service list
ros2 service call /plan/solve klotski_interfaces/srv/SolveBoard {}

# Action testing
ros2 action list
ros2 action send_goal /move_piece klotski_interfaces/action/MovePiece {}
```

### Dashboard Development

```bash
cd dashboard_app

# Development server with hot reload
npm run dev

# Production build
npm run build
npm run start

# Linting
npm run lint
```

## 📄 License

This project is developed for MTRN4231 coursework.
