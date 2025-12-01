# MTRN4231 Klotski Solver

A ROS2-based robotic system for solving the Klotski sliding puzzle using computer vision, path planning, and robotic manipulation.

## 📋 Overview

Klotski is a classic sliding-block puzzle in which ten blocks of varying sizes must be maneuvered within a confined board. While the traditional objective is to guide the largest block to the bottom, in practice every intermediate transition between board states can be just as demanding as the final solution.

This Klotski solver is designed to support players who want to deepen their understanding of the puzzle. It continuously observes a physical Klotski board, tracks player moves, and updates its internal model to mirror the real-world configuration. When prompted, the solver can compute the shortest sequence of moves needed to reach a specified goal state from the current layout. Whether a player requests a single hint or a complete solution, the system can also physically manipulate the board pieces using its robotic end effector, providing both guidance and automated demonstration.

In summmary this project implements an automated Klotski puzzle solver that:

- **Senses**: Uses computer vision to detect the current puzzle state as well as the board position
- **Plans**: Generates optimal move sequences to reach the goal configuration
- **Acts**: Controls a robotic arm to physically manipulate puzzle pieces on the board
- **Monitors**: Provides a web-based dashboard for real-time control and visualization

## System Architecture

TODO

## Technical Compenents

### Computer Vision
The vision pipeline consists of four major stages:

1. **Aruco Detection**
   - Aruco markers placed at the four corners of the board are identified and matched to known corner IDs.
   - Their pixel positions are stored for later use in frame-to-frame transformations.

2. **Board Isolation**
   - The detected Aruco marker positions are used to compute a homography matrix.
   - This matrix is then applied to warp the camera view and isolate a rectified top-down view of the board.

3. **Grid Colour Detection**
   - A 4×5 grid (matching the Klotski board dimensions) is overlaid on the rectified board image.
   - Colour masks are applied to determine the colour of each grid cell.

4. **Reconstructing the Board State from Grid Colours**
   - The grid-colour map is converted back into piece positions.
   - In Klotski, this mapping is **injective**, meaning each valid grid-colour configuration corresponds to a unique arrangement of pieces.
   - Therefore, once a set of grid colours is known, the corresponding board state is uniquely determined.
   - A sliding-window detection algorithm is used to identify connected shapes and assemble the complete piece-position map.

MUST INCLUDE
photo of the board on the table
photo of the grid colour image

### End Effector
The custom end effector is parallel gripper driven by a single servo motor. It consits of primarily laser-cut acrylic sheet and plywood. Its design is both simple and functional.

<img src="Images/Assembly_2.png" alt="Gripper assembly render" width="300">

#### Assembly Guide
The majority of the end effector is held toghether with metric bolts and nuts. 

### Visualisation

### Closed-Loop Operation
```mermaid
```

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
ros2 launch launch/klotski.launch.py

# Or launch individually:
ros2 run pkg_brain task_brain # Task orchestrator
ros2 launch rosbridge_server # ROS bridge
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
