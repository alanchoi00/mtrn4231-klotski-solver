# MTRN4231 Klotski Solver

A ROS2-based robotic system for solving the Klotski sliding puzzle using computer vision, path planning, and robotic manipulation with a UR5e robot arm.

## 📋 Overview

Klotski is a classic sliding-block puzzle in which blocks of varying sizes (1×1, 1×2, 2×1, and 2×2) must be maneuvered within a 4×5 confined board. The traditional objective is to guide the largest 2×2 block to the exit at the bottom.

This project implements an automated Klotski puzzle solver that:

- **Senses**: Uses computer vision to detect the current puzzle state and board position via ArUco markers
- **Plans**: Generates optimal move sequences using BFS-based path planning to reach the goal configuration
- **Acts**: Controls a UR5e robotic arm with a custom gripper to physically manipulate puzzle pieces
- **Monitors**: Provides a web-based dashboard for real-time control, visualization, and safety monitoring

The system continuously observes a physical Klotski board, tracks the board state, and when prompted, computes the shortest sequence of moves needed to reach a specified goal state. It can provide single-step hints or execute complete solutions autonomously.

## 🏗️ System Architecture

TODO: Insert architecture diagram here

## 📦 Package Structure

### `klotski_interfaces/`

Custom ROS2 interface definitions for inter-node communication.

**Messages:**

| Message | Description |
|---------|-------------|
| `Board.msg` | Board state with piece positions |
| `BoardSpec.msg` | Board dimensions (4×5 grid) |
| `BoardState.msg` | Complete board state with pose |
| `Cell.msg` | Grid cell coordinates (col, row) |
| `Piece.msg` | Piece definition (type, color, cells) |
| `Move.msg` | Single move (piece + target cell) |
| `MoveList.msg` | Sequence of moves |
| `HSVRange.msg` | HSV color range for detection |
| `HSVRanges.msg` | Multiple color ranges |
| `GripperCommand.msg` | Gripper open/close commands |
| `ContactState.msg` | Gripper contact feedback |
| `UICommand.msg` | Dashboard commands |

**Services:**

| Service | Description |
|---------|-------------|
| `SolveBoard.srv` | Request optimal path from current to goal state |
| `CaptureBoard.srv` | Capture and return current board state |
| `GetHSVRanges.srv` | Get current color detection ranges |
| `SetHSVRanges.srv` | Update color detection ranges |
| `ResetHSVRanges.srv` | Reset to default HSV ranges |
| `ExportHSVRangesYaml.srv` | Export HSV config to YAML |
| `GetSafetyZone.srv` | Get safety monitoring ROI |
| `SetSafetyZone.srv` | Set safety monitoring ROI |

**Actions:**

| Action | Description |
|--------|-------------|
| `MovePiece.action` | Execute 5-phase manipulation sequence |
| `GripPiece.action` | Open/close gripper |

---

### `klotski_utils/`

Shared Python utilities across packages.

- `params.py` - Type-safe ROS2 parameter declaration helper (`declare_param[T]()`)

---

### `pkg_brain/`

**Task Orchestrator** - Central coordination node managing the complete solving pipeline.

**Node:** `task_brain`

**Responsibilities:**

- UI mode management (auto/step/pause/reset)
- Pipeline orchestration through handler chain
- 5-phase manipulation execution (IDLE → APPROACH → PICK_PLACE → RETREAT)
- Safety stop integration

**Subscriptions:**

| Topic | Type | Description |
|-------|------|-------------|
| `/board_state` | `BoardState` | Current sensed board state |
| `/ui/cmd` | `UICommand` | Dashboard commands |
| `/ui/goal` | `BoardState` | User-defined goal state |
| `/safety/stop` | `Bool` | Emergency stop signal |

**Publications:**

| Topic | Type | Description |
|-------|------|-------------|
| `/ui/events` | `String` | Status updates for dashboard |

**Service Clients:**

| Service | Type | Description |
|---------|------|-------------|
| `/plan/solve` | `SolveBoard` | Request path planning |

**Action Clients:**

| Action | Type | Description |
|--------|------|-------------|
| `/arm_manipulation/move_piece` | `MovePiece` | Arm manipulation |
| `/gripper_manipulation/grip_piece` | `GripPiece` | Gripper control |

---

### `pkg_sense/`

**Computer Vision Module** - Board state detection via camera and ArUco markers.

**Nodes:**

| Node | Description |
|------|-------------|
| `sense` | Main sensing node - ArUco detection, board isolation, color detection |
| `hand_safety_monitor` | Hand detection for safety stopping using MediaPipe |

**Vision Pipeline:**

1. **ArUco Detection** - Detect 4 corner markers (IDs 0-3, DICT_4X4_50, 65mm)
2. **Board Isolation** - Homography transform for top-down rectified view
3. **Color Detection** - HSV masking for piece identification per grid cell
4. **Board Reconstruction** - Sliding-window algorithm to identify pieces from colors

**Subscriptions:**

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/camera/color/image_raw` | `Image` | Camera feed |
| `/camera/camera/color/camera_info` | `CameraInfo` | Camera intrinsics |

**Publications:**

| Topic | Type | Description |
|-------|------|-------------|
| `/board_state` | `BoardState` | Detected board state with pose |
| `/safety/stop` | `Bool` | Hand detection safety signal |
| `/safety/hand_detection_image` | `Image` | Annotated hand detection feed |

**TF Broadcasts:**

| Frame | Parent | Description |
|-------|--------|-------------|
| `aruco_X` | `camera_color_optical_frame` | Individual marker poses |
| `board` | `base_link` | Board origin (bottom-left corner) |

**Services:**

| Service | Type | Description |
|---------|------|-------------|
| `/sense/capture_board` | `CaptureBoard` | Capture current state |
| `/sense/get_hsv_ranges` | `GetHSVRanges` | Get color ranges |
| `/sense/set_hsv_ranges` | `SetHSVRanges` | Set color ranges |
| `/safety/get_zone` | `GetSafetyZone` | Get safety ROI |
| `/safety/set_zone` | `SetSafetyZone` | Set safety ROI |

---

### `pkg_plan/`

**Path Planning Module** - BFS-based Klotski solver with precomputed move database.

**Node:** `solve_service_node` (C++)

**Algorithm:**

- Uses precomputed adjacency graph (`possible_combinations.json`) of all valid Klotski states
- BFS search from current state to goal state
- Returns optimal move sequence

**Services:**

| Service | Type | Description |
|---------|------|-------------|
| `/plan/solve` | `SolveBoard` | Compute optimal path |

---

### `pkg_manipulation/`

**Robot Control Module** - UR5e arm and gripper control using MoveIt2.

**Nodes:**

| Node | Description |
|------|-------------|
| `arm_manipulator` | MoveIt-based arm motion planning and execution (C++) |
| `gripper_manipulator` | Arduino-controlled servo gripper via serial (Python) |

**Arm Manipulator Features:**

- Cartesian path planning for predictable linear motions
- Dynamic board pose subscription for accurate targeting
- Collision objects (table, walls, ceiling)
- Safety stop integration with immediate motion halt
- Configurable velocity/acceleration scaling

**Action Servers:**

| Action | Type | Description |
|--------|------|-------------|
| `/arm_manipulation/move_piece` | `MovePiece` | 5-phase manipulation |
| `/gripper_manipulation/grip_piece` | `GripPiece` | Gripper control |

**5-Phase Manipulation Sequence:**

1. **IDLE** - Ready state
2. **APPROACH** - Move above piece center
3. **PICK** - Descend to grip height, close gripper
4. **PLACE** - Move to target, open gripper
5. **RETREAT** - Rise to safe height

---

### `gripper_description/`

URDF description for the custom gripper end effector.

---

### `ur_with_gripper_description/`

Combined URDF for UR5e robot with custom gripper attached.

---

### `dashboard_app/`

**Web Interface** - Next.js application for real-time control and visualization.

**Technology Stack:**

- Next.js 14 + React 18
- TypeScript
- Tailwind CSS + shadcn/ui components
- roslib.js for ROS2 WebSocket communication

**Features:**

- Real-time board state visualization
- Interactive goal state editor
- Mode control (auto/step/pause/reset)
- HSV color range tuning
- Camera feed with safety zone overlay
- Hand detection monitor with ROI editing

**Key Components:**

| Component | Description |
|-----------|-------------|
| `ControlPanel` | Mode buttons and system control |
| `GoalEditor` | Interactive goal state configuration |
| `ColourMasker` | HSV range adjustment with live preview |
| `HandDetectionViewer` | Safety zone visualization and editing |

---

### `src/launch/`

Main launch configuration for the complete system.

**`klotski.launch.py`** - Launches all nodes:

- rosbridge_server (WebSocket)
- web_video_server (camera streaming)
- ur_moveit (robot driver + MoveIt)
- pkg_manipulation (arm + gripper)
- pkg_plan (solver)
- pkg_sense (vision + safety)
- pkg_brain (orchestrator)

## 🔧 Technical Components

### Computer Vision Pipeline

The vision pipeline consists of four major stages:

1. **ArUco Detection**
   - ArUco markers (DICT_4X4_50, 65mm) placed at the four corners of the board are identified
   - Marker IDs: TL=0, TR=1, BL=2, BR=3
   - Their pixel and 3D positions are stored for transformations

2. **Board Isolation**
   - The detected ArUco marker positions compute a homography matrix
   - Applied to warp the camera view into a rectified top-down view of the board

3. **Grid Color Detection**
   - A 4×5 grid is overlaid on the rectified board image
   - HSV color masks identify the color of each grid cell
   - Configurable via dashboard color masker tool

4. **Board State Reconstruction**
   - Grid colors are converted to piece positions
   - In Klotski, this mapping is **injective** - each color configuration uniquely determines piece arrangement
   - Sliding-window detection identifies connected shapes

### End Effector

The custom end effector is a parallel gripper driven by a single servo motor. It consists primarily of laser-cut acrylic sheet and plywood.

<img src="Images/Assembly_2.png" alt="Gripper assembly render" width="300">

- **Control**: Arduino-based serial communication
- **Actuation**: Single servo motor for parallel jaw movement
- **Feedback**: Contact state detection

### Safety System

Hand detection-based safety monitoring:

- MediaPipe Hands for real-time hand detection
- Configurable safety zone (ROI polygon)
- Immediate arm stop when hands detected in zone
- Automatic resume after hands clear for configurable frames

## 🚀 Quick Start

### Prerequisites

#### System Requirements

- **[Ubuntu 22.04](https://releases.ubuntu.com/jammy/)**
- **[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)**
- **[Node.js 18+](https://nodejs.org/en/download)** and npm
- **[Python 3.10+](https://www.python.org/downloads/)**
- **[OpenCV](https://opencv.org/)** with ArUco support
- **[MoveIt2](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)**

#### Required ROS2 Packages

```bash
# Core packages
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs

# TF2 for coordinate transforms
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# ROS Bridge for web interface
sudo apt install ros-humble-rosbridge-server ros-humble-web-video-server

# Camera support
sudo apt install ros-humble-realsense2-camera

# UR5e Robot Driver and MoveIt
sudo apt install ros-humble-ur-robot-driver ros-humble-ur-moveit-config

# MoveIt2
sudo apt install ros-humble-moveit

# CV Bridge
sudo apt install ros-humble-cv-bridge
```

#### Python Dependencies

```bash
pip install opencv-python mediapipe pyserial scipy
```

### 1. Clone and Build ROS Workspace

```bash
cd ~/mtrn4231-klotski-solver
colcon build
source install/setup.bash
```

### 2. Install Dashboard Dependencies

```bash
cd src/dashboard_app
npm install
```

### 3. Launch the System

#### Option A: Using Launch Scripts (Recommended)

**For Real Robot:**

```bash
# Default camera transform
./runKlotski.sh

# With custom camera transform (x y z qx qy qz qw)
./runKlotski.sh 1.31 0.02 0.67 -0.40 0.00 0.92 0.01
```

**For Simulation:**

```bash
./runKlotskiSim.sh
```

#### Option B: Manual Launch

**Terminal 1: UR5e Driver (real robot)**

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 launch_rviz:=false
```

**Terminal 2: Camera Transform**

```bash
ros2 run tf2_ros static_transform_publisher 1.31 0.02 0.67 -0.40 0.00 0.92 0.01 base_link camera_link
```

**Terminal 3: Main System**

```bash
source install/setup.bash
ros2 launch src/launch/klotski.launch.py
```

**Terminal 4: Dashboard**

```bash
cd src/dashboard_app
npm run dev
```

### 4. Access Dashboard

Open <http://localhost:3000> in your browser.

## 🔧 Development

### Building Individual Packages

```bash
# Build specific packages
colcon build --packages-select pkg_brain pkg_sense

# Build with debug info
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
rm -rf build/ install/ log/
colcon build
```

### ROS2 Commands

```bash
# Check running nodes
ros2 node list

# Monitor topics
ros2 topic echo /board_state
ros2 topic hz /safety/stop

# Test services
ros2 service call /plan/solve klotski_interfaces/srv/SolveBoard "{...}"

# Test actions
ros2 action send_goal /arm_manipulation/move_piece klotski_interfaces/action/MovePiece "{...}"
```

### Camera Calibration

Use the calibration script to determine camera-to-robot transform:

```bash
python3 src/pkg_sense/scripts/calibrate_camera_tf.py --mode preview
python3 src/pkg_sense/scripts/calibrate_camera_tf.py --mode collect --samples 20
python3 src/pkg_sense/scripts/calibrate_camera_tf.py --mode compute
```

See `docs/CAMERA_CALIBRATION.md` for detailed instructions.

## 📄 License

This project is developed for MTRN4231 coursework at UNSW.
