# Camera Calibration Guide

This guide explains how to calibrate the `base_link` → `camera_link` transform for accurate robot-camera coordination.

## Prerequisites

- UR5e robot driver running
- RealSense camera connected and publishing
- ArUco markers visible on the Klotski board (IDs 0-3 at corners)

## Quick Start

```bash
# 1. Start robot driver
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false

# 2. Start camera
ros2 launch realsense2_camera rs_launch.py

# 3. Run calibration
python3 src/pkg_sense/scripts/calibrate_camera_tf.py --mode collect --num-samples 5 --show-preview
```

## Calibration Workflow

### Step 1: Verify Markers
```bash
python3 src/pkg_sense/scripts/calibrate_camera_tf.py --mode check-markers
```
Ensure all 4 markers are detected. If not, adjust lighting or camera position.

### Step 2: Collect Samples
```bash
python3 src/pkg_sense/scripts/calibrate_camera_tf.py --mode collect --num-samples 5 --marker-id 2
```

For each sample:
1. Move the robot gripper **directly above the center of marker 2** (bottom-left)
2. The gripper tip should be at the same height as during normal operation
3. Press **Enter** to record the sample
4. Move to a slightly different position and repeat

> **Tip**: Collect samples at different heights and XY positions above the marker for better accuracy.

### Step 3: Compute Transform
```bash
python3 src/pkg_sense/scripts/calibrate_camera_tf.py --mode compute
```

This outputs the optimal camera transform values.

### Step 4: Apply Calibration
Use the computed values with `runKlotski.sh`:
```bash
./runKlotski.sh <tx> <ty> <tz> <qx> <qy> <qz> <qw>
```

Or update the defaults in `runKlotski.sh` directly.

## Available Modes

| Mode | Command | Description |
|------|---------|-------------|
| Check Markers | `--mode check-markers` | Verify ArUco markers are visible |
| Preview | `--mode preview` | Live camera view with detection overlay |
| Collect | `--mode collect` | Gather calibration samples |
| Compute | `--mode compute` | Calculate optimal transform |

## Options

| Option | Default | Description |
|--------|---------|-------------|
| `--num-samples` | 5 | Number of samples to collect |
| `--marker-id` | 2 | Target marker ID (2 = bottom-left) |
| `--samples-file` | `calibration_samples.json` | File to save/load samples |
| `--show-preview` | off | Show camera preview during collection |

## Accuracy Guidelines

| Error | Rating | Action |
|-------|--------|--------|
| < 5mm | ✓ Excellent | Ready to use |
| < 10mm | ✓ Good | Acceptable for most tasks |
| < 20mm | ⚠ Acceptable | Consider recalibrating |
| ≥ 20mm | ✗ Poor | Must recalibrate |

## Troubleshooting

**No markers detected:**
- Check lighting (avoid glare/shadows on markers)
- Ensure camera is focused and markers are in frame
- Verify marker size matches config (65mm default)

**High calibration error:**
- Collect more samples (8-10 recommended)
- Ensure gripper is precisely positioned above marker center
- Check that robot driver is providing accurate FK

**Transform seems wrong:**
- Verify `camera_link` frame orientation matches physical camera
- Check that RealSense is publishing to correct topics
