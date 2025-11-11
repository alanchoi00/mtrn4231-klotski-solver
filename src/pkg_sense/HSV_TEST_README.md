# HSV Color Testing Tool for Klotski Sense Node

This tool provides an interactive interface to test and tune HSV color detection parameters for the Klotski board recognition system.

## Features

- **Interactive HSV Sliders**: Real-time adjustment of HSV thresholds for all colors (red, yellow, green, blue, grey)
- **Live Mask Preview**: See color detection results in real-time
- **CSV Data Export**: Save HSV parameters and mask statistics for analysis
- **Board Capture Testing**: Test board recognition with current HSV settings
- **Image Saving**: Save original images and mask results for documentation

## Setup and Installation

1. Make sure you're in the ROS2 workspace:
   ```bash
   cd /home/mtrn/mtrn4231-klotski-solver
   ```

2. Build the package:
   ```bash
   colcon build --packages-select pkg_sense
   source install/setup.bash
   ```

## Usage

### Method 1: Launch with Camera (Complete Setup)
```bash
# Start everything including camera and sense node
ros2 launch pkg_sense test_hsv_sense.launch.py
```

### Method 2: Launch Test Node Only (Manual Setup)
```bash
# Terminal 1: Start camera
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true

# Terminal 2: Start sense node
ros2 run pkg_sense sense

# Terminal 3: Start HSV test tool
ros2 run pkg_sense test_hsv_sense
```

### Method 3: Simple Launch (Camera Already Running)
```bash
# If camera and sense node are already running
ros2 launch pkg_sense test_hsv_simple.launch.py
```

## GUI Controls

### Windows
- **HSV Controls**: Information display and control panel
- **Original Image**: Live camera feed
- **Color Masks**: Color-coded visualization of detected regions
- **Combined Mask**: All color masks combined

### HSV Sliders
For each color (red1, red2, yellow, green, blue, grey), you can adjust:
- **H Low/High**: Hue range (0-179)
- **S Low/High**: Saturation range (0-255)  
- **V Low/High**: Value/Brightness range (0-255)

### Keyboard Controls
- **'s'**: Save current HSV values and mask statistics to CSV
- **'c'**: Capture board using current HSV settings and save results
- **'q' or ESC**: Quit the application

## Output Files

All results are saved to `/tmp/hsv_test_results/`:

### CSV File
- **Format**: `hsv_test_results_YYYYMMDD_HHMMSS.csv`
- **Contents**: 
  - Timestamp and test ID
  - All HSV parameter values
  - Mask area statistics and percentages
  - Board capture success status
  - User notes and lighting conditions

### Images
- **Original**: `original_[test_id]_[timestamp].png`
- **Color Masks**: `mask_[color]_[test_id]_[timestamp].png`

## CSV Data Fields

| Field | Description |
|-------|-------------|
| `timestamp` | ISO format timestamp |
| `test_id` | Sequential test number |
| `lighting_condition` | User-entered lighting description |
| `notes` | User-entered notes |
| `[color]_h/s/v_low/high` | HSV threshold values for each color |
| `[color]_mask_area` | Number of pixels detected for each color |
| `[color]_percentage` | Percentage of image covered by each color |
| `board_capture_success` | Whether board recognition succeeded |
| `piece_count_valid` | Whether piece counts were valid |

## Workflow for Parameter Tuning

1. **Start the tool** with good lighting on the Klotski board
2. **Adjust HSV sliders** while watching the color masks in real-time
3. **Save good configurations** using 's' key with descriptive notes
4. **Test board capture** using 'c' key to verify detection works end-to-end
5. **Try different lighting conditions** and save results for comparison
6. **Analyze CSV data** to find optimal parameters across conditions

## Tips for Good Results

### Lighting
- Test under different lighting conditions (bright, dim, artificial, natural)
- Avoid harsh shadows and reflections
- Ensure even illumination across the board

### HSV Parameter Tuning
- **Hue**: Most important for color distinction
- **Saturation**: Adjust to exclude washed-out colors
- **Value**: Adjust for lighting variations
- **Red Color**: Use both red1 (0-10) and red2 (170-180) ranges for full red spectrum

### Mask Quality
- Look for clean, solid regions in the color masks
- Minimize noise and false positives
- Ensure piece boundaries are well-defined

## Troubleshooting

### Camera Not Working
- Check if RealSense camera is connected
- Verify camera drivers: `ros2 topic list | grep camera`
- Try restarting the camera node

### GUI Not Appearing
- Ensure X11 forwarding if using SSH: `ssh -X`
- Check OpenCV installation: `python3 -c "import cv2; print(cv2.__version__)"`
- Make sure display is available

### No Image Data
- Verify image topic: `ros2 topic echo /camera/camera/color/image_raw --max-count=1`
- Check topic remapping in launch files
- Ensure camera permissions

### Service Call Fails
- Confirm sense node is running: `ros2 service list | grep capture`
- Check for ArUco markers in camera view
- Verify board is properly positioned

## Data Analysis

The CSV output can be analyzed using Python pandas, Excel, or other data analysis tools:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load results
df = pd.read_csv('/tmp/hsv_test_results/hsv_test_results_*.csv')

# Plot color detection percentages
colors = ['red', 'yellow', 'green', 'blue', 'grey']
df[color + '_percentage'].plot(kind='hist', title=f'{color.title()} Detection %')
plt.show()

# Find best parameters for successful captures
successful = df[df['board_capture_success'] == True]
print("Successful capture HSV ranges:")
print(successful[['red1_h_low', 'red1_h_high', 'red1_s_low', 'red1_s_high']].describe())
```

This tool helps systematically find the optimal HSV parameters for reliable Klotski board recognition across different conditions.

ros2 launch pkg_sense test_hsv_sense.launch.py
