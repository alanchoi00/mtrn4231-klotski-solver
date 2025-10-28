# GripperActionServer Testing Guide

## Code Validation Summary

### ✅ **Issues Fixed:**

1. **Missing imports** - Added `time`, proper action imports
2. **No feedback publishing** - Added progress feedback during execution
3. **No error handling** - Added serial communication error handling
4. **No result success status** - Now properly sets `result.success`
5. **Missing cleanup** - Added proper serial port closure
6. **Action server topic** - Fixed to match brain expectation: `/gripper_manipulation/grip_piece`
7. **Progress indication** - Added detailed feedback with progress bar

### ✅ **Improvements Made:**

- **Simulation mode**: Works without hardware (when serial port unavailable)
- **Goal/cancel callbacks**: Proper action server lifecycle management
- **Detailed logging**: Comprehensive debug and info messages
- **Robust error handling**: Graceful failure handling
- **Progress feedback**: Real-time execution progress updates

## Testing Methods

### 1. **Build the Package**

```bash
cd /home/mtrn/4231/mtrn4231-klotski-solver/src
colcon build --packages-select pkg_manipulation
source install/setup.bash
```

### 2. **Basic Functionality Test**

#### Start the gripper action server:
```bash
ros2 run pkg_manipulation gripper_action_server
```

#### In another terminal, test with manual control:
```bash
# Test gripper open
ros2 run pkg_manipulation manual_gripper_control open

# Test gripper close  
ros2 run pkg_manipulation manual_gripper_control close
```

### 3. **Automated Testing**

#### Run comprehensive test client:
```bash
# Terminal 1: Start server
ros2 run pkg_manipulation gripper_action_server

# Terminal 2: Run automated tests
ros2 run pkg_manipulation test_gripper_client
```

### 4. **Launch File Testing**

#### Start server with launch file:
```bash
ros2 launch pkg_manipulation test_gripper.launch.py
```

#### Start server with automatic testing:
```bash
ros2 launch pkg_manipulation test_gripper.launch.py test_mode:=true
```

### 5. **ROS2 CLI Testing**

#### Check action server availability:
```bash
ros2 action list
ros2 action info /gripper_manipulation/grip_piece
```

#### Send action goals directly:
```bash
# Open gripper
ros2 action send_goal /gripper_manipulation/grip_piece klotski_interfaces/action/GripPiece "{grip_action: 0}"

# Close gripper
ros2 action send_goal /gripper_manipulation/grip_piece klotski_interfaces/action/GripPiece "{grip_action: 1}"
```

### 6. **Integration Testing with TaskBrain**

#### Test with actual brain system:
```bash
# Terminal 1: Start gripper server
ros2 run pkg_manipulation gripper_action_server

# Terminal 2: Start task brain (if available)
ros2 run pkg_brain task_brain

# Terminal 3: Send UI commands to trigger manipulation
ros2 topic pub /ui/cmd klotski_interfaces/msg/UICommand "{mode: 2}"  # STEP mode
```

## Expected Outputs

### ✅ **Success Indicators:**

1. **Server startup:**
   ```
   [INFO] [gripper_action_server]: GripperActionServer initialized
   [INFO] [gripper_action_server]: Serial port /dev/ttyACM0 opened successfully
   ```
   OR (simulation mode):
   ```
   [WARN] [gripper_action_server]: Failed to open serial port: [Errno 2] No such file or directory: '/dev/ttyACM0'
   [WARN] [gripper_action_server]: Gripper will operate in simulation mode
   ```

2. **Goal execution:**
   ```
   [INFO] [gripper_action_server]: Received goal request
   [INFO] [gripper_action_server]: Executing gripper OPEN (angle=30)
   [INFO] [gripper_action_server]: Gripper OPEN completed successfully
   ```

3. **Client feedback:**
   ```
   Progress: [████████████████████] 100.0%
   ✅ Gripper open completed successfully!
   ```

### ❌ **Error Indicators to Watch For:**

1. **Serial port issues:**
   ```
   [ERROR] [gripper_action_server]: Serial communication error: ...
   ```

2. **Action server not found:**
   ```
   [ERROR] [test_client]: Action server not available!
   ```

3. **Invalid actions:**
   ```
   [ERROR] [gripper_action_server]: Invalid grip action: 99
   ```

## Hardware Requirements

### **With Physical Gripper:**
- Serial device at `/dev/ttyACM0` (or modify path in code)
- Baud rate: 9600
- Gripper expecting angle commands (0-30 range)

### **Without Hardware (Simulation):**
- No special requirements
- Will run in simulation mode with logged commands

## Troubleshooting

### **Serial Port Issues:**
```bash
# Check available serial ports
ls /dev/ttyACM* /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Test serial communication
echo "15" > /dev/ttyACM0
```

### **Action Server Not Found:**
```bash
# Check if server is running
ros2 node list | grep gripper

# Check action server status
ros2 action list | grep grip
```

### **Build Issues:**
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --packages-select pkg_manipulation --cmake-clean-cache
```

## Integration Points

The GripperActionServer is designed to work with:

1. **TaskBrain**: Receives grip commands during manipulation sequences
2. **ActionExecutor**: Called during GRIP_OPEN and GRIP_CLOSE phases
3. **UI System**: Can be triggered through UI step/auto modes

The action server will receive goals from the brain's ActionExecutor during the 5-phase manipulation sequence.
