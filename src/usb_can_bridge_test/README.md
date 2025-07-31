# USB-CAN-A Bridge Test Package

This package provides a comprehensive test suite for validating the USB-CAN-A bridge concept with ros2_canopen integration using two Waveshare USB-CAN-A adapters.

## Overview

The test demonstrates:
1. **USB-CAN-A to SocketCAN Bridge**: Converting proprietary USB-CAN protocol to standard SocketCAN
2. **Virtual Motor Simulation**: Using second adapter to simulate a CANopen motor
3. **Turtle Visualization**: Real-time visualization of motor feedback using turtlesim
4. **ros2_canopen Integration**: Testing compatibility with standard CANopen stack

## Hardware Requirements

- 2x Waveshare USB-CAN-A adapters
- USB cables
- Computer running Ubuntu with ROS2 Humble

## Test Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ USB-CAN Bridge  │    │ Virtual CAN     │    │ ros2_canopen    │
│ (ttyUSB0)       │◄──►│ (vcan0)         │◄──►│ CANopen Master  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Virtual Motor   │◄──►│ CANopen Messages│    │ Turtle Sim      │
│ Simulator       │    │ (RPDO/TPDO)     │    │ Visualization   │
│ (ttyUSB1)       │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Installation & Setup

### 1. Build the Package

```bash
cd /home/ros/ros_ws
colcon build --packages-select usb_can_bridge_test
source install/setup.bash
```

### 2. Check USB-CAN Adapters

```bash
# List USB devices to verify adapters are connected
lsusb | grep QinHeng

# Check device nodes
ls -la /dev/ttyUSB*

# Expected output:
# /dev/ttyUSB0 (first adapter)
# /dev/ttyUSB1 (second adapter)
```

### 3. Set Permissions (if needed)

```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
# or add user to dialout group:
sudo usermod -a -G dialout $USER
# (requires logout/login)
```

## Test Procedures

### Test 1: Simple CAN Communication

Test basic USB-CAN communication between adapters:

```bash
# Terminal 1: Launch simple test
ros2 launch usb_can_bridge_test simple_test.launch.py

# This will:
# - Start CAN frame publisher on /dev/ttyUSB0
# - Start CAN frame subscriber on /dev/ttyUSB1  
# - Send sine wave velocity commands
# - Display received frames with CANopen interpretation
```

Expected output:
```
[can_publisher]: Sent velocity command: 1.234 rad/s
[can_subscriber]: RX: ID=0x202, DLC=6, Data=[15 0 210 4 0 0] | RPDO1 to node 2, Control: 0x000F, Target: 1.234 rad/s
```

### Test 2: Complete System with Turtle Visualization

Test full bridge with ros2_canopen and turtle simulation:

```bash
# Terminal 1: Launch complete system
ros2 launch usb_can_bridge_test complete_test.launch.py

# Terminal 2: Monitor topics
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /bridge_status

# Terminal 3: Send manual commands (optional)
ros2 topic pub /target_velocity std_msgs/msg/Float64 "data: 2.0"
```

This will start:
1. SocketCAN bridge (ttyUSB0 → vcan0)
2. Virtual motor simulator (ttyUSB1)
3. Turtlesim visualization
4. Turtle bridge (motor feedback → turtle movement)
5. ros2_canopen device manager

### Test 3: Automated Test Sequence

Run automated test with predefined velocity profiles:

```bash
# Terminal 1: Start complete system
ros2 launch usb_can_bridge_test complete_test.launch.py

# Terminal 2: Run automated test
python3 /home/ros/ros_ws/src/usb_can_bridge_test/scripts/test_usb_can_bridge.py
```

## Monitoring & Debugging

### Check Virtual CAN Interface

```bash
# Verify vcan0 is created and active
ip link show vcan0

# Monitor CAN traffic on virtual interface
candump vcan0

# Send test frame to virtual interface
cansend vcan0 202#0F00D204000000
```

### Monitor ROS2 Topics

```bash
# Bridge status
ros2 topic echo /bridge_status

# Motor status  
ros2 topic echo /motor_status

# Joint states (motor feedback)
ros2 topic echo /joint_states

# CAN frame status
ros2 topic echo /can_tx_status
ros2 topic echo /can_rx_status
```

### Debug CAN Communication

```bash
# Enable debug output
ros2 param set /usb_can_bridge enable_debug true
ros2 param set /virtual_motor_simulator enable_debug true

# Check adapter connectivity
ros2 run usb_can_bridge_test can_frame_publisher --ros-args -p device_path:=/dev/ttyUSB0
ros2 run usb_can_bridge_test can_frame_subscriber --ros-args -p device_path:=/dev/ttyUSB1
```

## Expected Results

### Successful Test Indicators

1. **USB-CAN Communication**: 
   - Frames sent on ttyUSB0 appear on ttyUSB1
   - CANopen message interpretation shows correct control/status

2. **Bridge Functionality**:
   - Virtual CAN interface (vcan0) is active
   - candump shows translated CAN frames
   - No frame loss or corruption

3. **Motor Simulation**:
   - Joint states published with realistic position/velocity
   - Status word shows proper CANopen state transitions
   - Target velocity tracking with reasonable response

4. **Turtle Visualization**:
   - Turtle moves proportionally to motor velocity
   - Smooth movement without jitter
   - Trace shows motor velocity profile

### Troubleshooting

#### Common Issues

1. **Permission Denied on /dev/ttyUSB***
   ```bash
   sudo chmod 666 /dev/ttyUSB*
   ```

2. **Virtual CAN Interface Failed**
   ```bash
   sudo modprobe vcan
   sudo ip link add dev vcan0 type vcan
   sudo ip link set up vcan0
   ```

3. **No CAN Frames Received**
   - Check adapter connections
   - Verify CAN speeds match (500k default)
   - Test with original canusb tool

4. **Turtle Not Moving**
   - Check joint_states topic
   - Verify turtle_bridge is running
   - Check turtlesim window is visible

#### Hardware Verification

Test adapters individually with original canusb tool:

```bash
# Terminal 1: Receiver
cd /home/ros/ros_ws/USB-CAN-A
./canusb -d /dev/ttyUSB1 -s 500000 -t

# Terminal 2: Sender  
cd /home/ros/ros_ws/USB-CAN-A
./canusb -d /dev/ttyUSB0 -s 500000 -t -i 123 -j DEADBEEF
```

## Next Steps

If tests are successful:

1. **Integrate with Real Traction Driver**:
   - Use your TRACTION_PWR.eds file
   - Configure proper CANopen parameters
   - Test with actual hardware

2. **Add ros2_control Support**:
   - Implement hardware interface
   - Create controller configuration
   - Enable position/velocity control modes

3. **Performance Optimization**:
   - Tune communication rates
   - Optimize frame processing
   - Add error recovery mechanisms

## Configuration Files

- `config/test_bus_config.yml`: CANopen bus configuration
- `launch/complete_test.launch.py`: Full system launch
- `launch/simple_test.launch.py`: Basic communication test

## Validation Criteria

✅ **Pass Criteria**:
- CAN frames transmitted between adapters
- Virtual CAN bridge operational  
- Motor simulation responds to commands
- Turtle visualization tracks motor movement
- No frame corruption or loss

❌ **Fail Criteria**:
- Communication timeouts
- Frame corruption
- Bridge crashes
- Virtual CAN setup failures

This test package provides a solid foundation for validating the USB-CAN-A bridge concept before implementing the full traction driver integration.
