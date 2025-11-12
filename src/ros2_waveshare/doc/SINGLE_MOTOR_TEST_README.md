# Single Motor Test System

Complete ROS2 system for testing and validating single motor velocity control with closed-loop PID feedback.

## 📋 Overview

The Single Motor Test System provides a comprehensive framework for:
- ✅ Keyboard teleoperation of motor velocity
- ✅ Closed-loop PID velocity control via plugins
- ✅ Encoder-based odometry and TF broadcasting
- ✅ Pluggable dynamic models (single motor, differential drive, Ackermann)
- ✅ Real-time monitoring and diagnostics

## 🏗️ Architecture

```
┌─────────────────┐
│  Keyboard Input │
└────────┬────────┘
         │
         ▼
┌─────────────────────┐       ┌──────────────┐
│ TeleopVelocityNode  │──────▶│   cmd_vel    │
│  (20 Hz)            │       │   (Twist)    │
└─────────────────────┘       └──────┬───────┘
                                     │
                                     ▼
                      ┌──────────────────────────┐
                      │ VelocityConverterNode    │
                      │ (100 Hz, PID Control)    │
                      │  - SingleMotorModel      │
                      │  - pluginlib loading     │
                      └─────────┬────────────────┘
                                │
                                ▼
                      ┌──────────────────┐
                      │ motor_command    │
                      │ (MotorCommand)   │
                      └─────────┬────────┘
                                │
                                ▼
                      ┌──────────────────┐
                      │ MotorDriverNode  │
                      │ (CAN bus)        │
                      └─────────┬────────┘
                                │
                                ▼
                      ┌──────────────────┐       ┌──────────────────┐
                      │ motor_feedback   │──────▶│ OdometryPublisher│
                      │ (MotorFeedback)  │       │ (50 Hz)          │
                      └──────────────────┘       └────────┬─────────┘
                                                          │
                                          ┌───────────────┴───────────────┐
                                          ▼                               ▼
                                    ┌──────────┐                   ┌──────────┐
                                    │  /odom   │                   │ TF tree  │
                                    │ (nav_msgs)│                   │odom→base │
                                    └──────────┘                   └──────────┘
```

## 🚀 Quick Start

### Prerequisites

```bash
# Ensure workspace is built
cd ~/ws
ros_build_pkg ros2_waveshare

# Set up virtual CAN (for simulation)
sudo ./vcan_start.sh
```

### Launch Complete System

```bash
# Source workspace
source ~/ws/install/setup.bash

# Launch with vcan0 (simulation)
ros2 launch ros2_waveshare single_motor_test.launch.py

# Or with real CAN interface
ros2 launch ros2_waveshare single_motor_test.launch.py can_interface:=can0
```

### Control the Motor

In a separate terminal:

```bash
# Run teleop node for keyboard control
ros2 run ros2_waveshare teleop_velocity_node

# Controls:
#   ↑ - Increase velocity
#   ↓ - Decrease velocity
#   0 - Stop (zero velocity)
#   ESC - Shutdown
```

Or publish commands directly:

```bash
# Send velocity command (0.5 m/s forward)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 📦 Components

### 1. TeleopVelocityNode
- **Executable**: `teleop_velocity_node`
- **Config**: `config/teleop_params.yaml`
- **Purpose**: Keyboard interface for velocity commands
- **Rate**: 20 Hz
- **Features**:
  - Arrow key control (↑↓)
  - Smooth acceleration ramping
  - State machine (Idle, Accelerating, Cruising, etc.)
  - Terminal I/O handling

### 2. VelocityConverterNode
- **Executable**: `velocity_converter_node`
- **Config**: `config/velocity_converter_params.yaml`
- **Purpose**: Convert cmd_vel to motor commands using plugins
- **Rate**: 100 Hz
- **Features**:
  - Pluginlib-based dynamic model loading
  - cmd_vel timeout safety (0.5s default)
  - Runtime plugin reloading via service
  - PID control loop

### 3. OdometryPublisherNode
- **Executable**: `odometry_publisher_node`
- **Config**: `config/odometry_params.yaml`
- **Purpose**: Integrate encoder feedback into odometry
- **Rate**: 50 Hz
- **Features**:
  - nav_msgs/Odometry publishing
  - TF2 broadcasting (odom → base_link)
  - Trapezoidal integration
  - Covariance matrices

### 4. SingleMotorModel Plugin
- **Library**: `libdynamic_models.so`
- **Class**: `ros2_waveshare::SingleMotorModel`
- **Purpose**: PID velocity control for single motor
- **Features**:
  - PID with anti-windup integral
  - Acceleration limiting
  - Linear ↔ angular velocity conversion
  - Tunable gains (kp, ki, kd)

### 5. MotorDriverNode
- **Executable**: `motor_driver_node`
- **Purpose**: Low-level CAN communication
- **Features**:
  - CANopen protocol
  - Motor command/feedback
  - Status monitoring

## ⚙️ Configuration

### Velocity Converter Parameters

Edit `config/velocity_converter_params.yaml`:

```yaml
velocity_converter:
  ros__parameters:
    # Plugin selection
    plugin_name: "ros2_waveshare::SingleMotorModel"
    
    # Control loop
    control_rate: 100.0
    cmd_vel_timeout: 0.5
    
    # Motor configuration
    motor_namespace: "/motors/motor_1"
    node_id: 1
    
    # PID gains
    single_motor_model:
      kp: 1.0
      ki: 0.1
      kd: 0.05
      max_velocity: 1.0
      max_acceleration: 2.0
      max_integral: 10.0
      wheel_radius: 0.1
      gear_ratio: 1.0
```

### Odometry Parameters

Edit `config/odometry_params.yaml`:

```yaml
odometry_publisher:
  ros__parameters:
    motor_namespace: "/motors/motor_1"
    wheel_radius: 0.1
    publish_rate: 50.0
    odom_frame: "odom"
    base_frame: "base_link"
```

### Teleop Parameters

Edit `config/teleop_params.yaml`:

```yaml
teleop_velocity:
  ros__parameters:
    max_velocity: 1.0
    accel_step: 0.1
    publish_rate: 20.0
```

## 🧪 Testing

### Automated Test Script

```bash
cd ~/ws/src/ros2_waveshare/scripts

# Run automated test (30 seconds)
./test_single_motor_system.sh

# Custom duration
./test_single_motor_system.sh --duration 60

# Keep nodes running after test
./test_single_motor_system.sh --no-cleanup

# Help
./test_single_motor_system.sh --help
```

### Manual Testing

1. **Launch system**:
   ```bash
   ros2 launch ros2_waveshare single_motor_test.launch.py
   ```

2. **Monitor topics**:
   ```bash
   # List all topics
   ros2 topic list
   
   # Monitor cmd_vel
   ros2 topic echo /cmd_vel
   
   # Monitor motor command
   ros2 topic echo /motors/motor_1/command
   
   # Monitor motor feedback (if available)
   ros2 topic echo /motors/motor_1/feedback
   
   # Monitor odometry
   ros2 topic echo /odom
   ```

3. **Check TF tree**:
   ```bash
   # View TF tree
   ros2 run tf2_tools view_frames
   
   # Echo specific transform
   ros2 run tf2_ros tf2_echo odom base_link
   ```

4. **Visualize in RViz**:
   ```bash
   rviz2
   # Add:
   # - TF display
   # - Odometry display
   ```

## 🎛️ PID Tuning

See detailed guide: [doc/PID_TUNING_GUIDE.md](doc/PID_TUNING_GUIDE.md)

### Quick Tuning Steps

1. **Start with P only**:
   ```yaml
   kp: 0.5
   ki: 0.0
   kd: 0.0
   ```

2. **Increase Kp until slight oscillation**

3. **Add Ki to eliminate steady-state error**:
   ```yaml
   ki: 0.1
   ```

4. **Add Kd to reduce overshoot**:
   ```yaml
   kd: 0.05
   ```

5. **Fine-tune and test at different velocities**

### Reload Plugin After Parameter Changes

```bash
ros2 service call /velocity_converter/reload_plugin std_srvs/srv/Trigger
```

## 📊 Monitoring

### Topic Rates

```bash
ros2 topic hz /cmd_vel                    # Should be ~20 Hz
ros2 topic hz /motors/motor_1/command     # Should be ~100 Hz
ros2 topic hz /motors/motor_1/feedback    # Depends on motor driver
ros2 topic hz /odom                       # Should be ~50 Hz
```

### PlotJuggler (Real-time Plotting)

```bash
# Install if needed
sudo apt install ros-jazzy-plotjuggler-ros

# Launch
ros2 run plotjuggler plotjuggler

# Subscribe to topics and plot:
# - cmd_vel.linear.x (target)
# - motors/motor_1/feedback.velocity_rad_s (actual)
# - motors/motor_1/command.target_velocity_rad_s (control output)
```

### Node Graph

```bash
rqt_graph
```

## 🔧 Troubleshooting

### Motor doesn't move

1. **Check feedback is arriving**:
   ```bash
   ros2 topic echo /motors/motor_1/feedback
   ```

2. **Check commands are being sent**:
   ```bash
   ros2 topic echo /motors/motor_1/command
   ```

3. **Verify physical parameters**:
   - wheel_radius matches actual wheel
   - gear_ratio is correct

4. **Increase Kp**:
   ```yaml
   kp: 2.0  # or higher
   ```

### Oscillations

1. **Reduce gains**:
   ```yaml
   kp: 0.5   # Reduce by 50%
   ki: 0.05
   kd: 0.01
   ```

2. **Check for delays in feedback**

### No feedback with vcan0

This is **expected** - vcan0 is a virtual interface for testing. For real feedback:
- Use actual CAN hardware (can0)
- Connect real motor
- Ensure motor driver is configured correctly

## 📁 File Structure

```
ros2_waveshare/
├── config/
│   ├── teleop_params.yaml
│   ├── odometry_params.yaml
│   └── velocity_converter_params.yaml
├── doc/
│   ├── PID_TUNING_GUIDE.md
│   └── SINGLE_MOTOR_TEST_README.md (this file)
├── include/ros2_waveshare/
│   ├── teleop_velocity_node.hpp
│   ├── odometry_publisher_node.hpp
│   ├── velocity_converter_node.hpp
│   ├── dynamic_model_interface.hpp
│   └── plugins/
│       └── single_motor_model.hpp
├── src/
│   ├── teleop_velocity_node.cpp
│   ├── teleop_velocity_main.cpp
│   ├── odometry_publisher_node.cpp
│   ├── odometry_publisher_main.cpp
│   ├── velocity_converter_node.cpp
│   ├── velocity_converter_main.cpp
│   └── plugins/
│       └── single_motor_model.cpp
├── launch/
│   └── single_motor_test.launch.py
├── scripts/
│   └── test_single_motor_system.sh
└── dynamic_model_plugins.xml
```

## 🔌 Plugin System

### Available Plugins

1. **SingleMotorModel** ✅ (Implemented)
   - Direct wheel control
   - PID velocity control
   - Linear ↔ angular conversion

2. **DifferentialDriveModel** 📋 (Future)
   - Two-wheel differential drive
   - Twist (linear.x + angular.z) → left/right motors

3. **AckermannModel** 📋 (Future)
   - Car-like steering
   - Steering angle + drive velocity

### Creating Custom Plugins

See `include/ros2_waveshare/dynamic_model_interface.hpp` for the base interface.

## 📚 References

- [ROS2 Control](https://control.ros.org/)
- [pluginlib](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
- [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)
- [nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)

## 🤝 Contributing

For issues, improvements, or new plugin implementations, please contact:
- Andrea Efficace <andrea.efficace1@gmail.com>

## 📄 License

Apache License 2.0

---

**Last Updated**: 2025-11-12  
**Package Version**: 0.0.0  
**ROS2 Distribution**: Jazzy
