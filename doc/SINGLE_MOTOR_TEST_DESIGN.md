# Single Motor Test System - Detailed Design Document

**Date:** November 12, 2025  
**Status:** Design Phase  
**Objective:** Test complete control chain with single motor using plugin-based architecture

---

## 1. System Overview

This design implements a complete closed-loop velocity control system for testing a single motor before scaling to multi-motor configurations (differential drive, Ackermann steering).

### 1.1 Key Features
- ✅ Plugin-based dynamic model architecture (extensible)
- ✅ Closed-loop control using encoder feedback
- ✅ Simple teleop interface for testing
- ✅ Odometry calculation from encoder data
- ✅ Safe shutdown procedures
- ✅ Integration with existing CANopen motor driver

### 1.2 Design Philosophy
1. **Plugin Architecture**: Dynamic models are plugins loaded at runtime
2. **Single Responsibility**: Each node has one clear purpose
3. **Feedback-Driven**: Closed-loop control using real encoder data
4. **Safety First**: Graceful degradation and safe shutdown
5. **Scalability**: Architecture supports future expansion

---

## 2. Node Architecture

### 2.1 Teleop Velocity Node
**Package:** `ros2_waveshare`  
**Executable:** `teleop_velocity_node`

#### Purpose
Simple keyboard interface for velocity commands with smooth acceleration/deceleration.

#### ROS2 Interface
```yaml
Publishers:
  /cmd_vel:
    type: geometry_msgs/msg/Twist
    rate: 20 Hz
    description: Target velocity commands

Subscribers:
  /odom:
    type: nav_msgs/msg/Odometry
    description: Current odometry for display (optional)

Parameters:
  max_velocity: 1.0  # m/s
  accel_step: 0.1    # m/s per keypress
  publish_rate: 20   # Hz
```

#### Key Behavior
- **↑ Arrow**: Increase velocity by accel_step (max: +1.0 m/s)
- **↓ Arrow**: Decrease velocity by accel_step (min: -1.0 m/s)
- **'0' Key**: Ramp to zero velocity
- **ESC Key**: Shutdown sequence (send zero, disable motor, exit)

#### State Machine
See diagram: `teleop_state_machine.mmd`

---

### 2.2 Velocity Converter Node
**Package:** `ros2_waveshare`  
**Executable:** `velocity_converter_node`

#### Purpose
Converts high-level velocity commands to motor-specific commands using pluggable dynamic models.

#### ROS2 Interface
```yaml
Subscribers:
  /cmd_vel:
    type: geometry_msgs/msg/Twist
    rate: 20 Hz
    description: Target velocity from teleop

  /motors/motor_1/feedback:
    type: ros2_waveshare_msgs/msg/MotorFeedback
    rate: 100 Hz
    description: Real-time encoder feedback

Publishers:
  /motors/motor_1/command:
    type: ros2_waveshare_msgs/msg/MotorCommand
    rate: 100 Hz
    description: Motor velocity commands

Services:
  ~/reload_plugin:
    type: std_srvs/srv/Trigger
    description: Reload dynamic model plugin

Parameters:
  plugin_name: "single_motor_model"
  plugin_type: "ros2_waveshare::SingleMotorModel"
  control_rate: 100  # Hz
  timeout_ms: 100    # Feedback timeout
```

#### Plugin Architecture
- Uses `pluginlib` for runtime plugin loading
- Base class: `DynamicModelInterface`
- Plugins defined in XML: `dynamic_model_plugins.xml`

#### Control Loop (100 Hz)
```
1. Receive /cmd_vel (target velocity)
2. Receive /motors/motor_1/feedback (current state)
3. Plugin: compute_control(target, current) → motor_command
4. Publish /motors/motor_1/command
```

---

### 2.3 Odometry Publisher Node
**Package:** `ros2_waveshare`  
**Executable:** `odometry_publisher_node`

#### Purpose
Integrate encoder feedback into odometry estimates.

#### ROS2 Interface
```yaml
Subscribers:
  /motors/motor_1/feedback:
    type: ros2_waveshare_msgs/msg/MotorFeedback
    rate: 100 Hz
    description: Encoder position and velocity

Publishers:
  /odom:
    type: nav_msgs/msg/Odometry
    rate: 50 Hz
    description: Integrated odometry

  /tf:
    type: tf2_msgs/msg/TFMessage
    description: Transform odom → base_link

Parameters:
  wheel_radius: 0.1      # meters
  publish_rate: 50       # Hz
  base_frame: "base_link"
  odom_frame: "odom"
```

#### Odometry Calculation
```python
# Single wheel odometry (1D motion)
angular_velocity = encoder_velocity_rad_s
linear_velocity = angular_velocity * wheel_radius

# Integration
delta_t = 1.0 / publish_rate
delta_x = linear_velocity * delta_t * cos(theta)
delta_y = linear_velocity * delta_t * sin(theta)
delta_theta = 0  # Single wheel, no rotation

# Update pose
x += delta_x
y += delta_y
theta += delta_theta  # Always 0 for single motor
```

---

## 3. Plugin System Design

### 3.1 Base Interface

**File:** `include/ros2_waveshare/dynamic_model_interface.hpp`

```cpp
namespace ros2_waveshare {

class DynamicModelInterface {
public:
    virtual ~DynamicModelInterface() = default;
    
    /**
     * @brief Initialize plugin with parameters
     * @param node ROS2 node for parameter access
     * @return true if initialization successful
     */
    virtual bool initialize(
        rclcpp::Node::SharedPtr node,
        const std::string& param_namespace) = 0;
    
    /**
     * @brief Update model with new target and feedback
     * @param cmd_vel Target velocity command (Twist)
     * @param feedback Current motor state (MotorFeedback)
     * @param dt Time since last update (seconds)
     */
    virtual void update(
        const geometry_msgs::msg::Twist& cmd_vel,
        const ros2_waveshare_msgs::msg::MotorFeedback& feedback,
        double dt) = 0;
    
    /**
     * @brief Compute motor command based on current state
     * @return Motor command to send to driver
     */
    virtual ros2_waveshare_msgs::msg::MotorCommand compute_control() = 0;
    
    /**
     * @brief Reset internal state (e.g., integral windup)
     */
    virtual void reset() = 0;
    
    /**
     * @brief Prepare for shutdown (safe stop)
     * @return Final motor command for safe stop
     */
    virtual ros2_waveshare_msgs::msg::MotorCommand shutdown() = 0;
    
    /**
     * @brief Get current model state (for debugging)
     */
    virtual std::string get_state_string() const = 0;
};

}  // namespace ros2_waveshare
```

### 3.2 Single Motor Model Plugin

**File:** `src/plugins/single_motor_model.cpp`

```cpp
class SingleMotorModel : public DynamicModelInterface {
private:
    // PID Controller State
    double target_velocity_;
    double current_velocity_;
    double position_error_;
    double integral_error_;
    double previous_error_;
    
    // PID Gains (tunable parameters)
    double kp_;  // Proportional gain
    double ki_;  // Integral gain
    double kd_;  // Derivative gain
    
    // Limits
    double max_velocity_;
    double max_acceleration_;
    double max_integral_;
    
    // Unit conversion
    double wheel_radius_;
    double gear_ratio_;
    
public:
    bool initialize(rclcpp::Node::SharedPtr node, 
                   const std::string& ns) override {
        // Load parameters
        kp_ = node->declare_parameter(ns + ".kp", 1.0);
        ki_ = node->declare_parameter(ns + ".ki", 0.1);
        kd_ = node->declare_parameter(ns + ".kd", 0.05);
        max_velocity_ = node->declare_parameter(ns + ".max_velocity", 1.0);
        max_acceleration_ = node->declare_parameter(ns + ".max_accel", 2.0);
        wheel_radius_ = node->declare_parameter(ns + ".wheel_radius", 0.1);
        gear_ratio_ = node->declare_parameter(ns + ".gear_ratio", 1.0);
        
        reset();
        return true;
    }
    
    void update(const Twist& cmd_vel, 
                const MotorFeedback& feedback, 
                double dt) override {
        // Convert target linear velocity to angular velocity
        target_velocity_ = cmd_vel.linear.x / wheel_radius_;
        
        // Get current velocity from encoder feedback
        current_velocity_ = feedback.velocity_rad_s;
        
        // Compute error
        double error = target_velocity_ - current_velocity_;
        
        // PID calculation
        integral_error_ += error * dt;
        integral_error_ = std::clamp(integral_error_, 
                                     -max_integral_, max_integral_);
        
        double derivative = (error - previous_error_) / dt;
        previous_error_ = error;
    }
    
    MotorCommand compute_control() override {
        // PID output
        double control_output = kp_ * previous_error_ + 
                               ki_ * integral_error_ + 
                               kd_ * (previous_error_ - position_error_) / dt;
        
        // Apply acceleration limits
        double max_change = max_acceleration_ * dt;
        control_output = std::clamp(control_output,
                                    current_velocity_ - max_change,
                                    current_velocity_ + max_change);
        
        // Create motor command
        MotorCommand cmd;
        cmd.target_velocity_rad_s = control_output;
        cmd.control_mode = MotorCommand::MODE_VELOCITY;
        return cmd;
    }
    
    void reset() override {
        integral_error_ = 0.0;
        previous_error_ = 0.0;
        target_velocity_ = 0.0;
    }
    
    MotorCommand shutdown() override {
        reset();
        MotorCommand cmd;
        cmd.target_velocity_rad_s = 0.0;
        cmd.control_mode = MotorCommand::MODE_VELOCITY;
        return cmd;
    }
};

// Plugin export
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros2_waveshare::SingleMotorModel, 
                      ros2_waveshare::DynamicModelInterface)
```

### 3.3 Plugin XML Registration

**File:** `dynamic_model_plugins.xml`

```xml
<library path="libdynamic_models">
  <class name="ros2_waveshare::SingleMotorModel" 
         type="ros2_waveshare::SingleMotorModel" 
         base_class_type="ros2_waveshare::DynamicModelInterface">
    <description>
      Simple PID-based velocity controller for single motor.
      Uses encoder feedback for closed-loop control.
    </description>
  </class>
  
  <!-- Future plugins -->
  <class name="ros2_waveshare::DifferentialDriveModel" 
         type="ros2_waveshare::DifferentialDriveModel" 
         base_class_type="ros2_waveshare::DynamicModelInterface">
    <description>
      Differential drive kinematics for two-wheel robots.
    </description>
  </class>
  
  <class name="ros2_waveshare::AckermannModel" 
         type="ros2_waveshare::AckermannModel" 
         base_class_type="ros2_waveshare::DynamicModelInterface">
    <description>
      Ackermann steering geometry for car-like robots.
    </description>
  </class>
</library>
```

---

## 4. Launch Configuration

### 4.1 Complete System Launch

**File:** `launch/single_motor_test.launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_waveshare')
    
    # Launch arguments
    motor_id = LaunchConfiguration('motor_id', default='1')
    plugin_name = LaunchConfiguration('plugin_name', 
                                      default='single_motor_model')
    
    return LaunchDescription([
        # 1. Bridge (if not already running)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'bridge_bringup.launch.py')
            )
        ),
        
        # 2. Motor Driver Node
        Node(
            package='ros2_waveshare',
            executable='motor_driver_node',
            name='motor_driver',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'motor_driver_params.yaml'),
                {'motor_ids': [int(motor_id)]}
            ]
        ),
        
        # 3. Velocity Converter with Plugin
        Node(
            package='ros2_waveshare',
            executable='velocity_converter_node',
            name='velocity_converter',
            output='screen',
            parameters=[{
                'plugin_name': plugin_name,
                'plugin_type': f'ros2_waveshare::{plugin_name}',
                'control_rate': 100.0,
                'motor_namespace': f'/motors/motor_{motor_id}',
                
                # Single Motor Model Parameters
                'single_motor_model.kp': 1.0,
                'single_motor_model.ki': 0.1,
                'single_motor_model.kd': 0.05,
                'single_motor_model.max_velocity': 1.0,
                'single_motor_model.max_accel': 2.0,
                'single_motor_model.wheel_radius': 0.1,
            }]
        ),
        
        # 4. Odometry Publisher
        Node(
            package='ros2_waveshare',
            executable='odometry_publisher_node',
            name='odometry_publisher',
            output='screen',
            parameters=[{
                'motor_namespace': f'/motors/motor_{motor_id}',
                'wheel_radius': 0.1,
                'publish_rate': 50.0,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
            }]
        ),
        
        # 5. Teleop Node
        Node(
            package='ros2_waveshare',
            executable='teleop_velocity_node',
            name='teleop_velocity',
            output='screen',
            prefix='xterm -e',  # Run in separate terminal
            parameters=[{
                'max_velocity': 1.0,
                'accel_step': 0.1,
                'publish_rate': 20.0,
            }]
        ),
    ])
```

---

## 5. Parameter Configuration

### 5.1 Velocity Converter Parameters

**File:** `config/velocity_converter_params.yaml`

```yaml
velocity_converter:
  ros__parameters:
    # Plugin selection
    plugin_name: "single_motor_model"
    plugin_type: "ros2_waveshare::SingleMotorModel"
    
    # Control loop
    control_rate: 100.0  # Hz
    feedback_timeout_ms: 100
    
    # Single Motor Model
    single_motor_model:
      # PID gains (tune these!)
      kp: 1.0
      ki: 0.1
      kd: 0.05
      
      # Limits
      max_velocity: 1.0      # m/s
      max_acceleration: 2.0  # m/s²
      max_integral: 10.0     # Anti-windup
      
      # Physical parameters
      wheel_radius: 0.1      # meters
      gear_ratio: 1.0
      encoder_resolution: 10000  # counts/rev
```

### 5.2 Odometry Parameters

**File:** `config/odometry_params.yaml`

```yaml
odometry_publisher:
  ros__parameters:
    # Motor feedback source
    motor_namespace: "/motors/motor_1"
    
    # Physical parameters
    wheel_radius: 0.1  # meters
    
    # Publishing
    publish_rate: 50.0  # Hz
    
    # TF frames
    base_frame: "base_link"
    odom_frame: "odom"
    publish_tf: true
    
    # Initial pose
    initial_x: 0.0
    initial_y: 0.0
    initial_theta: 0.0
```

---

## 6. Testing Procedure

### 6.1 Pre-Test Checklist
- [ ] CAN hardware connected (or vcan0 ready)
- [ ] Motor driver powered and configured
- [ ] Motor enabled via enable action
- [ ] All nodes launched successfully
- [ ] Feedback topic publishing at 100 Hz

### 6.2 Test Sequence

```bash
# Terminal 1: Launch complete system
ros2 launch ros2_waveshare single_motor_test.launch.py

# Terminal 2: Monitor feedback
ros2 topic echo /motors/motor_1/feedback --no-arr

# Terminal 3: Monitor commands
ros2 topic echo /motors/motor_1/command --no-arr

# Terminal 4: Monitor odometry
ros2 topic echo /odom --no-arr

# Teleop window opens automatically
# Use keyboard:
# - Press ↑ to accelerate forward
# - Press ↓ to accelerate backward
# - Press 0 to stop
# - Press ESC to shutdown
```

### 6.3 Expected Behavior
1. **Acceleration**: Velocity ramps smoothly from 0 to 1.0 m/s
2. **Feedback Loop**: Command adjusts based on encoder feedback
3. **Steady State**: Velocity stabilizes at target with minimal error
4. **Deceleration**: Smooth ramp back to zero
5. **Odometry**: Position integrates correctly (roughly linear motion)

### 6.4 Tuning PID Parameters
If oscillations occur:
- Reduce `kp` (proportional gain)
- Reduce `kd` (derivative gain)

If steady-state error exists:
- Increase `ki` (integral gain)
- Check `max_integral` limit

---

## 7. Safety Features

### 7.1 Timeout Protection
- **Feedback Timeout**: If no feedback for 100ms → send zero velocity
- **Command Timeout**: Motor driver stops if no command for 200ms

### 7.2 Shutdown Sequence
1. Teleop sends zero velocity on ESC
2. Velocity converter ramps motor to zero
3. Wait for velocity < 0.01 m/s
4. Call motor disable service
5. Exit all nodes

### 7.3 Emergency Stop
- **'0' Key**: Immediate ramp to zero (not instant stop)
- **Ctrl+C**: Calls shutdown() on all nodes
- **Motor fault**: Automatically stops command publishing

---

## 8. Future Extensions

### 8.1 Differential Drive Plugin
**File:** `differential_drive_model.cpp`
- Left/right motor commands from Twist
- Odometry with rotation
- Wheel base configuration

### 8.2 Ackermann Plugin
**File:** `ackermann_model.cpp`
- Steering angle control
- Four-wheel kinematics
- Vehicle dynamics model

### 8.3 Advanced Features
- Trajectory planning
- Obstacle avoidance
- Sensor fusion with IMU
- Nav2 integration

---

## 9. File Structure

```
ros2_waveshare/
├── include/ros2_waveshare/
│   ├── dynamic_model_interface.hpp       # Plugin base class
│   ├── velocity_converter_node.hpp
│   ├── odometry_publisher_node.hpp
│   └── teleop_velocity_node.hpp
├── src/
│   ├── velocity_converter_node.cpp
│   ├── odometry_publisher_node.cpp
│   ├── teleop_velocity_node.cpp
│   └── plugins/
│       ├── single_motor_model.cpp
│       ├── differential_drive_model.cpp  # Future
│       └── ackermann_model.cpp           # Future
├── config/
│   ├── velocity_converter_params.yaml
│   └── odometry_params.yaml
├── launch/
│   └── single_motor_test.launch.py
├── dynamic_model_plugins.xml
├── CMakeLists.txt                        # Add plugin library
└── package.xml                           # Add pluginlib dependency
```

---

## 10. Dependencies

### 10.1 New Package Dependencies
```xml
<!-- package.xml -->
<depend>pluginlib</depend>
<depend>nav_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

### 10.2 CMakeLists.txt Additions
```cmake
# Find pluginlib
find_package(pluginlib REQUIRED)

# Plugin library
add_library(dynamic_models SHARED
  src/plugins/single_motor_model.cpp
)
target_link_libraries(dynamic_models ${pluginlib_LIBRARIES})

# Install plugin description
install(FILES dynamic_model_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(
  ros2_waveshare dynamic_model_plugins.xml
)
```

---

## Summary

This design provides:
✅ Complete single-motor test system
✅ Plugin architecture for future expansion  
✅ Closed-loop control with encoder feedback  
✅ Simple teleop interface  
✅ Odometry integration  
✅ Safe shutdown procedures  
✅ Integration with existing motor driver  

**Next Step:** Implementation phase - start with teleop node (simplest), then odometry, then velocity converter with plugin system.
