# ROS2 USB-CAN-A Traction Driver Interface Package Design

## Package Overview

This document outlines the design for a ROS2 package that integrates the Waveshare USB-CAN-A adapter with a traction driver (inverter) for motor control applications.

## Package Architecture

### Package Name: `usb_can_traction_driver`

### Core Components

#### 1. USB-CAN-A Driver Node (`usb_can_driver_node`)
**Purpose**: Low-level interface to USB-CAN-A hardware
**Responsibilities**:
- Initialize and configure USB-CAN-A adapter
- Handle serial communication protocol
- Convert ROS2 messages to CAN frames
- Publish received CAN frames as ROS2 messages
- Manage device connection and error handling

#### 2. Traction Controller Node (`traction_controller_node`)
**Purpose**: High-level motor control interface
**Responsibilities**:
- Accept speed/torque commands via ROS2 topics
- Convert commands to appropriate CAN protocol for inverter
- Implement safety limits and validation
- Provide status feedback and diagnostics
- Handle emergency stop conditions

#### 3. CAN Protocol Handler (`can_protocol_handler`)
**Purpose**: Inverter-specific CAN protocol implementation
**Responsibilities**:
- Define CAN message structures for specific inverter model
- Implement protocol-specific encoding/decoding
- Handle inverter status interpretation
- Manage control mode switching

## Message Interfaces

### Input Messages (Command Interface)

#### 1. Twist Messages (geometry_msgs/Twist)
```yaml
Topic: /cmd_vel
Message Type: geometry_msgs/msg/Twist
Description: Standard ROS2 velocity commands
Fields:
  linear.x: Forward/backward speed (m/s)
  angular.z: Rotational speed (rad/s)
```

#### 2. Custom Speed Commands
```yaml
Topic: /traction/speed_cmd
Message Type: usb_can_traction_driver_msgs/msg/SpeedCommand
Fields:
  speed_rpm: int32          # Target speed in RPM
  acceleration: float32     # Acceleration limit (RPM/s)
  mode: uint8              # Control mode (speed/torque/position)
  enable: bool             # Motor enable/disable
  emergency_stop: bool     # Emergency stop flag
```

#### 3. Torque Commands
```yaml
Topic: /traction/torque_cmd  
Message Type: usb_can_traction_driver_msgs/msg/TorqueCommand
Fields:
  torque_nm: float32       # Target torque in Nm
  speed_limit_rpm: int32   # Maximum speed limit
  enable: bool             # Motor enable/disable
```

### Output Messages (Status Interface)

#### 1. Motor Status
```yaml
Topic: /traction/motor_status
Message Type: usb_can_traction_driver_msgs/msg/MotorStatus
Fields:
  actual_speed_rpm: int32      # Current motor speed
  actual_torque_nm: float32    # Current torque output
  motor_temperature_c: float32 # Motor temperature
  inverter_temperature_c: float32 # Inverter temperature
  dc_voltage_v: float32        # DC bus voltage
  dc_current_a: float32        # DC current
  ac_current_a: float32        # AC current RMS
  fault_code: uint32           # Error/fault codes
  operating_mode: uint8        # Current operating mode
  enabled: bool                # Motor enabled status
```

#### 2. CAN Frame Messages (for debugging)
```yaml
Topic: /can/tx_frames
Topic: /can/rx_frames
Message Type: usb_can_traction_driver_msgs/msg/CANFrame
Fields:
  timestamp: builtin_interfaces/msg/Time
  can_id: uint32               # CAN message ID
  data: uint8[8]              # CAN data payload
  dlc: uint8                  # Data length code
  is_extended: bool           # Extended frame flag
  is_rtr: bool               # Remote transmission request
```

## Package Structure

```
usb_can_traction_driver/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── inverter_config.yaml      # Inverter-specific parameters
│   ├── can_interface_config.yaml # CAN interface settings
│   └── safety_limits.yaml        # Speed/torque safety limits
├── include/usb_can_traction_driver/
│   ├── usb_can_driver.hpp         # USB-CAN-A driver class
│   ├── can_protocol_handler.hpp   # CAN protocol definitions
│   ├── traction_controller.hpp    # High-level controller
│   └── safety_monitor.hpp         # Safety systems
├── src/
│   ├── usb_can_driver.cpp         # USB-CAN-A implementation
│   ├── can_protocol_handler.cpp   # Protocol handling
│   ├── traction_controller.cpp    # Controller logic
│   ├── safety_monitor.cpp         # Safety implementation
│   ├── usb_can_driver_node.cpp    # Driver node main
│   └── traction_controller_node.cpp # Controller node main
├── launch/
│   ├── traction_driver.launch.py  # Complete system launch
│   ├── usb_can_only.launch.py     # USB-CAN driver only
│   └── simulation.launch.py       # Simulation mode
├── msg/
│   ├── SpeedCommand.msg           # Speed command message
│   ├── TorqueCommand.msg          # Torque command message
│   ├── MotorStatus.msg            # Motor status message
│   └── CANFrame.msg               # Raw CAN frame message
├── srv/
│   ├── SetOperatingMode.srv       # Change operating mode
│   ├── CalibrateMotor.srv         # Motor calibration
│   └── ResetFaults.srv            # Clear fault conditions
└── scripts/
    ├── can_monitor.py             # CAN traffic monitoring tool
    ├── motor_test.py              # Basic motor testing script
    └── diagnostics.py             # System diagnostics tool
```

## Implementation Details

### 1. USB-CAN-A Driver Class

```cpp
class USBCANDriver {
public:
    USBCANDriver(const std::string& device_path, uint32_t can_speed);
    ~USBCANDriver();
    
    bool initialize();
    bool configure(uint32_t speed, CANMode mode);
    bool sendFrame(const CANFrame& frame);
    bool receiveFrame(CANFrame& frame, int timeout_ms = 100);
    bool isConnected() const;
    
private:
    int tty_fd_;
    std::string device_path_;
    uint32_t can_speed_;
    bool is_initialized_;
    
    bool setupSerial();
    bool sendCommand(const std::vector<uint8_t>& command);
    bool receiveResponse(std::vector<uint8_t>& response);
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);
};
```

### 2. CAN Protocol Handler

```cpp
class CANProtocolHandler {
public:
    CANProtocolHandler(uint8_t inverter_node_id);
    
    // Command generation
    CANFrame createSpeedCommand(int32_t speed_rpm, bool enable);
    CANFrame createTorqueCommand(float torque_nm, bool enable);
    CANFrame createEmergencyStop();
    
    // Status parsing
    bool parseMotorStatus(const CANFrame& frame, MotorStatus& status);
    bool parseFaultStatus(const CANFrame& frame, uint32_t& fault_code);
    
    // Protocol-specific constants
    static constexpr uint32_t SPEED_CMD_ID = 0x200;
    static constexpr uint32_t TORQUE_CMD_ID = 0x201;
    static constexpr uint32_t STATUS_ID = 0x300;
    static constexpr uint32_t FAULT_ID = 0x301;
    
private:
    uint8_t node_id_;
    
    uint32_t calculateCANID(uint32_t base_id, uint8_t node_id);
    void encodeSpeed(int32_t speed_rpm, uint8_t* data);
    void encodeTorque(float torque_nm, uint8_t* data);
    int32_t decodeSpeed(const uint8_t* data);
    float decodeTorque(const uint8_t* data);
};
```

### 3. Traction Controller Node

```cpp
class TractionControllerNode : public rclcpp::Node {
public:
    TractionControllerNode();
    ~TractionControllerNode();
    
private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<SpeedCommand>::SharedPtr speed_cmd_sub_;
    rclcpp::Subscription<TorqueCommand>::SharedPtr torque_cmd_sub_;
    
    // Publishers
    rclcpp::Publisher<MotorStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<CANFrame>::SharedPtr can_tx_pub_;
    
    // Services
    rclcpp::Service<SetOperatingMode>::SharedPtr mode_service_;
    rclcpp::Service<ResetFaults>::SharedPtr reset_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    // Callback functions
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void speedCommandCallback(const SpeedCommand::SharedPtr msg);
    void torqueCommandCallback(const TorqueCommand::SharedPtr msg);
    void statusTimerCallback();
    void safetyTimerCallback();
    
    // Service callbacks
    void setOperatingModeCallback(
        const SetOperatingMode::Request::SharedPtr request,
        SetOperatingMode::Response::SharedPtr response);
    
    // Members
    std::unique_ptr<CANProtocolHandler> protocol_handler_;
    SafetyMonitor safety_monitor_;
    OperatingMode current_mode_;
    bool motor_enabled_;
    rclcpp::Time last_command_time_;
};
```

### 4. Safety Monitor

```cpp
class SafetyMonitor {
public:
    SafetyMonitor();
    
    bool checkSpeedLimits(int32_t speed_rpm);
    bool checkTorqueLimits(float torque_nm);
    bool checkTemperatureLimits(float motor_temp, float inverter_temp);
    bool checkCommandTimeout(const rclcpp::Time& last_command);
    bool checkEmergencyStop();
    
    void setSpeedLimits(int32_t max_speed, int32_t max_acceleration);
    void setTorqueLimits(float max_torque);
    void setTemperatureLimits(float max_motor_temp, float max_inverter_temp);
    void setCommandTimeout(double timeout_seconds);
    
private:
    int32_t max_speed_rpm_;
    int32_t max_acceleration_rpm_s_;
    float max_torque_nm_;
    float max_motor_temperature_;
    float max_inverter_temperature_;
    double command_timeout_seconds_;
    bool emergency_stop_active_;
};
```

## Configuration Files

### 1. Inverter Configuration (`config/inverter_config.yaml`)
```yaml
inverter:
  node_id: 1
  can_speed: 500000  # 500 kbps
  protocol_type: "custom"  # or "canopen", "j1939", etc.
  
  # CAN Message IDs
  can_ids:
    speed_command: 0x200
    torque_command: 0x201
    motor_status: 0x300
    fault_status: 0x301
    
  # Physical limits
  limits:
    max_speed_rpm: 3000
    max_torque_nm: 100.0
    max_acceleration_rpm_s: 1000
    max_motor_temperature_c: 120.0
    max_inverter_temperature_c: 80.0
    
  # Control parameters
  control:
    command_timeout_ms: 1000
    status_publish_rate_hz: 50
    safety_check_rate_hz: 100
```

### 2. CAN Interface Configuration (`config/can_interface_config.yaml`)
```yaml
usb_can:
  device_path: "/dev/ttyUSB0"
  serial_baudrate: 2000000
  can_speed: 500000
  operating_mode: "normal"  # normal, loopback, silent
  frame_type: "standard"    # standard, extended
  
  # Retry and timeout settings
  connection_retry_count: 3
  connection_timeout_ms: 5000
  frame_timeout_ms: 100
  
  # Debug settings
  enable_traffic_debug: false
  log_can_frames: false
```

## Launch File Example

### Complete System Launch (`launch/traction_driver.launch.py`)
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('usb_can_traction_driver')
    
    # Configuration files
    inverter_config = os.path.join(pkg_dir, 'config', 'inverter_config.yaml')
    can_config = os.path.join(pkg_dir, 'config', 'can_interface_config.yaml')
    
    # Launch arguments
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/ttyUSB0',
        description='USB-CAN-A device path'
    )
    
    can_speed_arg = DeclareLaunchArgument(
        'can_speed',
        default_value='500000',
        description='CAN bus speed in bps'
    )
    
    # USB-CAN Driver Node
    usb_can_driver = Node(
        package='usb_can_traction_driver',
        executable='usb_can_driver_node',
        name='usb_can_driver',
        parameters=[
            can_config,
            {
                'device_path': LaunchConfiguration('device_path'),
                'can_speed': LaunchConfiguration('can_speed')
            }
        ],
        output='screen'
    )
    
    # Traction Controller Node
    traction_controller = Node(
        package='usb_can_traction_driver',
        executable='traction_controller_node',
        name='traction_controller',
        parameters=[inverter_config],
        remappings=[
            ('/cmd_vel', '/traction/cmd_vel'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        device_path_arg,
        can_speed_arg,
        usb_can_driver,
        traction_controller,
    ])
```

## Usage Examples

### 1. Basic Speed Control
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from usb_can_traction_driver_msgs.msg import SpeedCommand

class SimpleSpeedController(Node):
    def __init__(self):
        super().__init__('simple_speed_controller')
        self.publisher = self.create_publisher(
            SpeedCommand, 
            '/traction/speed_cmd', 
            10
        )
        
        # Send speed command every 100ms
        self.timer = self.create_timer(0.1, self.send_speed_command)
        self.target_speed = 0
        
    def send_speed_command(self):
        msg = SpeedCommand()
        msg.speed_rpm = self.target_speed
        msg.acceleration = 500.0  # RPM/s
        msg.mode = 1  # Speed control mode
        msg.enable = True
        msg.emergency_stop = False
        
        self.publisher.publish(msg)
        
    def set_speed(self, speed_rpm):
        self.target_speed = speed_rpm

if __name__ == '__main__':
    rclpy.init()
    controller = SimpleSpeedController()
    
    # Ramp up speed
    for speed in range(0, 1000, 100):
        controller.set_speed(speed)
        rclpy.spin_once(controller, timeout_sec=1.0)
    
    rclpy.shutdown()
```

### 2. Twist Message Interface
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistSpeedController(Node):
    def __init__(self):
        super().__init__('twist_speed_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def send_twist_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x  # m/s
        msg.angular.z = angular_z  # rad/s
        self.publisher.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    controller = TwistSpeedController()
    
    # Move forward at 2 m/s
    controller.send_twist_command(2.0, 0.0)
    
    rclpy.spin_once(controller, timeout_sec=0.1)
    rclpy.shutdown()
```

## Development Roadmap

### Phase 1: Basic Implementation
- [ ] USB-CAN-A driver wrapper
- [ ] Basic CAN frame transmission/reception
- [ ] Simple speed command interface
- [ ] Motor status monitoring

### Phase 2: Safety and Robustness
- [ ] Safety monitoring system
- [ ] Emergency stop handling
- [ ] Connection failure recovery
- [ ] Command timeout protection

### Phase 3: Advanced Features
- [ ] Multiple control modes (speed/torque/position)
- [ ] Advanced diagnostics
- [ ] Parameter tuning interface
- [ ] Data logging capabilities

### Phase 4: Integration and Testing
- [ ] Unit tests for all components
- [ ] Integration tests with real hardware
- [ ] Performance optimization
- [ ] Documentation and examples

## Inverter Protocol Considerations

The package design is generic enough to support different inverter protocols. You'll need to customize the `CANProtocolHandler` class based on your specific inverter's CAN protocol. Common inverter protocols include:

1. **Custom Manufacturer Protocols**: Most inverters have proprietary CAN protocols
2. **CANopen**: Standardized protocol (DS-301/DS-402)
3. **J1939**: Heavy-duty vehicle protocol
4. **ModBus over CAN**: Industrial protocol adaptation

For implementation, you'll need:
- Inverter CAN protocol documentation
- CAN message ID assignments
- Data format specifications
- Control mode definitions
- Status/fault code meanings

This design provides a solid foundation for integrating the USB-CAN-A adapter with ROS2 for traction motor control while maintaining safety, modularity, and extensibility.
