# CANOpen Middleware Implementation TODO List

# Architecture Overview

```bash
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 Application Layer                       │
│         (Velocity Commands, Position Feedback, Services)        │
└──────────────────────────────────┬──────────────────────────────┘
                                   │
┌──────────────────────────────────▼──────────────────────────────┐
│                      ros2_control Layer                         │
│  • JointStateController    • VelocityController                 │
│  • PositionController      • EffortController                   │
└──────────────────────────────────┬──────────────────────────────┘
                                   │
┌──────────────────────────────────▼──────────────────────────────┐
│                    ros2_canopen Layer                           │
│  • CANopen Master (Device Manager)                              │
│  • CiA 402 Driver (Motor Control Profile)                       │
│  • SDO Client • PDO Manager • NMT Manager                       │
│  • EDS Parser (mini_traction_pwr.eds)                           │
└──────────────────────────────────┬──────────────────────────────┘
                                   │
                            SocketCAN (vcan0)
                                   │
┌──────────────────────────────────▼──────────────────────────────┐
│              Waveshare SocketCAN Bridge (NEW)                   │
│  • Frame conversion: SocketCAN ↔ Waveshare protocol             │
│  • Bidirectional forwarding threads                             │
│  • Uses existing USBAdapter + SocketCANHelper                   │
└──────────────────────────────────┬──────────────────────────────┘
                                   │
┌──────────────────────────────────▼──────────────────────────────┐
│              Waveshare USB-CAN-A Adapter                        │
│                    (Serial /dev/ttyUSB0)                        │
└──────────────────────────────────┬──────────────────────────────┘
                                   │
                          MICROPHASE Motor Driver
```
```bash
waveshare_socketcan_bridge/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── bridge_config.yaml
├── include/waveshare_socketcan_bridge/
│   ├── bridge_node.hpp
│   └── socketcan_manager.hpp
├── src/
│   ├── bridge_node.cpp
│   ├── socketcan_manager.cpp
│   └── main.cpp
└── launch/
    └── bridge.launch.py
```
## Phase 1: Setup Waveshare-to-SocketCAN Bridge Package

### 1.1 Create ROS2 Package Structure
- [ ] Navigate to ROS2 workspace: `cd /home/ubuntu/ros_ws/src`
- [ ] Create new package: `ros2 pkg create waveshare_socketcan_bridge --build-type ament_cmake --dependencies rclcpp rclcpp_lifecycle diagnostic_updater`
- [ ] Add waveshare_cpp as a dependency in `package.xml` and `CMakeLists.txt`
- [ ] Create directory structure: config, `launch/`, `include/waveshare_socketcan_bridge/`, src

### 1.2 Implement Bridge Components
- [ ] Create `socketcan_manager.hpp` - Manages virtual CAN interface creation/destruction
- [ ] Create `bridge_node.hpp` - ROS2 lifecycle node header
- [ ] Implement `socketcan_manager.cpp` - Uses `ip link add/del` commands for vcan interface
- [ ] Implement `bridge_node.cpp` - Lifecycle callbacks (configure, activate, deactivate, cleanup)
- [ ] Implement `main.cpp` - Node entry point with lifecycle manager
- [ ] Create `bridge_config.yaml` - Configuration parameters (device, baud rates, interface name)
- [ ] Create `bridge.launch.py` - Launch file with parameter loading

### 1.3 Build and Test Bridge
- [ ] Update `CMakeLists.txt` to link waveshare_cpp library
- [ ] Build package: `colcon build --packages-select waveshare_socketcan_bridge`
- [ ] Source workspace: `source install/setup.bash`
- [ ] Test bridge launch: `ros2 launch waveshare_socketcan_bridge bridge.launch.py`
- [ ] Verify virtual CAN interface: `ip link show vcan0`
- [ ] Test with can-utils: `candump vcan0` and `cansend vcan0 123#DEADBEEF`

---

## Phase 2: Install and Configure ros2_canopen

### 2.1 Install ros2_canopen
- [ ] Check ROS2 distro: `echo $ROS_DISTRO` (likely `humble` or `jazzy`)
- [ ] Install binary package: `sudo apt install ros-$ROS_DISTRO-ros2-canopen`
- [ ] **OR** build from source if custom modifications needed:
  - [ ] Clone repo: `cd /home/ubuntu/ros_ws/src && git clone https://github.com/ros-industrial/ros2_canopen.git -b $ROS_DISTRO`
  - [ ] Install dependencies: `rosdep install --from-paths . --ignore-src -r -y`
  - [ ] Build: `colcon build --packages-up-to canopen_core canopen_402_driver`

### 2.2 Obtain Motor Driver EDS File
- [ ] Locate MICROPHASE motor driver documentation
- [ ] Download/obtain EDS file (e.g., `microphase_motor.eds` or similar)
- [ ] Create directory: `mkdir -p /home/ubuntu/ros_ws/src/motor_driver_config/config/eds`
- [ ] Copy EDS file to config directory
- [ ] Validate EDS file syntax (check for DS-402 profile: Object 0x6040, 0x6041, 0x6060, etc.)

### 2.3 Create Motor Driver Configuration Package
- [ ] Create package: `ros2 pkg create motor_driver_config --build-type ament_cmake`
- [ ] Create directory structure:
  ```
  motor_driver_config/
  ├── config/
  │   ├── eds/
  │   │   └── microphase_motor.eds
  │   ├── bus.yml
  │   ├── master.dcf
  │   └── ros2_control.yaml
  └── launch/
      └── motor_system.launch.py
  ```

---

## Phase 3: Configure CANopen Network

### 3.1 Create bus.yml Configuration
- [ ] Define CAN bus configuration:
  - [ ] Set `can_interface_name: "vcan0"`
  - [ ] Configure master node ID (typically 1)
  - [ ] Define motor driver node(s) with IDs (e.g., node_id: 2)
  - [ ] Reference EDS file path
  - [ ] Set PDO mapping (TPDO for sensors, RPDO for commands)

### 3.2 Create master.dcf Configuration
- [ ] Define CANopen master parameters:
  - [ ] Heartbeat producer/consumer settings
  - [ ] Emergency consumer configuration
  - [ ] Sync producer settings (if using synchronous PDOs)
  - [ ] Default SDO timeout values

### 3.3 Test CANopen Network Discovery
- [ ] Launch bridge: `ros2 launch waveshare_socketcan_bridge bridge.launch.py`
- [ ] Launch CANopen master: `ros2 launch canopen_core canopen.launch.py bus_config:=<path/to/bus.yml> master_config:=<path/to/master.dcf>`
- [ ] Check node discovery: `ros2 topic echo /canopen/master/rpdo` (or appropriate topic)
- [ ] Verify NMT state transitions in logs

---

## Phase 4: Integrate with ros2_control

### 4.1 Create ros2_control Configuration
- [ ] Create `ros2_control.yaml` defining:
  - [ ] Hardware interface plugin: `canopen_ros2_control/RobotSystem`
  - [ ] Joint definitions (name, type, state/command interfaces)
  - [ ] Map joints to CANopen objects (0x6064 for position, 0x606C for velocity, etc.)
  - [ ] Controller parameters (PID gains, limits, etc.)

### 4.2 Configure Controllers
- [ ] Define controller configuration:
  - [ ] `joint_state_broadcaster` - Publish joint states
  - [ ] `velocity_controller` or `position_controller` - Control motor
  - [ ] Define controller parameters (update rate, command topic names)

### 4.3 Create Launch File
- [ ] Create `motor_system.launch.py`:
  - [ ] Launch waveshare bridge node
  - [ ] Launch CANopen master with bus/master configs
  - [ ] Load ros2_control configuration
  - [ ] Spawn controllers using `spawner` utility

---

## Phase 5: Testing and Validation

### 5.1 Unit Testing
- [ ] Test bridge node lifecycle transitions
- [ ] Test SocketCAN interface creation/cleanup
- [ ] Verify frame conversion (Waveshare ↔ SocketCAN)
- [ ] Test with mock CAN frames using `cansend`

### 5.2 Integration Testing
- [ ] Power on motor driver and connect CAN bus
- [ ] Launch full system: `ros2 launch motor_driver_config motor_system.launch.py`
- [ ] Verify CANopen state machine: Pre-operational → Operational
- [ ] Check joint state publication: `ros2 topic echo /joint_states`
- [ ] Send velocity command: `ros2 topic pub /velocity_controller/commands ...`
- [ ] Monitor motor response and encoder feedback

### 5.3 Diagnostics and Monitoring
- [ ] Check diagnostics: `ros2 topic echo /diagnostics`
- [ ] Monitor CANopen events: `ros2 topic echo /canopen/master/emcy` (emergency messages)
- [ ] Verify heartbeat: `ros2 topic echo /canopen/master/heartbeat`
- [ ] Test error recovery (disconnect/reconnect scenarios)

---

## Phase 6: Advanced Features (Optional)

### 6.1 Multi-Motor Support
- [ ] Add multiple motor entries in `bus.yml` with unique node IDs
- [ ] Configure separate controllers for each motor
- [ ] Test synchronized motion

### 6.2 Custom Services
- [ ] Create service definitions for motor-specific operations:
  - [ ] `ResetMotorError.srv` - Clear error states
  - [ ] `SetMotorMode.srv` - Change operation mode (position/velocity/torque)
  - [ ] `HomingSequence.srv` - Execute homing procedure
- [ ] Implement service handlers in custom ROS2 node

### 6.3 Motion Planning Integration
- [ ] Integrate with MoveIt2 (if doing trajectory planning)
- [ ] Configure trajectory controllers
- [ ] Define planning groups and kinematic chains

---

## Phase 7: Documentation and Deployment

### 7.1 Documentation
- [ ] Document architecture in README.md
- [ ] Create wiring diagrams (CAN bus topology)
- [ ] Write EDS file mapping guide (which objects control what)
- [ ] Document troubleshooting steps (common errors, solutions)

### 7.2 Deployment
- [ ] Create systemd service for auto-start on boot
- [ ] Configure udev rules for `/dev/ttyUSB*` device permissions
- [ ] Set up logging and monitoring
- [ ] Create backup configurations

---

## Troubleshooting Checklist

### Common Issues to Debug
- [ ] Bridge not creating vcan0: Check permissions, kernel modules (`modprobe vcan`)
- [ ] CANopen node not discovered: Verify node ID, baud rate, physical connections
- [ ] EDS parsing errors: Validate EDS syntax, check mandatory objects
- [ ] PDO not updating: Check PDO mapping in EDS, sync settings
- [ ] Motor not responding: Verify NMT state, check error register (object 0x1001)
- [ ] Performance issues: Monitor CPU usage, adjust PDO transmission rates

---

## Success Criteria

- [ ] Virtual CAN interface automatically created on bridge startup
- [ ] Motor driver discovered and configured via EDS file
- [ ] Joint states published to `/joint_states` at 10+ Hz
- [ ] Velocity/position commands control motor smoothly
- [ ] System recovers gracefully from communication errors
- [ ] No data loss during sustained operation (1+ hour test)

---

This todo list provides a structured path from your existing `waveshare_cpp` library to a fully functional CANopen motor control system using ROS2 standard tools. Start with Phase 1 and validate each step before moving forward!