# CANOpen Motor Control System - Next Steps

## Current Status ✅

### Completed Components:
1. ✅ **Waveshare Bridge Lifecycle Node** (`ros2_waveshare` package)
   - Bridges Waveshare USB-CAN-A adapter to SocketCAN (vcan0)
   - Lifecycle management (configure → activate → deactivate)
   - Diagnostics integration with bridge statistics
   - Configuration via YAML parameters

2. ✅ **CANOpen Configuration** (`microphase_can_config` package)
   - EDS file for TRACTION_PWR motor (CiA 402 compliant)
   - Automatic DCF generation from EDS using `cogen_dcf()`
   - Bus configuration with 4 motors (node IDs 2-5)
   - Unified launch file that starts both bridge and CANopen stack

3. ✅ **Build System**
   - Both packages build successfully
   - Config files properly installed
   - Launch files integrated

---

## Next Steps for Development

### Phase 1: System Testing (No Hardware Required)

#### 1. Test Complete System Launch
```bash
# Terminal 1: Launch the complete system
cd /home/ubuntu/ros_ws
source install/setup.bash
ros2 launch microphase_can_config microphase_can_config.launch.py

# Expected output:
# - Waveshare bridge starts and activates
# - SocketCAN interface vcan0 created
# - CANopen device manager starts
# - Attempts to discover motors on the bus
```

#### 2. Verify System Components
```bash
# Terminal 2: Check running nodes
ros2 node list
# Expected:
# /waveshare_bridge
# /device_manager_node (or similar CANopen node)

# Check lifecycle states
ros2 lifecycle list
ros2 lifecycle get /waveshare_bridge
# Should be: active

# Monitor diagnostics
ros2 topic echo /diagnostics

# List all topics
ros2 topic list
# Look for CANopen topics like /rpdo, /tpdo, etc.
```

#### 3. Monitor CAN Traffic (Simulation)
```bash
# Terminal 3: Monitor vcan0 traffic
candump vcan0

# Terminal 4: Send test CAN message
cansend vcan0 123#DEADBEEF

# You should see the message in candump
```

---

### Phase 2: Hardware Integration

#### Prerequisites:
- MICROPHASE motor driver powered on
- Motor driver configured for:
  - Node ID: 2 (or 3, 4, 5 for other motors)
  - CAN baud rate: 1 Mbps (1000000)
  - CAN bus properly terminated (120Ω resistors at both ends)
- Waveshare USB-CAN-A connected via USB

#### 1. Hardware Setup
```bash
# Check USB device presence
ls -l /dev/ttyUSB*
# Should show /dev/ttyUSB0 (or similar)

# Fix permissions if needed
sudo chmod 666 /dev/ttyUSB0

# OR create udev rule for permanent fix:
sudo nano /etc/udev/rules.d/99-usb-serial.rules
# Add: SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666"
sudo udevadm control --reload-rules
```

#### 2. Configure Bridge Parameters
Edit `/home/ubuntu/ros_ws/src/ros2_waveshare/config/bridge_params.yaml`:
```yaml
usb_device: "/dev/ttyUSB0"  # Adjust if different
can_baud: 1000000           # Must match motor driver
```

#### 3. Launch with Hardware
```bash
cd /home/ubuntu/ros_ws
source install/setup.bash

# Launch system
ros2 launch microphase_can_config microphase_can_config.launch.py

# Expected:
# - Bridge connects to /dev/ttyUSB0
# - CANopen performs NMT boot-up sequence
# - Motor nodes discovered and initialized
```

#### 4. Monitor Motor Communication
```bash
# Terminal 2: Monitor CAN traffic
candump vcan0

# Look for:
# - Heartbeat messages (0x700 + node_id)
# - SDO communication (0x580-0x5FF, 0x600-0x67F)
# - PDO messages (0x180-0x57F)

# Terminal 3: Check CANopen topics
ros2 topic echo /front_left_traction/tpdo
# Should show motor status, position, velocity

# Check diagnostics
ros2 topic echo /diagnostics
# Look for motor-specific diagnostic messages
```

---

### Phase 3: Motor Control Implementation

#### 1. Understand CiA 402 State Machine

Your motor follows the CiA 402 profile. Key states:
1. **Not Ready to Switch On** (initial)
2. **Switch On Disabled**
3. **Ready to Switch On**
4. **Switched On**
5. **Operation Enabled** ← Target state for control
6. **Fault** (if errors occur)

State transitions via **Controlword (0x6040)**:
- Shutdown: 0x06
- Switch On: 0x07
- Enable Operation: 0x0F
- Quick Stop: 0x02
- Fault Reset: 0x80

#### 2. Create Motor Control Script

Create `/home/ubuntu/ros_ws/src/microphase_can_config/scripts/motor_control.py`:

```python
#!/usr/bin/env python3
"""Simple motor control script using ros2_canopen."""

import rclpy
from rclpy.node import Node
from canopen_interfaces.msg import COData
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Publisher for RPDO (commands to motor)
        self.cmd_pub = self.create_publisher(
            COData,
            '/front_left_traction/rpdo',
            10
        )
        
        # Subscriber for TPDO (status from motor)
        self.status_sub = self.create_subscription(
            COData,
            '/front_left_traction/tpdo',
            self.status_callback,
            10
        )
        
        self.statusword = 0
        
    def status_callback(self, msg):
        """Receive motor status."""
        if msg.index == 0x6041:  # Statusword
            self.statusword = msg.data
            self.get_logger().info(f'Motor status: 0x{self.statusword:04X}')
    
    def send_sdo(self, index, subindex, data):
        """Send SDO command."""
        msg = COData()
        msg.index = index
        msg.subindex = subindex
        msg.data = data
        self.cmd_pub.publish(msg)
        time.sleep(0.1)  # Wait for processing
    
    def enable_motor(self):
        """Execute CiA 402 state machine to enable operation."""
        self.get_logger().info('Enabling motor...')
        
        # 1. Shutdown
        self.send_sdo(0x6040, 0, 0x06)
        time.sleep(0.2)
        
        # 2. Switch On
        self.send_sdo(0x6040, 0, 0x07)
        time.sleep(0.2)
        
        # 3. Enable Operation
        self.send_sdo(0x6040, 0, 0x0F)
        time.sleep(0.2)
        
        self.get_logger().info('Motor enabled!')
    
    def set_velocity_mode(self):
        """Set motor to velocity mode (CSV - 0x09)."""
        self.get_logger().info('Setting velocity mode...')
        self.send_sdo(0x6060, 0, 0x09)  # Modes of operation
    
    def set_velocity(self, rpm):
        """Set target velocity in RPM."""
        self.get_logger().info(f'Setting velocity: {rpm} RPM')
        self.send_sdo(0x60FF, 0, int(rpm))  # Target velocity
    
    def disable_motor(self):
        """Disable motor (quick stop)."""
        self.get_logger().info('Disabling motor...')
        self.send_sdo(0x6040, 0, 0x02)


def main():
    rclpy.init()
    controller = MotorController()
    
    try:
        # Give time for connection
        time.sleep(2.0)
        
        # Enable motor
        controller.enable_motor()
        
        # Set velocity mode
        controller.set_velocity_mode()
        
        # Set velocity to 100 RPM
        controller.set_velocity(100)
        
        # Run for 5 seconds
        rclpy.spin_until_future_complete(
            controller,
            rclpy.task.Future(),
            timeout_sec=5.0
        )
        
        # Stop motor
        controller.set_velocity(0)
        time.sleep(1.0)
        
        # Disable motor
        controller.disable_motor()
        
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make it executable:
```bash
chmod +x /home/ubuntu/ros_ws/src/microphase_can_config/scripts/motor_control.py
```

#### 3. Test Motor Control
```bash
# Terminal 1: Launch system
ros2 launch microphase_can_config microphase_can_config.launch.py

# Terminal 2: Run control script
python3 /home/ubuntu/ros_ws/src/microphase_can_config/scripts/motor_control.py

# Expected behavior:
# - Motor goes through state transitions
# - Motor starts rotating at 100 RPM
# - Motor stops after 5 seconds
```

---

### Phase 4: Advanced Features

#### 1. Position Control Mode
```python
# Set position mode (CSP - 0x08)
self.send_sdo(0x6060, 0, 0x08)

# Set target position (encoder counts)
self.send_sdo(0x607A, 0, 10000)  # Move 10000 counts
```

#### 2. Torque Control Mode
```python
# Set torque mode (CST - 0x0A)
self.send_sdo(0x6060, 0, 0x0A)

# Set target torque (per-mille of rated torque)
self.send_sdo(0x6071, 0, 500)  # 50% torque
```

#### 3. Read Motor Parameters
```python
# Read actual position
self.send_sdo(0x6064, 0, 0)  # Position actual value

# Read actual velocity
self.send_sdo(0x606C, 0, 0)  # Velocity actual value

# Read actual torque
self.send_sdo(0x6077, 0, 0)  # Torque actual value
```

#### 4. Error Handling
```python
def check_fault(self):
    """Check if motor is in fault state."""
    if self.statusword & 0x08:  # Fault bit
        self.get_logger().error('Motor in FAULT state!')
        # Read error code
        self.send_sdo(0x603F, 0, 0)  # Error code
        # Reset fault
        self.send_sdo(0x6040, 0, 0x80)  # Fault reset
```

---

### Phase 5: Integration with ros2_control (Optional)

#### 1. Create Hardware Interface
For more advanced control (MoveIt2, Nav2 integration), implement a ros2_control hardware interface.

Reference documentation:
- https://control.ros.org/jazzy/doc/getting_started/getting_started.html
- https://ros-industrial.github.io/ros2_canopen/

#### 2. Create URDF with ros2_control Tags
```xml
<ros2_control name="motor_system" type="system">
  <hardware>
    <plugin>canopen_ros2_control/RobotSystem</plugin>
    <param name="bus_config">config/can_config/bus.yml</param>
  </hardware>
  
  <joint name="wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

---

## Troubleshooting Guide

### Issue: Bridge doesn't activate
```bash
# Check USB device
ls -l /dev/ttyUSB0

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Check bridge logs
ros2 lifecycle get /waveshare_bridge
```

### Issue: No CAN traffic on vcan0
```bash
# Verify vcan0 exists
ip link show vcan0

# Check if bridge is forwarding
candump vcan0  # Should see traffic when motor is connected
```

### Issue: Motor not discovered
```bash
# Verify motor node ID matches bus.yml
# Check motor is powered on
# Verify CAN baud rate matches (1 Mbps)
# Check CAN bus termination (120Ω at both ends)

# Monitor raw CAN traffic
candump -d vcan0  # -d shows data decoding
```

### Issue: Motor in FAULT state
```bash
# Check statusword (0x6041)
# Bit 3 = 1 indicates fault
# Read error code (0x603F)
# Send fault reset (Controlword = 0x80)
```

### Issue: Communication timeout
```bash
# Check bridge diagnostics
ros2 topic echo /diagnostics | grep -i error

# Verify serial baud rate (should be 2 Mbps)
# Check USB cable quality
# Try different USB port
```

---

## Key Configuration Files Summary

### 1. Bridge Configuration
**File**: `/home/ubuntu/ros_ws/src/ros2_waveshare/config/bridge_params.yaml`
```yaml
socketcan_interface: "vcan0"
usb_device: "/dev/ttyUSB0"
serial_baud: 2000000  # 2 Mbps
can_baud: 1000000     # 1 Mbps - must match motor
```

### 2. CANopen Bus Configuration
**File**: `/home/ubuntu/ros_ws/src/microphase_can_config/config/can_config/bus.yml`
```yaml
master:
  node_id: 1
  baudrate: 1000000  # Must match bridge can_baud

nodes:
  front_left_traction:
    node_id: 2
    dcf: eds/TRACTION_PWR.eds  # Auto-generates master.dcf
```

### 3. Launch Files
- **Bridge only**: `ros2 launch ros2_waveshare bridge_bringup.launch.py`
- **Complete system**: `ros2 launch microphase_can_config microphase_can_config.launch.py`

---

## Important CiA 402 Objects (from TRACTION_PWR.eds)

| Object | Name                       | Description                  |
| ------ | -------------------------- | ---------------------------- |
| 0x6040 | Controlword                | State machine control        |
| 0x6041 | Statusword                 | Motor state feedback         |
| 0x6060 | Modes of operation         | Select control mode          |
| 0x6061 | Modes of operation display | Current mode                 |
| 0x6064 | Position actual value      | Encoder position             |
| 0x606C | Velocity actual value      | Current velocity             |
| 0x6077 | Torque actual value        | Current torque               |
| 0x607A | Target position            | Position setpoint (CSP mode) |
| 0x60FF | Target velocity            | Velocity setpoint (CSV mode) |
| 0x6071 | Target torque              | Torque setpoint (CST mode)   |
| 0x603F | Error code                 | Fault diagnosis              |

**Control Modes**:
- 0x01: Profile Position Mode (PP)
- 0x03: Profile Velocity Mode (PV)
- 0x04: Profile Torque Mode (TQ)
- 0x06: Homing Mode (HM)
- 0x08: Cyclic Sync Position (CSP)
- 0x09: Cyclic Sync Velocity (CSV)
- 0x0A: Cyclic Sync Torque (CST)

---

## Quick Reference Commands

```bash
# Build workspace
cd /home/ubuntu/ros_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch system
ros2 launch microphase_can_config microphase_can_config.launch.py

# Monitor CAN traffic
candump vcan0

# Check node status
ros2 node list
ros2 lifecycle list
ros2 lifecycle get /waveshare_bridge

# Monitor topics
ros2 topic list
ros2 topic echo /diagnostics
ros2 topic echo /front_left_traction/tpdo

# Send manual command (example)
ros2 topic pub --once /front_left_traction/rpdo canopen_interfaces/msg/COData \
  "{index: 0x6040, subindex: 0, data: 15}"
```

---

## Additional Resources

- **ros2_canopen Documentation**: https://ros-industrial.github.io/ros2_canopen/
- **CiA 402 Profile Spec**: https://www.can-cia.org/can-knowledge/canopen/cia402/
- **Waveshare USB-CAN-A Manual**: Check manufacturer documentation
- **Your EDS File**: `/home/ubuntu/ros_ws/src/microphase_can_config/config/can_config/eds/TRACTION_PWR.eds`
- **PDO Configuration**: `/home/ubuntu/ros_ws/src/microphase_can_config/config/PDO_CONFIGURATION.md`

---

## Success Criteria Checklist

- [ ] Bridge launches and activates successfully
- [ ] vcan0 interface created and operational
- [ ] CANopen device manager starts without errors
- [ ] Motor discovered on CAN bus (node ID 2)
- [ ] Heartbeat messages visible on candump
- [ ] Motor transitions through CiA 402 states
- [ ] Motor responds to velocity/position/torque commands
- [ ] No communication errors in diagnostics
- [ ] System runs stable for extended periods

---

**Last Updated**: October 22, 2025  
**Status**: Ready for hardware testing  
**Next Milestone**: Complete Phase 2 (Hardware Integration)
