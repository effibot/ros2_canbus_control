# ROS2 CAN Bus Control System - AI Coding Instructions

## Project Overview

ROS2 middleware for industrial motor control using Waveshare USB-CAN-A adapters. The system bridges USB-CAN hardware (`/dev/ttyUSB*`) to Linux SocketCAN, integrating with CANOpen for MICROPHASE TRAC_PWR servo drives (DS-402 profile).

**Status**: Waveshare C++ library complete (132 tests, 100%), SocketCAN bridge operational, CANOpen lifecycle nodes active, targeting 4-motor traction control with PDO/SDO.

**Critical Architecture**: `USB Hardware → Serial Protocol → SocketCAN Bridge → CANOpen Stack → ROS2 Control`

## Core Components

1. **Waveshare C++ Library** (`src/ros2_waveshare/lib/waveshare_cpp/`) - Git submodule
   - State-First Architecture with CRTP patterns
   - Serial I/O via POSIX termios2 (up to 2Mbps)
   - Thread-safe: 3-mutex pattern (state/write/read)
   - Frame types: FixedFrame (20B), VariableFrame (5-15B), ConfigFrame (20B)
   
2. **ROS2 Lifecycle Bridge** (`src/ros2_waveshare/`) - Package: `ros2_waveshare`
   - `CanopenLifeCycleNode`: Manages SocketCAN bridge lifecycle
   - Wraps `SocketCANBridge` from waveshare_cpp library
   - Launch: `bridge_bringup.launch.py` with `bridge_params.yaml`
   
3. **CANOpen Configuration** (`src/microphase_can_config/`) - Package: `microphase_can_config`
   - Bus config: `bus.yml` defines 4 nodes (IDs 2-5) + master (ID 1)
   - EDS file: `config/can_config/eds/TRACTION_PWR.eds` (DS-402 compliant)
   - Unified launch: `microphase_can_config.launch.py` starts bridge + CANOpen stack
   
4. **Speed Control** (`src/speed_control/`) - Package: `speed_control`
   - High-level motor control with custom message types
   - Legacy implementation - PDO/SDO handling being migrated to CANOpen stack

## Critical Development Patterns

### State-First Architecture (Waveshare Library)

**Core Principle**: Frames hold state, buffers generated on-demand
```cpp
// CORRECT: State-first pattern
FixedFrame frame;
frame.set_id(0x123);              // Modifies data_state_.can_id
frame.set_data({0x11, 0x22});     // Modifies data_state_.data
auto buffer = frame.serialize();   // Generates 20-byte protocol buffer NOW

// WRONG: Don't manipulate buffers directly
// Frame does NOT expose internal buffers for modification
```

**CRTP Hierarchy**: `CoreInterface<T>` → `DataInterface<T>` → `FixedFrame/VariableFrame`
- Interfaces use `derived()` to call `impl_serialize()`, `impl_deserialize()` in concrete frames
- See `src/ros2_waveshare/lib/waveshare_cpp/.github/copilot-instructions.md` for full details

### ROS2 Workspace Management

**Build Commands** (always from workspace root):
```bash
# Full build with dependency install
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Incremental build for specific packages
colcon build --symlink-install --packages-select ros2_waveshare microphase_can_config

# Source after every build
source install/setup.bash
```

**Environment Setup**: 
- Development currently in WSL (will move to Docker container)
- Use `scripts/build_ws.sh` for automated builds (expects `WORKSPACE` env variable)
- Workspace root varies by environment - use `$(pwd)` for relative paths in scripts

### SocketCAN Bridge Setup

**Required One-Time Setup**:
```bash
# 1. Load kernel modules immediately
sudo modprobe can can-raw can-bcm vcan

# Persist modules at boot (create /etc/modules-load.d/vcan.conf)
echo "can" | sudo tee /etc/modules-load.d/can.conf
echo "can-raw" | sudo tee -a /etc/modules-load.d/can.conf
echo "can-bcm" | sudo tee -a /etc/modules-load.d/can.conf
echo "vcan" | sudo tee /etc/modules-load.d/vcan.conf

# 2. Create virtual CAN interface (for testing without hardware)
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 3. USB device permissions (persist via udev rule)
# Create /etc/udev/rules.d/50-myusb.rules:
KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"
# Then: sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Bridge Operation**: Node creates `/dev/ttyUSB0` ↔ `vcan0` bidirectional forwarding
- Configure: Reads YAML params, validates USB device
- Activate: Opens serial port, creates SocketCAN socket, starts forwarding threads
- Deactivate: Stops threads, closes connections

### CANOpen Device Integration

**DS-402 Object Dictionary** (from `TRACTION_PWR.eds`):
```
0x6040 - Control Word (16-bit, RW, PDO)    # State machine control
0x6041 - Status Word (16-bit, RO, PDO)     # Drive state feedback
0x6060 - Mode of Operation (8-bit, RW)     # 1=PP, 3=PV, 4=TQ, etc.
0x607A - Target Position (32-bit, RW, PDO)
0x60FF - Target Velocity (32-bit, RW, PDO)
0x606C - Velocity Actual (32-bit, RO, PDO)
0x6064 - Position Actual (32-bit, RO, PDO)
```

**Launch System**:
```bash
# Full system: Bridge + CANOpen stack
ros2 launch microphase_can_config microphase_can_config.launch.py

# Verify lifecycle state
ros2 lifecycle get /waveshare_bridge  # Should be: active

# Monitor diagnostics
ros2 topic echo /diagnostics

# List CANOpen topics (once stack starts)
ros2 topic list | grep -E 'rpdo|tpdo|sdo'
```

### Lifecycle Node Transitions

**State Machine**: `unconfigured` → `inactive` → `active` → `inactive` → `cleanup` → `shutdown`

**Transitions in `CanopenLifeCycleNode`**:
- `on_configure`: Load YAML params, create `BridgeConfig`, validate USB device exists
- `on_activate`: Instantiate `SocketCANBridge`, open serial + socket, start forwarding
- `on_deactivate`: Stop bridge threads, close connections (USB still accessible)
- `on_cleanup`: Destroy bridge object, reset params
- `on_shutdown`: Final cleanup from any state

**Launch File Automation**: `bridge_bringup.launch.py` uses `auto_configure` and `auto_activate` args to trigger transitions automatically.

## Debugging Workflows

**Testing Strategy**: Manual execution and debugging only - no automated integration tests currently implemented.

### Hardware-Independent Testing
```bash
# Waveshare library unit tests (132 tests, no hardware needed)
cd src/ros2_waveshare/lib/waveshare_cpp
cmake -B build && cmake --build build
ctest --test-dir build --output-on-failure

# SocketCAN simulation (without USB adapter)
# Terminal 1: Monitor vcan0
candump vcan0

# Terminal 2: Send test frames
cansend vcan0 "123#DEADBEEF"

# Terminal 3: Bridge with mock USB (if adapted for testing)
./build/scripts/wave_reader /dev/ttyUSB0  # Will timeout if no device
```

### Hardware Debugging
```bash
# 1. Verify USB device presence
ls -l /dev/ttyUSB*  # Should show ttyUSB0 (or similar)

# 2. Check permissions
sudo chmod 666 /dev/ttyUSB0  # Temporary fix

# 3. Test direct USB communication (wave_reader from waveshare_cpp)
cd src/ros2_waveshare/lib/waveshare_cpp/build
./scripts/wave_reader /dev/ttyUSB0

# 4. Run bridge standalone (non-ROS)
./scripts/wave_bridge -d /dev/ttyUSB0 -i vcan0 -b 2000000 -c 1000000

# 5. Monitor bridge in ROS2
ros2 topic echo /diagnostics  # Shows TX/RX counts, errors
ros2 lifecycle get /waveshare_bridge
```

**Serial Port Exclusivity**: Only one process can open `/dev/ttyUSB0`. If bridge is running, `wave_writer` auto-detects and falls back to SocketCAN mode.

### CANOpen Stack Debugging
```bash
# Check node discovery
ros2 topic echo /device_manager_node/heartbeat  # CANOpen heartbeat messages

# SDO read example (once integrated)
ros2 service call /sdo_read canopen_interfaces/srv/CORead "{node_id: 2, index: 0x6041, subindex: 0}"

# PDO monitoring
ros2 topic echo /tpdo1  # Check for position/velocity feedback
```

## File Organization

**Configuration**: All YAML/launch files in `*/config/` and `*/launch/` subdirectories
- `bridge_params.yaml`: Serial baud (2Mbps), CAN baud (500k/1Mbps), interface names
- `bus.yml`: CANOpen node IDs, EDS paths, driver types
- EDS files: Device object dictionaries (don't edit unless hardware spec changes)

**Source Layout**:
- `include/<package>/`: Public headers (installed)
- `src/`: Implementation files (.cpp)
- `scripts/`: Standalone utilities (wave_reader, wave_writer, etc.)
- `doc/`: Architecture diagrams (Mermaid), integration guides

**Git Submodules**: `waveshare_cpp` is a submodule. Update with:
```bash
git submodule update --remote src/ros2_waveshare/lib/waveshare_cpp
# Then commit the submodule pointer change
```

## Common Issues

**Build Failures**:
- Missing ROS2 deps: `rosdep install --from-paths src --ignore-src -r -y`
- Submodule not initialized: `git submodule update --init --recursive`
- Stale CMake cache: `rm -rf build/ install/` then rebuild

**Runtime Errors**:
- "Device not found": Check `/dev/ttyUSB*` exists, verify udev rules
- "Permission denied": Run `sudo chmod 666 /dev/ttyUSB0` or fix udev rules
- "CAN interface not found": Verify `ip link show vcan0` shows interface UP
- Bridge won't activate: Check serial/CAN baud rates in YAML match hardware config

**CANOpen Issues**:
- No motor response: Verify CAN baud rate (1Mbps typical), node IDs (2-5) match physical DIP switches
- PDO not received: Check EDS PDO mappings match motor configuration

## DevContainer Support

**Multi-GPU Configurations**: `.devcontainer/intel-gpu/` and `.devcontainer/nvidia-gpu/`
- Supports Xorg and Wayland
- Use `.devcontainer/setup-devcontainer.sh` to select configuration
- Each config includes ROS2 Humble, CAN tools, hardware acceleration

## Integration with External Systems

**ros2_canopen Stack** (Lely-based):
- Expects SocketCAN interface (bridge provides this)
- Loads EDS files at launch, generates DCF files
- Manages NMT state machine, SDO configuration, PDO subscriptions

**ros2_control** (Future):
- Hardware interface will communicate via CANOpen topics/services
- Control loop runs at ROS2 rate, PDOs provide real-time feedback

**External CAN Tools**: Standard Linux utilities work via SocketCAN:
- `candump`, `cansend`, `cangen`, `cansequence` for testing
- `can-utils` package provides full toolchain