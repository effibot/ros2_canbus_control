# CAN Message Reference for speed_control Package

This document maps Waveshare USB-CAN adapter messages to standard CAN messages for use with `cansend` command.

## Prerequisites

```bash
# Setup CAN interface (500 kbps to match speed_control default)
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Monitor CAN traffic
candump can0
```

## Message Conversion Format

**Waveshare Frame Structure:**
```
AA [C8+DLC] [ID_LSB] [ID_MSB] [DATA0-7] 55
```

**cansend Format:**
```
cansend can0 [CAN_ID]#[DATA_BYTES]
```

---

## 1. Initialization Sequence

### 1.1 NMT Start (Set Node to Operational)
**Waveshare:** `aa c2 00 00 01 01 00 00 00 00 00 00 55`
```bash
cansend can0 000#0101
```
- **Purpose:** Start CANopen node 1 (enter operational state)
- **CAN ID:** 0x000 (NMT command)
- **Data:** `01 01` (Start node, Node ID = 1)

### 1.2 Set Modes of Operation (Profile Velocity Mode)
**Waveshare:** `aa c8 01 06 2f 60 60 00 03 00 00 00 55`
```bash
cansend can0 601#2F60600003000000
```
- **Purpose:** Configure motor for velocity control mode
- **CAN ID:** 0x601 (SDO TX, Node 1)
- **Data:** 
  - `2F` = SDO write 1 byte
  - `60 60` = Index 0x6060 (Modes of operation)
  - `00` = Subindex 0
  - `03` = Profile Velocity Mode
  - `00 00 00` = Reserved

---

## 2. Enable Motor Sequence

### 2.1 Read Statusword (Check Initial State)
**Waveshare:** `aa c8 01 06 40 41 60 00 00 00 00 00 55`
```bash
cansend can0 601#4041600000000000
```
- **Purpose:** Read current motor state
- **Expected Response:** `581#4B41600000XXYY` where XXYY is statusword

### 2.2 Shutdown Command
**Waveshare:** `aa c8 01 06 2b 40 60 00 06 00 00 00 55`
```bash
cansend can0 601#2B40600006000000
```
- **Purpose:** Send controlword 0x0006 (Shutdown)
- **Data:**
  - `2B` = SDO write 2 bytes
  - `40 60` = Index 0x6040 (Controlword)
  - `00` = Subindex 0
  - `06 00` = 0x0006 (Shutdown command)

### 2.3 Switch On Command
**Waveshare:** `aa c8 01 06 2b 40 60 00 07 00 00 00 55`
```bash
cansend can0 601#2B40600007000000
```
- **Purpose:** Send controlword 0x0007 (Switch On)
- **Data:** `07 00` = 0x0007 (Switch On command)

### 2.4 Enable Operation Command
**Waveshare:** `aa c8 01 06 2b 40 60 00 0f 00 00 00 55`
```bash
cansend can0 601#2B4060000F000000
```
- **Purpose:** Send controlword 0x000F (Enable Operation)
- **Data:** `0F 00` = 0x000F (Enable Operation command)

### 2.5 Verify Enabled State
**Waveshare:** `aa c8 01 06 40 41 60 00 00 00 00 00 55`
```bash
cansend can0 601#4041600000000000
```
- **Purpose:** Verify motor is in "Operation Enabled" state
- **Expected Response:** `581#4B41600000XXYY` where XXYY should be 0x0027 or 0x0037

---

## 3. Speed Control Commands

### 3.1 Set Target Velocity (SDO Method)
**Format:** `aa c8 01 06 23 ff 60 00 [VEL_BYTES] 55`

#### Example: 100 RPM
**Speed Calculation:**
```
internal_speed = (RPM * FEED_CONSTANT * REDUCTION_FACTOR) / 60
internal_speed = (100 * 10000 * 100) / 60 = 1666666 = 0x00196C0A
```

**Waveshare:** `aa c8 01 06 23 ff 60 00 0a 6c 19 00 55`
```bash
cansend can0 601#23FF60000A6C1900
```
- **Data:**
  - `23` = SDO write 4 bytes
  - `FF 60` = Index 0x60FF (Target velocity)
  - `00` = Subindex 0
  - `0A 6C 19 00` = 1666666 (little-endian)

### 3.2 Set Target Velocity (PDO Method - Faster)
**Format:** `aa c4 01 02 [VEL_BYTES] 55`

#### Example: 100 RPM
**Waveshare:** `aa c4 01 02 0a 6c 19 00 55`
```bash
cansend can0 201#0A6C1900
```
- **CAN ID:** 0x201 (RPDO1, Node 1 = 0x200 + 1)
- **Data:** `0A 6C 19 00` = 1666666 (little-endian)

---

## 4. Common Speed Values

### Speed Conversion Formula
```
internal_speed = (RPM * FEED_CONSTANT * REDUCTION_FACTOR) / 60
```

**Typical Parameters:**
- FEED_CONSTANT = 10000 (encoder pulses per revolution)
- REDUCTION_FACTOR = 100 (gear ratio)

### Pre-calculated Speed Commands (PDO Method)

#### Stop (0 RPM)
```bash
cansend can0 201#00000000
```

#### 10 RPM (166666 = 0x00028B0A)
```bash
cansend can0 201#0A8B0200
```

#### 50 RPM (833333 = 0x000CB64D)
```bash
cansend can0 201#4DB60C00
```

#### 100 RPM (1666666 = 0x00196C0A)
```bash
cansend can0 201#0A6C1900
```

#### 200 RPM (3333333 = 0x0032D814)
```bash
cansend can0 201#14D83200
```

#### 500 RPM (8333333 = 0x007F1A55)
```bash
cansend can0 201#551A7F00
```

#### -50 RPM (Reverse) (-833333 = 0xFFF349B3)
```bash
cansend can0 201#B34993FF
```

#### -100 RPM (Reverse) (-1666666 = 0xFFE693F6)
```bash
cansend can0 201#F693E6FF
```

---

## 5. Diagnostic Commands

### 5.1 Read Statusword (0x6041)
```bash
cansend can0 601#4041600000000000
```
**Expected Response:** `581#4B41600000XXYY`

**Statusword Values:**
- `0x0000`: Not ready to switch on
- `0x0040`: Switch on disabled
- `0x0021`: Ready to switch on
- `0x0023`: Switched on
- `0x0027`: Operation enabled
- `0x0037`: Operation enabled (target reached)
- `0x0008`: Fault

### 5.2 Read Actual Velocity (0x606C)
```bash
cansend can0 601#406C600000000000
```
**Expected Response:** `581#4B6C600000XXYYZZ` where XXYYZZ is current velocity

### 5.3 Read Position Actual Value (0x6064)
```bash
cansend can0 601#4064600000000000
```
**Expected Response:** `581#4B64600000XXYYZZ` where XXYYZZ is current position

### 5.4 Read Modes of Operation Display (0x6061)
```bash
cansend can0 601#4061600000000000
```
**Expected Response:** `581#4F61600000XX` where XX is current mode
- `03` = Profile Velocity Mode
- `01` = Profile Position Mode

### 5.5 Read Error Register (0x1001)
```bash
cansend can0 601#4001100000000000
```
**Expected Response:** `581#4F01100000XX` where XX is error code

---

## 6. Emergency Stop

### Quick Stop (Immediate)
```bash
cansend can0 601#2B40600002000000
```
- **Controlword:** 0x0002 (Quick Stop)

### Disable Operation
```bash
cansend can0 601#2B40600007000000
```
- **Controlword:** 0x0007 (Disable Operation, maintain "Switched On")

### Full Shutdown
```bash
cansend can0 601#2B40600006000000
```
- **Controlword:** 0x0006 (Shutdown)

---

## 7. Complete Test Sequence Script

Save this as `test_can_messages.sh`:

```bash
#!/bin/bash
# Complete CAN test sequence for speed_control equivalent

# Configuration
CAN_INTERFACE="can0"
NODE_ID="1"

echo "=== CAN Message Test Sequence ==="
echo "Interface: $CAN_INTERFACE"
echo "Node ID: $NODE_ID"
echo ""

# Setup CAN interface
echo "1. Setting up CAN interface..."
sudo ip link set $CAN_INTERFACE type can bitrate 500000
sudo ip link set $CAN_INTERFACE up
sleep 1

# Start node
echo "2. Starting CANopen node..."
cansend $CAN_INTERFACE 000#0101
sleep 0.5

# Set velocity mode
echo "3. Setting Profile Velocity Mode..."
cansend $CAN_INTERFACE 601#2F60600003000000
sleep 0.2

# Enable sequence
echo "4. Enabling motor (Shutdown -> Switch On -> Enable Operation)..."
cansend $CAN_INTERFACE 601#2B40600006000000  # Shutdown
sleep 0.1
cansend $CAN_INTERFACE 601#2B40600007000000  # Switch On
sleep 0.1
cansend $CAN_INTERFACE 601#2B4060000F000000  # Enable Operation
sleep 0.5

# Read statusword
echo "5. Reading Statusword..."
cansend $CAN_INTERFACE 601#4041600000000000
sleep 0.2

# Send speed commands
echo "6. Testing speed commands..."
echo "   - Setting 50 RPM..."
cansend $CAN_INTERFACE 201#4DB60C00
sleep 2

echo "   - Setting 100 RPM..."
cansend $CAN_INTERFACE 201#0A6C1900
sleep 2

echo "   - Setting 200 RPM..."
cansend $CAN_INTERFACE 201#14D83200
sleep 2

echo "   - Setting 0 RPM (Stop)..."
cansend $CAN_INTERFACE 201#00000000
sleep 2

echo "   - Setting -50 RPM (Reverse)..."
cansend $CAN_INTERFACE 201#B34993FF
sleep 2

echo "   - Setting 0 RPM (Stop)..."
cansend $CAN_INTERFACE 201#00000000
sleep 1

# Disable
echo "7. Disabling motor..."
cansend $CAN_INTERFACE 601#2B40600006000000
sleep 0.5

echo ""
echo "=== Test sequence complete ==="
```

Make it executable:
```bash
chmod +x test_can_messages.sh
```

---

## 8. Monitoring and Debugging

### Monitor all CAN traffic
```bash
candump can0
```

### Monitor with timestamps
```bash
candump -ta can0
```

### Filter specific CAN IDs
```bash
# Only show SDO responses from node 1
candump can0,581:7FF

# Only show TPDO messages from node 1
candump can0,181:7FF,281:7FF,381:7FF,481:7FF
```

### Log CAN traffic to file
```bash
candump -l can0
```

### Replay CAN messages from log
```bash
canplayer -I candump-2024-10-21_*.log
```

---

## 9. Troubleshooting

### No response to commands
1. Check CAN interface is up:
   ```bash
   ip link show can0
   ```

2. Verify bitrate matches motor driver (500 kbps):
   ```bash
   ip -details link show can0
   ```

3. Check for CAN errors:
   ```bash
   ip -s link show can0
   ```

### Motor not moving
1. Read statusword to check state:
   ```bash
   cansend can0 601#4041600000000000
   ```

2. Check for errors:
   ```bash
   cansend can0 601#4001100000000000
   ```

3. Verify mode is set correctly:
   ```bash
   cansend can0 601#4061600000000000
   ```

### Reset motor driver
```bash
# Send NMT Reset Node command
cansend can0 000#8101
sleep 2
# Restart initialization sequence
./test_can_messages.sh
```

---

## 10. Parameter Customization

If your motor has different parameters, adjust the speed calculation:

### Custom Feed Constant and Reduction Factor
```python
#!/usr/bin/env python3
# Speed calculation helper

FEED_CONSTANT = 10000  # Change this to your encoder resolution
REDUCTION_FACTOR = 100  # Change this to your gear ratio

def rpm_to_internal(rpm):
    internal = int((rpm * FEED_CONSTANT * REDUCTION_FACTOR) / 60)
    return internal

def internal_to_hex(internal):
    # Convert to 32-bit signed integer, little-endian
    if internal < 0:
        internal = (1 << 32) + internal
    hex_str = f"{internal:08X}"
    # Reverse byte order for little-endian
    return ''.join([hex_str[i:i+2] for i in range(6, -1, -2)])

# Example usage
rpm = 100
internal = rpm_to_internal(rpm)
hex_data = internal_to_hex(internal)
print(f"{rpm} RPM = {internal} internal = {hex_data}")
print(f"cansend can0 201#{hex_data}")
```

---

## Notes

1. **Node ID:** All examples assume Node ID = 1. For different node IDs:
   - SDO TX: 0x600 + node_id
   - SDO RX: 0x580 + node_id
   - RPDO1: 0x200 + node_id
   - TPDO1: 0x180 + node_id

2. **Byte Order:** CAN uses little-endian for multi-byte values

3. **Timing:** Add delays between commands (100-500ms) to allow driver processing

4. **Safety:** Always ensure emergency stop procedures are in place before testing

5. **PDO vs SDO:** Use PDO (0x201) for real-time speed control, SDO (0x601) for configuration