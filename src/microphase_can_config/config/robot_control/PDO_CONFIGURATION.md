# TRACTION_PWR PDO Configuration Guide

This document explains the PDO (Process Data Object) configuration in `bus.yml` for the TRACTION_PWR motor controller.

## Overview

The configuration includes:
- **24 SDO parameters** for initialization
- **4 RPDOs** (Receive PDOs - Controller → Device)
- **4 TPDOs** (Transmit PDOs - Device → Controller)

## SDO Configuration (Service Data Objects)

SDOs are used for one-time configuration during device initialization. These parameters are written once at startup.

### Motion Profile Parameters

| Index  | Sub | Parameter               | Value | Purpose                 |
| ------ | --- | ----------------------- | ----- | ----------------------- |
| 0x6081 | 0   | Profile velocity        | 10000 | Default target velocity |
| 0x6083 | 0   | Profile acceleration    | 5000  | Acceleration rate       |
| 0x6084 | 0   | Profile deceleration    | 5000  | Deceleration rate       |
| 0x6085 | 0   | Quick stop deceleration | 10000 | Emergency stop rate     |
| 0x607F | 0   | Max profile velocity    | 50000 | Velocity limit          |
| 0x60C5 | 0   | Max acceleration        | 20000 | Acceleration limit      |
| 0x60C6 | 0   | Max deceleration        | 20000 | Deceleration limit      |

### Position Limits

| Index  | Sub | Parameter          | Value       | Purpose              |
| ------ | --- | ------------------ | ----------- | -------------------- |
| 0x607D | 1   | Min position limit | -2147483648 | Software limit (min) |
| 0x607D | 2   | Max position limit | 2147483647  | Software limit (max) |

### Current Limit

| Index  | Sub | Parameter   | Value | Purpose               |
| ------ | --- | ----------- | ----- | --------------------- |
| 0x6073 | 0   | Max Current | 1000  | Maximum motor current |

### Following Error Configuration

| Index  | Sub | Parameter               | Value | Purpose                    |
| ------ | --- | ----------------------- | ----- | -------------------------- |
| 0x6065 | 0   | Following error window  | 5000  | Max allowed tracking error |
| 0x6066 | 0   | Following error timeout | 1000  | Error timeout (ms)         |
| 0x6067 | 0   | Position window         | 100   | Target reached threshold   |
| 0x6068 | 0   | Position timeout        | 500   | Position timeout (ms)      |

### Gear Ratio (Feed Constant)

| Index  | Sub | Parameter               | Value | Purpose                |
| ------ | --- | ----------------------- | ----- | ---------------------- |
| 0x6092 | 1   | Feed constant numerator | 65536 | Gear ratio numerator   |
| 0x6092 | 2   | Shaft revolutions denom | 1     | Gear ratio denominator |

**Note**: Gear ratio = 65536:1 by default. Adjust for your specific application.

### Homing Configuration

| Index  | Sub | Parameter                      | Value | Purpose               |
| ------ | --- | ------------------------------ | ----- | --------------------- |
| 0x6098 | 0   | Homing method                  | 35    | Homing procedure code |
| 0x6099 | 1   | Speed during search for switch | 1000  | Homing search speed   |
| 0x6099 | 2   | Speed during search for zero   | 500   | Homing zero speed     |
| 0x609A | 0   | Homing acceleration            | 2000  | Homing accel rate     |

### Interpolation Time Period

| Index  | Sub | Parameter  | Value | Purpose            |
| ------ | --- | ---------- | ----- | ------------------ |
| 0x60C2 | 1   | Time units | 2     | 2 = milliseconds   |
| 0x60C2 | 2   | Time index | -3    | Exponent (-3 = ms) |

## RPDO Configuration (Receive PDOs)

RPDOs carry commands from the controller to the device. Data flows: **Controller → Device**

### RPDO1 (0x1600) - Control and Position Commands

**Transmission**: Synchronous (0x01) - Sent every SYNC  
**Total Size**: 10 bytes  
**Purpose**: Primary motion control

| Byte Offset | Index  | Sub | Parameter        | Type     | Size | Description           |
| ----------- | ------ | --- | ---------------- | -------- | ---- | --------------------- |
| 0-1         | 0x6040 | 0   | Control word     | uint16_t | 2    | State machine control |
| 2-5         | 0x607A | 0   | Target position  | int32_t  | 4    | Position setpoint     |
| 6-9         | 0x6081 | 0   | Profile velocity | uint32_t | 4    | Target velocity       |

**Use Case**: Position mode (CSP, PP) with velocity override

### RPDO2 (0x1601) - Velocity and Mode Control

**Transmission**: Synchronous (0x01)  
**Total Size**: 9 bytes  
**Purpose**: Velocity control and mode switching

| Byte Offset | Index  | Sub | Parameter            | Type     | Size | Description           |
| ----------- | ------ | --- | -------------------- | -------- | ---- | --------------------- |
| 0           | 0x6060 | 0   | Mode of operation    | int8_t   | 1    | Operating mode select |
| 1-4         | 0x60FF | 0   | Target velocity      | int32_t  | 4    | Velocity setpoint     |
| 5-8         | 0x6083 | 0   | Profile acceleration | uint32_t | 4    | Accel rate override   |

**Use Case**: Velocity mode (CSV, PV) with dynamic acceleration

### RPDO3 (0x1602) - Motion Profile Parameters

**Transmission**: Asynchronous (0xFF) - Event-driven  
**Total Size**: 8 bytes  
**Purpose**: Dynamic motion profile adjustment

| Byte Offset | Index  | Sub | Parameter               | Type     | Size | Description         |
| ----------- | ------ | --- | ----------------------- | -------- | ---- | ------------------- |
| 0-3         | 0x6084 | 0   | Profile deceleration    | uint32_t | 4    | Decel rate          |
| 4-7         | 0x6085 | 0   | Quick stop deceleration | uint32_t | 4    | Emergency stop rate |

**Use Case**: Runtime motion profile tuning

### RPDO4 (0x1603) - Digital Outputs

**Transmission**: Asynchronous (0xFF)  
**Total Size**: 8 bytes  
**Purpose**: Digital I/O control

| Byte Offset | Index  | Sub | Parameter                | Type     | Size | Description         |
| ----------- | ------ | --- | ------------------------ | -------- | ---- | ------------------- |
| 0-3         | 0x60FE | 1   | Physical digital outputs | uint32_t | 4    | Output state bits   |
| 4-7         | 0x60FE | 2   | Digital output bit mask  | uint32_t | 4    | Enable/disable mask |

**Use Case**: Brake control, auxiliary outputs

## TPDO Configuration (Transmit PDOs)

TPDOs carry feedback from the device to the controller. Data flows: **Device → Controller**

### TPDO1 (0x1A00) - Status and Position Feedback

**Transmission**: Synchronous (0x01) - Every SYNC  
**Total Size**: 10 bytes  
**Purpose**: Primary feedback for motion control

| Byte Offset | Index  | Sub | Parameter             | Type     | Size | Description      |
| ----------- | ------ | --- | --------------------- | -------- | ---- | ---------------- |
| 0-1         | 0x6041 | 0   | Status word           | uint16_t | 2    | Device state     |
| 2-5         | 0x6064 | 0   | Position actual value | int32_t  | 4    | Current position |
| 6-9         | 0x606C | 0   | Velocity actual value | int32_t  | 4    | Current velocity |

**Use Case**: Real-time position/velocity monitoring, state machine status

### TPDO2 (0x1A01) - Operation Mode and Error Status

**Transmission**: Synchronous (0x01)  
**Total Size**: 8 bytes  
**Purpose**: Operating mode and error monitoring

| Byte Offset | Index  | Sub | Parameter                 | Type     | Size | Description             |
| ----------- | ------ | --- | ------------------------- | -------- | ---- | ----------------------- |
| 0           | 0x6061 | 0   | Mode of operation display | int8_t   | 1    | Active operating mode   |
| 1           | 0x1001 | 0   | Error register            | uint8_t  | 1    | Error status bits       |
| 2-5         | 0x60F4 | 0   | Following error actual    | int32_t  | 4    | Position tracking error |
| 6-7         | 0x6073 | 0   | Max Current               | uint16_t | 2    | Current limit status    |

**Use Case**: Error detection, mode verification, current monitoring

### TPDO3 (0x1A02) - Digital I/O Status

**Transmission**: Asynchronous (0xFF) - Event-driven  
**Total Size**: 8 bytes  
**Purpose**: Digital I/O and manufacturer status

| Byte Offset | Index  | Sub | Parameter                    | Type     | Size | Description            |
| ----------- | ------ | --- | ---------------------------- | -------- | ---- | ---------------------- |
| 0-3         | 0x60FD | 0   | Digital inputs               | uint32_t | 4    | Input state bits       |
| 4-7         | 0x1002 | 0   | Manufacturer Status Register | uint32_t | 4    | Vendor-specific status |

**Use Case**: Limit switch monitoring, auxiliary sensor inputs, vendor diagnostics

### TPDO4 (0x1A03) - Cam and Extended Status

**Transmission**: Asynchronous (0xFF)  
**Total Size**: 2 bytes  
**Enabled**: False (disabled by default)  
**Purpose**: Cam functionality and polarity status

| Byte Offset | Index  | Sub | Parameter          | Type    | Size | Description         |
| ----------- | ------ | --- | ------------------ | ------- | ---- | ------------------- |
| 0           | 0x6300 | 1   | Cam state register | uint8_t | 1    | Cam output state    |
| 1           | 0x607E | 0   | Polarity           | uint8_t | 1    | Direction inversion |

**Use Case**: Electronic cam applications (enable when needed)

## PDO Size Limits

CANopen PDO size constraints:
- **Maximum PDO payload**: 8 bytes (CAN 2.0A/B)
- **All configured PDOs comply** with this limit

## Transmission Types

| Type  | Value | Behavior                              | Use Case                     |
| ----- | ----- | ------------------------------------- | ---------------------------- |
| SYNC  | 0x01  | Sent synchronously every SYNC message | Real-time control/feedback   |
| ASYNC | 0xFF  | Sent on change of data or event       | Configuration, status events |

## Usage Recommendations

### For Position Control (Profile Position Mode)
- **Use**: RPDO1 (control word + target position)
- **Monitor**: TPDO1 (status word + position actual)
- **Sync Period**: 10ms (as configured in master settings)

### For Velocity Control (Profile Velocity Mode)
- **Use**: RPDO2 (mode + target velocity)
- **Monitor**: TPDO1 (status word + velocity actual)

### For Error Monitoring
- **Monitor**: TPDO2 (error register + following error)
- **Check**: Error register bits (0x1001)

### For Digital I/O
- **Control**: RPDO4 (digital outputs)
- **Monitor**: TPDO3 (digital inputs)

## Customization Guide

### Adjusting SDO Values

Edit `bus.yml` under the `sdo:` section. Example:

```yaml
# Increase velocity limit
- {index: 0x607F, sub_index: 0, value: 100000}  # Max profile velocity

# Tighter position window
- {index: 0x6067, sub_index: 0, value: 50}      # Position window
```

### Modifying PDO Mappings

To change what data is transmitted in a PDO:

1. **Disable the PDO** during startup (COB-ID bit 31)
2. **Clear the mapping**: Set mapping count to 0
3. **Add new mappings**: Configure desired objects
4. **Enable the PDO**: Clear COB-ID bit 31

Example (not directly in bus.yml, requires SDO writes):
```yaml
sdo:
  # Disable TPDO1
  - {index: 0x1800, sub_index: 1, value: 0x80000180}
  # Clear mapping count
  - {index: 0x1A00, sub_index: 0, value: 0}
  # Add custom mapping (example: only status word + position)
  - {index: 0x1A00, sub_index: 1, value: 0x60410010}  # Status word (16 bits)
  - {index: 0x1A00, sub_index: 2, value: 0x60640020}  # Position (32 bits)
  - {index: 0x1A00, sub_index: 0, value: 2}           # 2 objects mapped
  # Enable TPDO1
  - {index: 0x1800, sub_index: 1, value: 0x00000180}
```

### Enabling TPDO4 (Cam Functionality)

If you need cam functionality:

```yaml
tpdo:
  - index: 0x1A03
    cob_id: "auto"
    transmission: 0xFF
    enabled: true  # Change from false to true
    mapping:
      - {index: 0x6300, sub_index: 1}
      - {index: 0x607E, sub_index: 0}
```

## Troubleshooting

### PDO Not Updating
- Check if SYNC messages are being sent (sync_period in master config)
- Verify COB-ID is not disabled (bit 31 = 0 for enabled)
- Ensure device is in OPERATIONAL state

### Data Mismatch
- Verify data types match between mapping and application
- Check byte order (CANopen uses little-endian)
- Confirm PDO mapping matches EDS file definitions

### Performance Issues
- Reduce SYNC period if too slow (increase frequency)
- Use asynchronous TPDOs for status that changes infrequently
- Minimize number of enabled PDOs

## Reference Documents

- **EDS File**: `TRACTION_PWR.eds` - Full device description
- **PDO Table**: `TRACTION_PWR_table.md` - Object dictionary reference
- **CiA 402 Spec**: CANopen device profile for drives and motion control
- **CiA 301 Spec**: CANopen application layer and communication profile
