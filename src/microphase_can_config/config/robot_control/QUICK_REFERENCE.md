# TRACTION_PWR Bus Configuration Quick Reference

## 📋 Configuration Summary

| Category           | Count | Details                |
| ------------------ | ----- | ---------------------- |
| **SDO Parameters** | 24    | Initialization values  |
| **RPDOs** (RX)     | 4     | Controller → Device    |
| **TPDOs** (TX)     | 4     | Device → Controller    |
| **SYNC Period**    | 10ms  | Master synchronization |

## 🔧 SDO Configuration Overview

```yaml
Motion Profile: 6081, 6083-6085, 607F, 60C5-60C6
Position Limits: 607D.1, 607D.2
Current Limit: 6073
Following Error: 6065-6068
Gear Ratio: 6092.1-.2
Homing: 6098-609A
Interpolation: 60C2.1-.2
```

## 📤 RPDO Mappings (Controller → Device)

### RPDO1 (0x1600) - Control & Position [10 bytes, SYNC]

```yaml
0x6040.0  Control word           uint16_t  (2 bytes)
0x607A.0  Target position        int32_t   (4 bytes)
0x6081.0  Profile velocity       uint32_t  (4 bytes)
```

**Use**: Position mode control

### RPDO2 (0x1601) - Velocity & Mode [9 bytes, SYNC]

```yaml
0x6060.0  Mode of operation      int8_t    (1 byte)
0x60FF.0  Target velocity        int32_t   (4 bytes)
0x6083.0  Profile acceleration   uint32_t  (4 bytes)
```

**Use**: Velocity mode control

### RPDO3 (0x1602) - Profile Parameters [8 bytes, ASYNC]

```yaml
0x6084.0  Profile deceleration   uint32_t  (4 bytes)
0x6085.0  Quick stop decel       uint32_t  (4 bytes)
```

**Use**: Dynamic motion tuning

### RPDO4 (0x1603) - Digital I/O [8 bytes, ASYNC]

```yaml
0x60FE.1  Physical outputs       uint32_t  (4 bytes)
0x60FE.2  Output bit mask        uint32_t  (4 bytes)
```

**Use**: Brake control, auxiliary outputs

## 📥 TPDO Mappings (Device → Controller)

### TPDO1 (0x1A00) - Status & Feedback [10 bytes, SYNC]

```yaml
0x6041.0  Status word            uint16_t  (2 bytes)
0x6064.0  Position actual        int32_t   (4 bytes)
0x606C.0  Velocity actual        int32_t   (4 bytes)
```

**Use**: Real-time motion feedback

### TPDO2 (0x1A01) - Mode & Error Status [8 bytes, SYNC]

```yaml
0x6061.0  Mode display           int8_t    (1 byte)
0x1001.0  Error register         uint8_t   (1 byte)
0x60F4.0  Following error        int32_t   (4 bytes)
0x6073.0  Max Current            uint16_t  (2 bytes)
```

**Use**: Error monitoring, mode verification

### TPDO3 (0x1A02) - Digital I/O [8 bytes, ASYNC]

```yaml
0x60FD.0  Digital inputs         uint32_t  (4 bytes)
0x1002.0  Manufacturer status    uint32_t  (4 bytes)
```

**Use**: Sensor inputs, vendor diagnostics

### TPDO4 (0x1A03) - Cam Status [2 bytes, ASYNC, DISABLED]

```yaml
0x6300.1  Cam state register     uint8_t   (1 byte)
0x607E.0  Polarity               uint8_t   (1 byte)
```

**Use**: Electronic cam (enable if needed)

## 🎯 Typical Use Cases

### Position Control (PP Mode)

```yaml
Send: RPDO1 (Control word + Target position)
Receive: TPDO1 (Status word + Position actual)
Monitor: TPDO2 (Error register)
```

### Velocity Control (PV Mode)

```yaml
Send: RPDO2 (Mode + Target velocity)
Receive: TPDO1 (Status word + Velocity actual)
Monitor: TPDO2 (Error register)
```

### Cyclic Synchronous Position (CSP)

```yaml
Send: RPDO1 (Target position every SYNC)
Receive: TPDO1 (Position actual every SYNC)
Period: 10ms (sync_period)
```

## 🔑 Key SDO Default Values

| Parameter        | Index  | Value | Unit              |
| ---------------- | ------ | ----- | ----------------- |
| Profile velocity | 0x6081 | 10000 | counts/s          |
| Acceleration     | 0x6083 | 5000  | counts/s²         |
| Max velocity     | 0x607F | 50000 | counts/s          |
| Max current      | 0x6073 | 1000  | mA (device units) |
| Position window  | 0x6067 | 100   | counts            |
| Following error  | 0x6065 | 5000  | counts            |

## 📊 Data Flow Diagram

```bash
Controller                     Device (TRACTION_PWR)
    |                                  |
    |----RPDO1 (Control+Position)---->|
    |----RPDO2 (Mode+Velocity)------->|
    |                                  |
    |<---TPDO1 (Status+Feedback)------|
    |<---TPDO2 (Mode+Error)-----------|
    |                                  |
    |----RPDO3 (Profile Params)------>| (Async)
    |----RPDO4 (Digital Out)--------->|
    |                                  |
    |<---TPDO3 (Digital In)-----------|
    |                                  |
    └─SYNC (10ms)─────────────────────┘
```

## ⚙️ Configuration Steps

1. **SDO Initialization** (at startup)
   - Set motion limits
   - Configure gear ratio
   - Set homing parameters

2. **Enable PDOs**
   - RPDOs: Map command objects
   - TPDOs: Map feedback objects

3. **Start SYNC**
   - Master sends SYNC every 10ms

4. **Enter OPERATIONAL**
   - Device begins PDO communication

## 🛠️ Quick Customization

### Change Velocity Limit

```yaml
sdo:
  - { index: 0x607F, sub_index: 0, value: 100000 } # Double the limit
```

### Adjust Position Window

```yaml
sdo:
  - { index: 0x6067, sub_index: 0, value: 50 } # Tighter tolerance
```

### Modify Gear Ratio (e.g., 1000:1)

```yaml
sdo:
  - { index: 0x6092, sub_index: 1, value: 1000 }
  - { index: 0x6092, sub_index: 2, value: 1 }
```

## 📚 Related Files

- **Configuration**: `bus.yml`
- **Full Documentation**: `PDO_CONFIGURATION.md`
- **Object Dictionary**: `TRACTION_PWR_table.md`
- **Device Description**: `TRACTION_PWR.eds`

## ⚠️ Important Notes

- **PDO Size Limit**: 8 bytes maximum per PDO (CAN 2.0)
- **SYNC Required**: For synchronous TPDOs (0x01 transmission type)
- **Byte Order**: Little-endian (LSB first)
- **State Machine**: Device must be in OPERATIONAL for PDO communication
- **TPDO4**: Disabled by default (cam functionality)

## 🔍 Verification Checklist

- [ ] SDO values match application requirements
- [ ] RPDO mappings contain necessary command objects
- [ ] TPDO mappings provide required feedback
- [ ] SYNC period appropriate for control loop (10ms default)
- [ ] All PDOs ≤ 8 bytes
- [ ] Device COB-IDs don't conflict with other nodes
- [ ] Error monitoring objects mapped (0x1001, 0x60F4)
