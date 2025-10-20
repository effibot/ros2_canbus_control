# TRACTION_PWR PDO Mapping Table

This table lists all `PDOMapping=1` objects from `TRACTION_PWR.eds` within the standard CANopen range (0x1000-0x6FFF). It includes CANopen index, parameter name, data type, C++ equivalent, size, default value, and access type.

## Standard CANopen Objects (0x1000-0x6FFF)

| Index    | ParameterName                   | DataType | C++ type | Size | Default Value | Access | Notes                   |
| -------- | ------------------------------- | -------- | -------- | ---- | ------------- | ------ | ----------------------- |
| 0x1001   | Error register                  | 0x0005   | uint8_t  | 1    | 0             | ro     | Device status           |
| 0x1002   | Manufacturer Status Register    | 0x0007   | uint32_t | 4    | -             | ro     | Vendor-specific         |
| 0x6040   | Control word                    | 0x0006   | uint16_t | 2    | -             | rw     | Device state control    |
| 0x6041   | Status word                     | 0x0006   | uint16_t | 2    | -             | ro     | Device state feedback   |
| 0x6060   | Mode of operation               | 0x0002   | int8_t   | 1    | 3             | rww    | Operating mode select   |
| 0x6061   | Mode of operation display       | 0x0002   | int8_t   | 1    | 3             | ro     | Active operating mode   |
| 0x6064   | Position actual value           | 0x0004   | int32_t  | 4    | -             | ro     | Current position        |
| 0x6065   | Following error window          | 0x0007   | uint32_t | 4    | 0xffffffff    | rww    | Max allowed error       |
| 0x6066   | Following error time-out        | 0x0006   | uint16_t | 2    | -             | rww    | Error timeout (ms)      |
| 0x6067   | Position window                 | 0x0007   | uint32_t | 4    | 0xffffffff    | rww    | Target reached window   |
| 0x6068   | Position time-out               | 0x0006   | uint16_t | 2    | -             | rww    | Position timeout (ms)   |
| 0x606C   | Velocity actual value           | 0x0004   | int32_t  | 4    | -             | ro     | Current velocity        |
| 0x6073   | Max Current                     | 0x0006   | uint16_t | 2    | 1000          | rw     | Maximum motor current   |
| 0x607A   | Target position                 | 0x0004   | int32_t  | 4    | -             | rww    | Position setpoint       |
| 0x607C   | Home offset                     | 0x0004   | int32_t  | 4    | -             | rww    | Homing position offset  |
| 0x607D.1 | Min position limit              | 0x0004   | int32_t  | 4    | -2147483648   | rww    | Software limit (min)    |
| 0x607D.2 | Max position limit              | 0x0004   | int32_t  | 4    | 2147483647    | rww    | Software limit (max)    |
| 0x607E   | Polarity                        | 0x0005   | uint8_t  | 1    | -             | rw     | Direction inversion     |
| 0x607F   | Max profile velocity            | 0x0007   | uint32_t | 4    | 0xffffffff    | rw     | Velocity limit          |
| 0x6081   | Profile velocity                | 0x0007   | uint32_t | 4    | -             | rww    | Target velocity         |
| 0x6083   | Profile acceleration            | 0x0007   | uint32_t | 4    | -             | rww    | Acceleration rate       |
| 0x6084   | Profile deceleration            | 0x0007   | uint32_t | 4    | -             | rww    | Deceleration rate       |
| 0x6085   | Quick stop deceleration         | 0x0007   | uint32_t | 4    | -             | rww    | Emergency stop rate     |
| 0x6086   | Motion profile type             | 0x0003   | int16_t  | 2    | -             | ro     | Profile type (0=linear) |
| 0x6089   | Position notation index         | 0x0002   | int8_t   | 1    | -6            | rw     | Position unit exponent  |
| 0x608A   | Position dimension index        | 0x0005   | uint8_t  | 1    | 0x01          | rw     | Position dimension      |
| 0x608B   | Velocity notation index         | 0x0002   | int8_t   | 1    | -6            | rw     | Velocity unit exponent  |
| 0x608C   | Velocity dimension index        | 0x0005   | uint8_t  | 1    | 0xa6          | rw     | Velocity dimension      |
| 0x608D   | Acceleration notation index     | 0x0002   | int8_t   | 1    | -6            | rw     | Accel unit exponent     |
| 0x608E   | Acceleration dimension index    | 0x0005   | uint8_t  | 1    | 0xa6          | rw     | Accel dimension         |
| 0x6092.1 | Feed constant (numerator)       | 0x0007   | uint32_t | 4    | 65536         | rww    | Gear ratio numerator    |
| 0x6092.2 | Shaft revolutions (denominator) | 0x0007   | uint32_t | 4    | 1             | rww    | Gear ratio denominator  |
| 0x6098   | Homing method                   | 0x0002   | int8_t   | 1    | 35            | rww    | Homing procedure code   |
| 0x6099.1 | Speed during search for switch  | 0x0007   | uint32_t | 4    | 0             | rww    | Homing search speed     |
| 0x6099.2 | Speed during search for zero    | 0x0007   | uint32_t | 4    | 0             | rww    | Homing zero speed       |
| 0x609A   | Homing acceleration             | 0x0007   | uint32_t | 4    | -             | rww    | Homing accel rate       |
| 0x60C1.1 | Interpolation position setpoint | 0x0004   | int32_t  | 4    | 0x00          | rww    | PVT mode position       |
| 0x60C2.2 | Interpolation time index        | 0x0002   | int8_t   | 1    | -3            | ro     | Time unit exponent      |
| 0x60C5   | Max acceleration                | 0x0007   | uint32_t | 4    | 0xffffffff    | rw     | Acceleration limit      |
| 0x60C6   | Max deceleration                | 0x0007   | uint32_t | 4    | 0xffffffff    | rw     | Deceleration limit      |
| 0x60F4   | Following error actual value    | 0x0004   | int32_t  | 4    | -             | ro     | Position tracking error |
| 0x60FD   | Digital inputs                  | 0x0007   | uint32_t | 4    | -             | ro     | Digital I/O status      |
| 0x60FE.1 | Physical digital outputs        | 0x0007   | uint32_t | 4    | -             | rww    | Digital output state    |
| 0x60FE.2 | Digital output bit mask         | 0x0007   | uint32_t | 4    | -             | rww    | Output enable mask      |
| 0x60FF   | Target velocity                 | 0x0004   | int32_t  | 4    | -             | rww    | Velocity mode setpoint  |
| 0x6300.1 | Cam state register (Channel 1)  | 0x0005   | uint8_t  | 1    | -             | ro     | Cam output state        |
| 0x6301.1 | Cam enable (Channel 1)          | 0x0005   | uint8_t  | 1    | -             | rww    | Cam function enable     |
| 0x6302.1 | Cam polarity (Channel 1)        | 0x0005   | uint8_t  | 1    | -             | rww    | Cam output polarity     |

## Access Type Legend
- **ro**: Read-only (device → controller)
- **rw**: Read/write (bidirectional, persistent)
- **rww**: Read/write, written (runtime write, may not persist)
