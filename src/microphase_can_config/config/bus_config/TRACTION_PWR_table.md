# TRACTION_PWR PDO Mapping Table

This table lists all `PDOMapping=1` objects from `TRACTION_PWR.eds`, including CANopen index, parameter name, CANopen data type, equivalent C++ type, and size in bytes.

| Index     | ParameterName                   | DataType code | C++ type   | Size (bytes) |
|-----------|---------------------------------|---------------|------------|--------------|
| 0x1001    | Error register                  | 0x0005        | uint8_t    | 1            |
| 0x6040    | Control word                    | 0x0006        | uint16_t   | 2            |
| 0x6041    | Status word                     | 0x0006        | uint16_t   | 2            |
| 0x6060    | Mode of operation               | 0x0002        | int8_t     | 1            |
| 0x6061    | Mode of operation display       | 0x0002        | int8_t     | 1            |
| 0x6064    | Position actual value           | 0x0004        | int32_t    | 4            |
| 0x606C    | Velocity actual value           | 0x0004        | int32_t    | 4            |
| 0x6073    | Max Current                     | 0x0006        | uint16_t   | 2            |
| 0x6065    | Following error window          | 0x0007        | uint32_t   | 4            |
| 0x6066    | Following error time-out        | 0x0006        | uint16_t   | 2            |
| 0x6067    | Position window                 | 0x0007        | uint32_t   | 4            |
| 0x6068    | Position time-out               | 0x0006        | uint16_t   | 2            |
| 0x607A    | Target position                 | 0x0004        | int32_t    | 4            |
| 0x607C    | Home offset                     | 0x0004        | int32_t    | 4            |
| 0x607D.1  | Min position limit              | 0x0004        | int32_t    | 4            |
| 0x607D.2  | Max position limit              | 0x0004        | int32_t    | 4            |
| 0x607E    | Polarity                        | 0x0005        | uint8_t    | 1            |
| 0x607F    | Max profile velocity            | 0x0007        | uint32_t   | 4            |
| 0x6081    | Profile velocity                | 0x0007        | uint32_t   | 4            |
| 0x6083    | Profile acceleration            | 0x0007        | uint32_t   | 4            |
| 0x6084    | Profile deceleration            | 0x0007        | uint32_t   | 4            |
| 0x6085    | Quick stop deceleration         | 0x0007        | uint32_t   | 4            |
| 0x6086    | Motion profile type             | 0x0003        | int16_t    | 2            |
| 0x6089    | Position notation index         | 0x0002        | int8_t     | 1            |
| 0x608A    | Position dimension index        | 0x0005        | uint8_t    | 1            |
| 0x608B    | Velocity notation index         | 0x0002        | int8_t     | 1            |
| 0x608C    | Velocity dimension index        | 0x0005        | uint8_t    | 1            |
| 0x608D    | Acceleration notation index     | 0x0002        | int8_t     | 1            |
| 0x608E    | Acceleration dimension index    | 0x0005        | uint8_t    | 1            |
| 0x6092.1  | Feed                            | 0x0007        | uint32_t   | 4            |
| 0x6092.2  | Shaft revolutions               | 0x0007        | uint32_t   | 4            |
| 0x6098    | Homing method                   | 0x0002        | int8_t     | 1            |
