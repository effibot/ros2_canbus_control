# Speed Control ROS2 Package

This package provides a framework for managing and controlling speed in a ROS2 environment by interfacing with CAN-based communication modules. It is designed as part of a larger ROS2 system for handling CAN bus signals, particularly for applications that require precision motor control and status monitoring.

## Overview

The **Speed Control** package is responsible for:

- Managing CAN bus frames via the `FrameManager` module.
- Reading status words and monitoring device states through the `ReadStatusWord` functionality.
- Mapping Process Data Objects (PDO) using `mapPDO` to correlate sensor and actuator data.
- Sending Service Data Objects (SDO) with the `sendSDO` module for device configuration.
- Adding simulation noise through the `NoiseGenerator` to mimic real-life signal fluctuations.
- Providing miscellaneous helper functionalities in the `Tools` module.

## Package Structure

The package follows a modular design with clear separations between interfaces, implementations, and configuration files. Below is the directory layout and description of each major component:

```
(speed_control)
├── CMakeLists.txt             # Build configuration for CMake
├── package.xml                # ROS2 package manifest
├── config
│   └── speed_control_parameters.yaml  # Runtime parameters for speed control
├── include
│   ├── frameManager.hpp       # Declarations for frame management
│   ├── readStatusWord.hpp     # Interfaces to read device status
│   ├── tools.hpp              # Utility functions and helpers
│   └── speed_control
│       └── speed_control.hpp  # Main speed control class declaration
└── src
    ├── frameManager.cpp       # Frame management implementation
    ├── mapPDO.cpp             # Functionality to map PDO data
    ├── noiseGenerator.cpp     # Noise generation for signal simulation
    ├── readStatusWord.cpp     # Implementation of status word reading
    ├── sendSDO.cpp            # SDO sending functions for configuration
    ├── speed_control.cpp      # Main speed control class implementation
    └── tools.cpp              # Definitions for helper functions
```

## Logic & Code Structure

The design ensures that each module has a specific responsibility and communicates with others through clearly defined interfaces:

- **SpeedControl Class (`speed_control.hpp`/`speed_control.cpp`):**
  - Acts as the central control unit orchestrating all other modules.
  - Initializes required components, sets up ROS2 publishers/subscribers, and handles periodic execution.
  - Calls into the `FrameManager` to process and manage CAN frames.
  - Invokes `ReadStatusWord` functions to get live status feedback from connected devices.
  - Utilizes `Tools` for miscellaneous operations and data formatting as needed.
  - Interfaces with `NoiseGenerator` to add simulation noise during test phases or for robustness testing.
  - Leverages `mapPDO` and `sendSDO` to translate and transmit control commands over the CAN network.

- **FrameManager (`frameManager.hpp`/`frameManager.cpp`):**
  - Manages the reception, parsing, and storage of incoming CAN frames.
  - Provides an interface for the main `SpeedControl` class to fetch current frame data.

- **ReadStatusWord (`readStatusWord.hpp`/`readStatusWord.cpp`):**
  - Reads and interprets status words from devices on the network.
  - Supplies critical feedback to the main control loop in `SpeedControl`.

- **Tools (`tools.hpp`/`tools.cpp`):**
  - Contains shared utility functions that support various operations across the package.
  - Facilitates tasks such as logging, error checking, and data conversions.

- **NoiseGenerator (`noiseGenerator.cpp`):**
  - Generates simulated noise signals to test the resilience of the speed control algorithms.
  - Can be enabled or disabled based on runtime configuration parameters.

- **MapPDO (`mapPDO.cpp`):**
  - Handles the re-mapping of PDO data, ensuring data consistency between sensors and controllers.

- **SendSDO (`sendSDO.cpp`):**
  - Encapsulates the logic for sending SDO messages, which are critical for device configuration and command execution.

## Class Diagram

![Class Diagram](class_diagram.png)

## How the Components Interact

1. The **SpeedControl** class starts its execution with initialization routines, including the configuration of ROS2 nodes and parameters loaded from `speed_control_parameters.yaml`.
2. Upon activation, **SpeedControl** queries the **FrameManager** for any new CAN frames that need processing, ensuring real-time updates are captured.
3. The **ReadStatusWord** module is periodically called within the control loop to obtain the latest operational status from connected devices, providing feedback for safe operation.
4. **Tools** support the process by offering utility functions such as error checking and data transformations throughout the control algorithms.
5. When simulating environmental effects or testing the resilience of the control loop, the **NoiseGenerator** module injects controlled noise into the system.
6. Finally, modifications to device states (like speed adjustments) are transmitted through the **SendSDO** module, with **MapPDO** verifying that data mappings are consistent before execution.

## Building and Running

Use the standard ROS2 build tools to compile this package:

```bash
colcon build --packages-select speed_control
```

After building, source the setup file and run the package:

```bash
source install/setup.bash
ros2 run speed_control <executable_name>
```

Replace `<executable_name>` with the name of the node executable defined in the package configuration.

## Conclusion

This package illustrates a modular approach to managing speed control in a ROS2-based environment using CAN bus communications. Its clearly separated components make it easier to manage, extend, and test each functionality independently.
