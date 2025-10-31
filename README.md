# CANOpen Middleware Implementation TODO List

## Architecture Overview

![](doc/arch.md)

## Project idea

This project aims to provide a ROS2-based CAN bus control middleware using the Waveshare USB-CAN-A adapter. It includes a C++ library for interfacing with the CAN bus and ROS2 nodes for sending and receiving CAN messages.

The main components of the project are:
- **Waveshare C++ Library**: A C++ library for interfacing with the Waveshare USB-CAN-A adapter. This initialize the USB serial connection through Linux device files (e.g., `/dev/ttyUSB0`) and implement the waveshare protocol to send and receive CAN frames through the microcontroller on the adapter. It also provides a bridge to Linux SocketCAN interface to allow easy integration with existing CAN tools and libraries through basic message forwarding from one interface to another.
- **CANOpen Lifecycle Node**: ROS2 nodes that utilize the Waveshare C++ library to manage the bridge through lifecycle management. This node can be started, stopped, and configured through ROS2 lifecycle services.
- **ROS2-CANOpen Integration**: Integration with ROS2-CANOpen stack to provide higher-level CANOpen functionalities such as PDO and SDO communication, network management, and device monitoring.
- **CIA402 Integration**: Implementation of the CIA402 profile for controlling servo drives over CANopen. When started, opens a ROS2 action server to receive position/velocity/torque commands and translate them into appropriate CANopen messages.
- **Launch Files and Configuration**: ROS2 launch files and configuration parameters to easily set up and run the CAN bus control middleware.
  

## note

To set up proper permissions for the Waveshare USB-CAN-A adapter, you may need to create a udev rule. You can do this by editing or creating a new udev rules file. Here’s how to do it:

```bash
sudoedit /etc/udev/rules.d/50-myusb.rules
```

Add the following lines:

```
KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"
```

Then reload the udev rules and replug the device:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

To automatically load the can and vcan kernel modules at boot, you can add them to the `/etc/modules` file:

```bash
echo "can" | sudo tee -a /etc/modules
echo "can-raw" | sudo tee -a /etc/modules
echo "can-bcm" | sudo tee -a /etc/modules
echo "vcan" | sudo tee -a /etc/modules
```
to load the modules immediately without rebooting, run:

```bash
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-bcm
sudo modprobe vcan
```
Set up ip link for vcan0:

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```
