#!/bin/bash

echo "=== USB Device Access Troubleshooting ==="

echo "1. Available USB/TTY devices:"
ls -la /dev/tty*USB* /dev/tty*ACM* 2>/dev/null || echo "No USB serial devices found"

echo -e "\n2. Current user info:"
echo "User: $(whoami)"
echo "UID: $(id -u)"
echo "GID: $(id -g)"
echo "Groups: $(groups)"

echo -e "\n3. Adding user to dialout group (if needed):"
if ! groups | grep -q dialout; then
    echo "Adding user to dialout group..."
    sudo usermod -a -G dialout $(whoami) 2>/dev/null || echo "Failed to add to dialout group"
else
    echo "User already in dialout group"
fi

echo -e "\n4. Setting device permissions (if devices exist):"
for device in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$device" ]; then
        echo "Setting permissions for $device"
        sudo chmod 666 "$device" 2>/dev/null || echo "Failed to set permissions for $device"
        ls -la "$device"
    fi
done

echo -e "\n5. Testing canusb compilation:"
cd /home/ros/ros_ws/USB-CAN-A
if [ -f canusb.c ]; then
    echo "Compiling canusb..."
    gcc -o canusb canusb.c
    if [ $? -eq 0 ]; then
        echo "Compilation successful!"
        echo "Usage: ./canusb -d /dev/ttyUSB0 -s 1000000 -t"
    else
        echo "Compilation failed!"
    fi
else
    echo "canusb.c not found!"
fi

echo -e "\n=== End of troubleshooting ==="
