#!/usr/bin/env python3

"""Launch file for simple CAN communication test."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Simple test launch for basic CAN communication."""

    # Launch arguments
    usb0_device_arg = DeclareLaunchArgument(
        'usb0_device',
        default_value='/dev/ttyUSB0',
        description='First USB-CAN adapter (publisher)'
    )

    usb1_device_arg = DeclareLaunchArgument(
        'usb1_device',
        default_value='/dev/ttyUSB1',
        description='Second USB-CAN adapter (subscriber)'
    )

    # CAN Frame Publisher (sends commands)
    can_publisher = Node(
        package='usb_can_bridge_test',
        executable='can_frame_publisher',
        name='can_publisher',
        parameters=[{
            'device_path': LaunchConfiguration('usb0_device'),
            'can_speed': 500000,
            'target_node_id': 2,
            'publish_rate': 2.0
        }],
        output='screen'
    )

    # CAN Frame Subscriber (receives and displays)
    can_subscriber = Node(
        package='usb_can_bridge_test',
        executable='can_frame_subscriber',
        name='can_subscriber',
        parameters=[{
            'device_path': LaunchConfiguration('usb1_device'),
            'can_speed': 500000,
            'enable_debug': True
        }],
        output='screen'
    )

    # assert that the permissions are set correctly on the devices
    # os.system('sudo chmod 666 ' + LaunchConfiguration('usb0_device').perform({}))
    # os.system('sudo chmod 666 ' + LaunchConfiguration('usb1_device').perform({}))

    return LaunchDescription([
        usb0_device_arg,
        usb1_device_arg,
        can_publisher,
        can_subscriber,
    ])
