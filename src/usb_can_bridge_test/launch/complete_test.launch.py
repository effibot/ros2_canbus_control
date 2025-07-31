import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    usb0_device_arg = DeclareLaunchArgument(
        'usb0_device',
        default_value='/dev/ttyUSB0',
        description='First USB-CAN adapter device path (for bridge)'
    )

    usb1_device_arg = DeclareLaunchArgument(
        'usb1_device',
        default_value='/dev/ttyUSB1',
        description='Second USB-CAN adapter device path (for motor simulator)'
    )

    can_speed_arg = DeclareLaunchArgument(
        'can_speed',
        default_value='500000',
        description='CAN bus speed in bps'
    )

    enable_turtle_arg = DeclareLaunchArgument(
        'enable_turtle',
        default_value='true',
        description='Enable turtle visualization'
    )

    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='false',
        description='Enable debug output'
    )

    # Setup virtual CAN interface
    setup_vcan = ExecuteProcess(
        cmd=['bash', '-c',
             'sudo modprobe vcan && '
             'sudo ip link add dev vcan0 type vcan 2>/dev/null || true && '
             'sudo ip link set up vcan0 && '
             'echo "Virtual CAN interface vcan0 ready"'],
        name='setup_vcan',
        output='screen'
    )

    # USB-CAN Bridge (ttyUSB0 -> vcan0)
    usb_can_bridge = Node(
        package='usb_can_bridge_test',
        executable='usb_can_socketcan_bridge',
        name='usb_can_bridge',
        parameters=[{
            'device_path': LaunchConfiguration('usb0_device'),
            'can_speed': LaunchConfiguration('can_speed'),
            'vcan_interface': 'vcan0',
            'enable_debug': LaunchConfiguration('enable_debug')
        }],
        output='screen'
    )

    # Virtual Motor Simulator (ttyUSB1)
    virtual_motor = Node(
        package='usb_can_bridge_test',
        executable='virtual_motor_simulator',
        name='virtual_motor_simulator',
        parameters=[{
            'device_path': LaunchConfiguration('usb1_device'),
            'can_speed': LaunchConfiguration('can_speed'),
            'node_id': 2,
            'enable_debug': LaunchConfiguration('enable_debug'),
            'max_velocity': 10.0,
            'update_rate': 50.0
        }],
        output='screen'
    )

    # Turtlesim for visualization
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        condition=IfCondition(LaunchConfiguration('enable_turtle')),
        output='screen'
    )

    # Turtle Bridge for motor feedback visualization
    turtle_bridge = Node(
        package='usb_can_bridge_test',
        executable='turtle_bridge',
        name='turtle_bridge',
        parameters=[{
            'turtle_name': 'motor_turtle',
            'trace_enabled': True,
            'velocity_scale': 0.2
        }],
        condition=IfCondition(LaunchConfiguration('enable_turtle')),
        output='screen'
    )

    # CANopen Device Manager (uses vcan0)
    device_manager = Node(
        package='canopen_core',
        executable='device_manager_node',
        name='device_manager',
        parameters=[{
            'bus_config': os.path.join(
                '/home/ros/ros_ws/src/usb_can_bridge_test/config',
                'test_bus_config.yml'
            )
        }],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        usb0_device_arg,
        usb1_device_arg,
        can_speed_arg,
        enable_turtle_arg,
        enable_debug_arg,

        # Nodes with timing
        setup_vcan,
        TimerAction(period=2.0, actions=[usb_can_bridge]),
        TimerAction(period=3.0, actions=[virtual_motor]),
        TimerAction(period=4.0, actions=[turtlesim_node]),
        TimerAction(period=5.0, actions=[turtle_bridge]),
        TimerAction(period=6.0, actions=[device_manager]),
    ])
