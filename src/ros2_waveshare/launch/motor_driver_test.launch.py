#!/usr/bin/env python3
"""
Launch file for testing motor driver node with single traction motor.

This launch file:
- Launches SocketCAN bridge (CanopenLifeCycleNode)
- Loads traction motor test parameters
- Launches motor_driver_node
- Provides easy debugging of SDO services

Usage:
    ros2 launch ros2_waveshare motor_driver_test.launch.py

Test SDO read:
    ros2 service call /motors/motor_1/sdo/read \
        ros2_waveshare_msgs/srv/SDORead "{object_name: 'Device Type'}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Generate launch description with bridge and motor driver."""
    # Declare launch arguments
    bridge_params_file_arg = DeclareLaunchArgument(
        'bridge_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_waveshare'),
            'config',
            'bridge_params.yaml'
        ]),
        description='Path to bridge parameters file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_waveshare'),
            'config',
            'traction_motor_test.yaml'
        ]),
        description='Path to motor driver parameters file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )

    # SocketCAN Bridge (Lifecycle Node)
    bridge_node = LifecycleNode(
        package='ros2_waveshare',
        executable='canopen_lifecycle_node',
        name='socketcan_bridge',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('bridge_params_file')],
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level')
        ],
        emulate_tty=True,
    )

    # Auto-configure bridge on startup
    configure_bridge = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.name == 'socketcan_bridge',
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
            ]
        )
    )

    # Auto-activate bridge after configuration
    activate_bridge = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.name == 'socketcan_bridge',
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # Motor driver node (start after bridge is ready)
    motor_driver_node = Node(
        package='ros2_waveshare',
        executable='motor_driver_node',
        name='motor_driver',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        arguments=[
            '--ros-args',
            '--log-level',
            LaunchConfiguration('log_level')
        ],
        emulate_tty=True,
    )

    # Delay motor driver start to allow bridge to initialize
    delayed_motor_driver = TimerAction(
        period=2.0,
        actions=[motor_driver_node]
    )

    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            'Motor Driver Test Launch\n',
            '=' * 80, '\n',
            'Bridge params: ',
            LaunchConfiguration('bridge_params_file'), '\n',
            'Motor params: ', LaunchConfiguration('params_file'), '\n',
            'Log level: ', LaunchConfiguration('log_level'), '\n',
            '\n',
            'Nodes:\n',
            '  1. SocketCAN Bridge (socketcan_bridge)\n',
            '  2. Motor Driver (motor_driver)\n',
            '\n',
            'Available services:\n',
            '  - /motors/motor_1/sdo/read\n',
            '  - /motors/motor_1/sdo/write\n',
            '  - /motors/motor_1/set_operation_mode\n',
            '  - /motors/motor_1/get_motor_info\n',
            '\n',
            'Available actions:\n',
            '  - /motors/motor_1/enable\n',
            '  - /motors/motor_1/reset_fault\n',
            '  - /motors/motor_1/move_to_position\n',
            '\n',
            'Test commands:\n',
            '  ros2 service list\n',
            '  ros2 service call /motors/motor_1/sdo/read \\\n',
            '    ros2_waveshare_msgs/srv/SDORead '
            '"{object_name: \'Device Type\'}"\n',
            '  ros2 topic echo /motors/motor_1/feedback\n',
            '  ros2 lifecycle list\n',
            '\n',
            '=' * 80, '\n'
        ]
    )

    return LaunchDescription([
        bridge_params_file_arg,
        params_file_arg,
        log_level_arg,
        info_msg,
        bridge_node,
        configure_bridge,
        activate_bridge,
        delayed_motor_driver,
    ])
