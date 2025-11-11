#!/usr/bin/env python3
"""
Launch file for testing motor driver node with single traction motor.

This launch file:
- Launches SocketCAN bridge (CanopenLifeCycleNode) with runtime validation
- Loads traction motor test parameters
- Launches motor_driver_node
- Provides easy debugging of SDO services
- Includes automatic shutdown on failures

Usage:
    ros2 launch ros2_waveshare motor_driver_test.launch.py

Test SDO read:
    ros2 service call /motors/motor_1/sdo/read \
        ros2_waveshare_msgs/srv/SDORead "{object_name: 'Device Type'}"
"""

import subprocess
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.actions import Shutdown as ShutdownAction
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown, matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def check_node_running(node_name: str) -> bool:
    """Check if a ROS2 node with the given name is already running.

    Args:
        node_name: Name of the node to check

    Returns:
        True if node is running, False otherwise
    """
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            return f'/{node_name}' in nodes or node_name in nodes
        return False
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def validate_and_create_entities(context, *args, **kwargs):
    """Validate launch conditions and create entities.

    Checks if bridge node already exists before launching.
    Returns shutdown action if validation fails.
    """
    bridge_node_name = 'socketcan_bridge'

    # Check if bridge already running
    if check_node_running(bridge_node_name):
        print(f"\n{'='*80}", file=sys.stderr)
        print(
            f"ERROR: Cannot launch - node '/{bridge_node_name}' is already running!", file=sys.stderr)
        print(f"{'='*80}", file=sys.stderr)
        print(f"\nOptions:", file=sys.stderr)
        print(f"  1. Stop the existing bridge:", file=sys.stderr)
        print(
            f"     ros2 lifecycle set /{bridge_node_name} shutdown", file=sys.stderr)
        print(f"  2. Check running nodes:", file=sys.stderr)
        print(f"     ros2 node list", file=sys.stderr)
        print(f"{'='*80}\n", file=sys.stderr)
        return [ShutdownAction(reason=f"Node '/{bridge_node_name}' already exists")]

    # Get launch configuration values
    bridge_params_file = context.launch_configurations.get(
        'bridge_params_file', '')
    params_file = context.launch_configurations.get('params_file', '')
    log_level = context.launch_configurations.get('log_level', 'info')

    return create_launch_entities(bridge_params_file, params_file, log_level)


def create_launch_entities(bridge_params_file: str, params_file: str, log_level: str):
    """Create the launch entities for bridge and motor driver.

    Args:
        bridge_params_file: Path to bridge parameters
        params_file: Path to motor driver parameters
        log_level: ROS logging level

    Returns:
        List of launch entities
    """
    entities = []

    # SocketCAN Bridge (Lifecycle Node)
    bridge_node = LifecycleNode(
        package='ros2_waveshare',
        executable='canopen_lifecycle_node',
        name='socketcan_bridge',
        namespace='',
        output='screen',
        parameters=[bridge_params_file] if bridge_params_file else [],
        arguments=[
            '--ros-args',
            '--log-level',
            log_level
        ],
        emulate_tty=True,
    )
    entities.append(bridge_node)

    # Auto-configure bridge event (only when process starts)
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=bridge_node,
            on_start=[
                LogInfo(
                    msg="[LifecycleLaunch] Bridge node started, configuring..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ]
        )
    )
    entities.append(configure_event)

    # Auto-activate bridge after configuration completes
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg="[LifecycleLaunch] Bridge node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(bridge_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True,
        )
    )
    entities.append(activate_event)

    # Activation failure handler - shutdown if activation fails
    activation_failure_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge_node,
            start_state='activating',
            goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Bridge activation failed! Shutting down..."),
                EmitEvent(event=Shutdown(reason='Bridge activation failed'))
            ],
            handle_once=True,
        )
    )
    entities.append(activation_failure_handler)

    # Bridge exit handler
    bridge_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Bridge node exited'))]
        )
    )
    entities.append(bridge_exit_handler)

    # Motor driver node (start after bridge is ready)
    motor_driver_node = Node(
        package='ros2_waveshare',
        executable='motor_driver_node',
        name='motor_driver',
        output='screen',
        parameters=[params_file] if params_file else [],
        arguments=[
            '--ros-args',
            '--log-level',
            log_level
        ],
        emulate_tty=True,
    )

    # Delay motor driver start to allow bridge to initialize
    delayed_motor_driver = TimerAction(
        period=2.0,
        actions=[motor_driver_node]
    )
    entities.append(delayed_motor_driver)

    return entities


def generate_launch_description():
    """Generate launch description with runtime validation and lifecycle management."""
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

    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            'Motor Driver Test Launch\n',
            '=' * 80, '\n',
            'Bridge params: ', LaunchConfiguration('bridge_params_file'), '\n',
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
            '    ros2_waveshare_msgs/srv/SDORead "{object_name: \'Device Type\'}"\n',
            '  ros2 topic echo /motors/motor_1/feedback\n',
            '  ros2 lifecycle list\n',
            '\n',
            '=' * 80, '\n'
        ]
    )

    # Validation function - checks if bridge already running
    validation_function = OpaqueFunction(function=validate_and_create_entities)

    return LaunchDescription([
        # Launch arguments
        bridge_params_file_arg,
        params_file_arg,
        log_level_arg,

        # Info message
        info_msg,

        # Validation and conditional entity creation
        validation_function,
    ])
