"""Launch file for Waveshare CAN-USB bridge lifecycle node.

This launch file includes runtime checks to prevent duplicate node launches
and provides clear error messages when conflicts are detected.
"""

import os
import subprocess
import sys

from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.actions import Shutdown as ShutdownAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown, matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
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


def check_usb_device_available(device_path: str) -> tuple[bool, str]:
    """Check if USB device exists and is not locked by another process.

    Args:
        device_path: Path to USB device (e.g., /dev/ttyUSB0)

    Returns:
        Tuple of (is_available, message)
    """
    if not os.path.exists(device_path):
        return False, f"USB device {device_path} does not exist"

    try:
        # Try to identify process using the device
        result = subprocess.run(
            ['fuser', device_path],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0 and result.stdout.strip():
            pids = result.stdout.strip()
            return False, f"USB device {device_path} is in use by process(es): {pids}"
        return True, f"USB device {device_path} is available"
    except (subprocess.TimeoutExpired, FileNotFoundError):
        # fuser not available or timeout, assume device is available
        return True, f"USB device {device_path} exists (lock check skipped)"


def validate_launch_conditions(context, *args, **kwargs):
    """Validate that the bridge can be launched safely.

    This function performs runtime checks before launching the bridge node:
    - Checks if node with same name already exists
    - Checks if USB device is available

    Returns empty list if validation fails (prevents launch),
    otherwise returns the launch entities.
    """
    # Get launch configuration values
    node_name = context.launch_configurations.get(
        'node_name', 'waveshare_bridge')
    params_file = context.launch_configurations.get('params_file', '')
    auto_configure = context.launch_configurations.get(
        'auto_configure', 'true').lower() == 'true'
    auto_activate = context.launch_configurations.get(
        'auto_activate', 'true').lower() == 'true'

    # Check 1: Is node already running?
    if check_node_running(node_name):
        print(f"\n{'='*80}", file=sys.stderr)
        print(
            f"ERROR: Cannot launch bridge - node '/{node_name}' is already running!", file=sys.stderr)
        print(f"{'='*80}", file=sys.stderr)
        print(f"\nOptions:", file=sys.stderr)
        print(f"  1. Stop the existing node first:", file=sys.stderr)
        print(
            f"     ros2 lifecycle set /{node_name} shutdown", file=sys.stderr)
        print(f"  2. Use a different node name:", file=sys.stderr)
        print(f"     ros2 launch ros2_waveshare bridge_bringup.launch.py node_name:=bridge_2", file=sys.stderr)
        print(f"  3. Check running nodes:", file=sys.stderr)
        print(f"     ros2 node list", file=sys.stderr)
        print(f"{'='*80}\n", file=sys.stderr)

        # Return a Shutdown action to terminate the launch process
        return [ShutdownAction(reason=f"Node '/{node_name}' already exists")]

    # Check 2: Is USB device available? (basic check, actual flock happens during activation)
    # We can't easily get the USB device path from params_file here, so skip this check
    # The actual device lock will be checked during on_activate()

    # All checks passed, create launch entities
    return create_launch_entities(node_name, params_file, auto_configure, auto_activate)


def create_launch_entities(node_name: str, params_file: str,
                           auto_configure: bool, auto_activate: bool):
    """Create the launch entities for the bridge node.

    Args:
        node_name: Name for the lifecycle node
        params_file: Path to parameters file
        auto_configure: Whether to auto-configure on startup
        auto_activate: Whether to auto-activate after configuration

    Returns:
        List of launch entities (node, event handlers, etc.)
    """
    entities = []

    # Create lifecycle node
    bridge_node = LifecycleNode(
        package='ros2_waveshare',
        executable='canopen_lifecycle_node',
        name=node_name,
        namespace='',
        parameters=[params_file] if params_file else [],
        output='screen',
        emulate_tty=True
    )
    entities.append(bridge_node)

    # Configure event handler (only if auto_configure is enabled)
    if auto_configure:
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

    # Activate event handler (only if auto_activate is enabled)
    if auto_activate:
        activate_event = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=bridge_node,
                goal_state='inactive',
                entities=[
                    LogInfo(
                        msg="[LifecycleLaunch] Bridge node is activating."),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(bridge_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        ),
                    ),
                ],
                handle_once=True,
            )
        )
        entities.append(activate_event)

        # Activation failure handler - shutdown if activation fails
        # This handles cases like USB device already in use
        activation_failure_handler = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=bridge_node,
                start_state='activating',
                goal_state='inactive',
                entities=[
                    LogInfo(
                        msg="[LifecycleLaunch] Bridge activation failed! Shutting down..."),
                    EmitEvent(event=Shutdown(
                        reason='Bridge activation failed'))
                ],
                handle_once=True,
            )
        )
        entities.append(activation_failure_handler)

    # Shutdown event handler
    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Bridge node exited'))]
        )
    )
    entities.append(shutdown_event)

    return entities


def generate_launch_description():
    """Generate launch description with runtime validation and lifecycle management."""

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_waveshare'),
            'config',
            'bridge_params.yaml'
        ]),
        description='Full path to bridge parameters file'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='waveshare_bridge',
        description='Name of the lifecycle node (must be unique)'
    )

    auto_configure_arg = DeclareLaunchArgument(
        'auto_configure',
        default_value='true',
        description='Automatically configure the node on startup'
    )

    auto_activate_arg = DeclareLaunchArgument(
        'auto_activate',
        default_value='true',
        description='Automatically activate the node after configuration'
    )

    # Info message
    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            'Waveshare CAN-USB Bridge Launch\n',
            '=' * 80, '\n',
            'Node name: ', LaunchConfiguration('node_name'), '\n',
            'Params file: ', LaunchConfiguration('params_file'), '\n',
            'Auto-configure: ', LaunchConfiguration('auto_configure'), '\n',
            'Auto-activate: ', LaunchConfiguration('auto_activate'), '\n',
            '=' * 80, '\n'
        ]
    )

    # OpaqueFunction allows us to run validation checks at launch time
    # and conditionally add entities based on the results
    validation_function = OpaqueFunction(function=validate_launch_conditions)

    return LaunchDescription([
        # Launch arguments
        params_file_arg,
        node_name_arg,
        auto_configure_arg,
        auto_activate_arg,

        # Info message
        info_msg,

        # Validation and conditional node creation
        validation_function,
    ])
