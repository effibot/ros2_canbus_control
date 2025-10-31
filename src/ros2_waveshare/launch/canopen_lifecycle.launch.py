"""
Launch file for the CANopen lifecycle node with automatic configuration and activation.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for the CANopen lifecycle node with automatic configuration and activation.

    Return: LaunchDescription: The launch description object containing all launch actions and configurations.
    """
    # Get package directory
    pkg_dir = get_package_share_directory('ros2_microphase')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'bridge_params.yaml'),
        description='Full path to bridge parameters file'
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

    # Create lifecycle node
    bridge_node = LifecycleNode(
        package='ros2_microphase',
        executable='canopen_lifecycle_node',
        name='waveshare_bridge',
        namespace='',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Configure transition event
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node.name == 'waveshare_bridge',
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure'))
    )

    # Activate transition event (after successful configuration)
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.name == 'waveshare_bridge',
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate'))
    )

    # Shutdown on exit
    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Bridge node exited'))
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        auto_configure_arg,
        auto_activate_arg,
        bridge_node,
        configure_event,
        activate_event,
        shutdown_event,
    ])
