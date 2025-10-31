"""Launch file for Waveshare CAN-USB bridge lifecycle node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():
    """Generate launch description with lifecycle node management."""
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_microphase'),
            'config',
            'bridge_params.yaml'
        ]),
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
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True
    )
    
    # Emit configure event when node starts
    configure_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge_node,
            start_state='unconfigured',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.name == 'waveshare_bridge',
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure'))
    )
    
    # Emit activate event when node is configured
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge_node,
            start_state='configuring',
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
    
    # Shutdown on node exit
    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Bridge node exited'))]
        )
    )
    
    return LaunchDescription([
        params_file_arg,
        auto_configure_arg,
        auto_activate_arg,
        bridge_node,
        configure_event,
        activate_event,
        shutdown_event,
    ])
