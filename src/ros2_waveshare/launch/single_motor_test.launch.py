#!/usr/bin/env python3
# Copyright 2025 Andrea Efficace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Single Motor Test System Launch File

Orchestrates all nodes for single motor testing with closed-loop velocity control:
1. CANopen Lifecycle Node - Motor driver with CAN communication
2. Motor Driver Node - Low-level motor control and status monitoring  
3. Teleop Velocity Node - Keyboard interface for velocity commands
4. Velocity Converter Node - Plugin-based velocity controller (PID)
5. Odometry Publisher Node - Encoder feedback integration and TF broadcasting

Architecture:
    Keyboard → TeleopVelocityNode → cmd_vel (Twist)
                                         ↓
                              VelocityConverterNode (PID plugin)
                                         ↓
                              motor_command (MotorCommand)
                                         ↓
                              MotorDriverNode → CAN bus → Motor
                                         ↓
                              motor_feedback (MotorFeedback)
                                         ↓
                              OdometryPublisherNode → /odom + TF

Usage:
    # With default vcan0 interface (simulation):
    ros2 launch ros2_waveshare single_motor_test.launch.py
    
    # With real CAN interface:
    ros2 launch ros2_waveshare single_motor_test.launch.py can_interface:=can0
    
    # With custom motor namespace:
    ros2 launch ros2_waveshare single_motor_test.launch.py motor_namespace:=/motors/motor_2
    
    # Change plugin for different kinematic model:
    ros2 launch ros2_waveshare single_motor_test.launch.py plugin_name:=ros2_waveshare::DifferentialDriveModel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for single motor test system."""

    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='vcan0',
        description='CAN interface to use (vcan0 for simulation, can0 for real hardware)'
    )

    motor_namespace_arg = DeclareLaunchArgument(
        'motor_namespace',
        default_value='/motors/motor_1',
        description='Namespace for motor topics'
    )

    node_id_arg = DeclareLaunchArgument(
        'node_id',
        default_value='1',
        description='CANopen node ID (1-127)'
    )

    plugin_name_arg = DeclareLaunchArgument(
        'plugin_name',
        default_value='ros2_waveshare::SingleMotorModel',
        description='Dynamic model plugin to use for velocity control'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.1',
        description='Wheel radius in meters'
    )

    # Get launch configurations
    can_interface = LaunchConfiguration('can_interface')
    motor_namespace = LaunchConfiguration('motor_namespace')
    node_id = LaunchConfiguration('node_id')
    plugin_name = LaunchConfiguration('plugin_name')
    wheel_radius = LaunchConfiguration('wheel_radius')

    # Find package share directory
    pkg_share = FindPackageShare('ros2_waveshare')

    # Parameter file paths
    teleop_params = PathJoinSubstitution(
        [pkg_share, 'config', 'teleop_params.yaml'])
    odometry_params = PathJoinSubstitution(
        [pkg_share, 'config', 'odometry_params.yaml'])
    velocity_converter_params = PathJoinSubstitution(
        [pkg_share, 'config', 'velocity_converter_params.yaml'])

    # === Node 1: CANopen Lifecycle Node ===
    canopen_lifecycle_node = Node(
        package='ros2_waveshare',
        executable='canopen_lifecycle_node',
        name='canopen_lifecycle',
        output='screen',
        parameters=[{
            'can_interface': can_interface,
        }],
        remappings=[],
    )

    # === Node 2: Motor Driver Node ===
    motor_driver_node = Node(
        package='ros2_waveshare',
        executable='motor_driver_node',
        name='motor_driver',
        namespace=motor_namespace,
        output='screen',
        parameters=[{
            'node_id': node_id,
            'can_interface': can_interface,
        }],
        remappings=[],
    )

    # === Node 3: Teleop Velocity Node ===
    teleop_velocity_node = Node(
        package='ros2_waveshare',
        executable='teleop_velocity_node',
        name='teleop_velocity',
        output='screen',
        parameters=[teleop_params],
        remappings=[],
        # Run in separate terminal for keyboard input
        prefix='gnome-terminal -- bash -c "cd $(pwd); source install/setup.bash; ros2 run ros2_waveshare teleop_velocity_node; exec bash"',
    )

    # === Node 4: Velocity Converter Node (PID Controller) ===
    velocity_converter_node = Node(
        package='ros2_waveshare',
        executable='velocity_converter_node',
        name='velocity_converter',
        output='screen',
        parameters=[
            velocity_converter_params,
            {
                'plugin_name': plugin_name,
                'motor_namespace': motor_namespace,
                'node_id': node_id,
                'single_motor_model.wheel_radius': wheel_radius,
            }
        ],
        remappings=[],
    )

    # === Node 5: Odometry Publisher Node ===
    odometry_publisher_node = Node(
        package='ros2_waveshare',
        executable='odometry_publisher_node',
        name='odometry_publisher',
        output='screen',
        parameters=[
            odometry_params,
            {
                'motor_namespace': motor_namespace,
                'wheel_radius': wheel_radius,
            }
        ],
        remappings=[],
    )

    # Startup message
    startup_msg = LogInfo(
        msg=[
            '\n',
            '═══════════════════════════════════════════════════════════════\n',
            '  Single Motor Test System Starting\n',
            '═══════════════════════════════════════════════════════════════\n',
            '  CAN Interface: ', can_interface, '\n',
            '  Motor Namespace: ', motor_namespace, '\n',
            '  Node ID: ', node_id, '\n',
            '  Plugin: ', plugin_name, '\n',
            '  Wheel Radius: ', wheel_radius, ' m\n',
            '═══════════════════════════════════════════════════════════════\n',
            '\n',
            '  Nodes:\n',
            '    1. CANopen Lifecycle - Motor driver lifecycle management\n',
            '    2. Motor Driver - Low-level motor control\n',
            '    3. Teleop Velocity - Keyboard control (↑↓0 ESC)\n',
            '    4. Velocity Converter - PID controller with plugin\n',
            '    5. Odometry Publisher - Encoder feedback and TF\n',
            '\n',
            '  Topics:\n',
            '    /cmd_vel - Velocity commands from teleop\n',
            '    ', motor_namespace, '/command - Motor commands\n',
            '    ', motor_namespace, '/feedback - Motor feedback\n',
            '    /odom - Odometry output\n',
            '\n',
            '  TF Frames:\n',
            '    odom → base_link\n',
            '\n',
            '  Services:\n',
            '    ~/velocity_converter/reload_plugin - Reload dynamic model\n',
            '\n',
            '  Controls:\n',
            '    ↑ - Increase velocity\n',
            '    ↓ - Decrease velocity\n',
            '    0 - Stop (zero velocity)\n',
            '    ESC - Shutdown\n',
            '\n',
            '═══════════════════════════════════════════════════════════════\n',
        ]
    )

    return LaunchDescription([
        # Launch arguments
        can_interface_arg,
        motor_namespace_arg,
        node_id_arg,
        plugin_name_arg,
        wheel_radius_arg,

        # Startup message
        startup_msg,

        # Nodes
        canopen_lifecycle_node,
        motor_driver_node,
        velocity_converter_node,
        odometry_publisher_node,
        # Note: teleop_velocity_node commented out - needs manual terminal launch
        # teleop_velocity_node,  # Requires interactive terminal
    ])
