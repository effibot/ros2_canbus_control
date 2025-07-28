"""Launch file for microphase CAN configuration."""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description with multiple components."""
    path_file = os.path.dirname(__file__)

    package_name = 'microphase_can_config'
    bus_config_name = 'bus_config'
    can_interface_name = os.getenv('CAN_INTERFACE_NAME', 'can0')

    ld = launch.LaunchDescription()

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory(
                    'canopen_core'), 'launch'),
                '/canopen.launch.py',
            ]
        ),
        launch_arguments={
            'master_config': os.path.join(
                get_package_share_directory(f'{package_name}'),
                'config',
                f'{bus_config_name}',
                'master.dcf',
            ),
            'master_bin': os.path.join(
                get_package_share_directory(f'{package_name}'),
                'config',
                f'{bus_config_name}',
                'master.bin',
            ),
            'bus_config': os.path.join(
                get_package_share_directory(f'{package_name}'),
                'config',
                f'{bus_config_name}',
                'bus.yml',
            ),
            'can_interface_name': f'{can_interface_name}',
        }.items(),

    )

    ld.add_action(device_container)

    return ld
