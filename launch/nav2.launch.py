import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('rover')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([navigation])