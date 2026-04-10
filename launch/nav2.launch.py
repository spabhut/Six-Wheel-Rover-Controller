import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('rover')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    pc_to_laser = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/d455/depth/color/points'),
            ('scan',     '/scan')
        ],
        parameters=[{
            'target_frame': 'camera_link',
            'transform_tolerance': 0.05,
            'min_height': -0.05,
            'max_height': 0.5,
            'range_min': 0.2,
            'range_max': 8.0,
            'use_sim_time': False
        }]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([pc_to_laser, navigation])