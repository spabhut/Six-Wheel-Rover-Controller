import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'rover'
    pkg_dir = get_package_share_directory(pkg_name)

    # Process Xacro
    xacro_file = os.path.join(pkg_dir, 'urdf', 'rover.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Define RViz config path
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'rover.rviz')

    # Define World file path
    world_file = os.path.join(pkg_dir, 'worlds', 'mars_terrain.world')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Gazebo Classic (Now pointing to your custom world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world_file
        }.items()
    )
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rover'],
        output='screen'
    )

    # RViz2 with config argument
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        rsp_node,
        gazebo,
        spawn_node,
        rviz_node
    ])