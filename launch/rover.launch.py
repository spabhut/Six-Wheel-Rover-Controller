import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'rover'
    pkg_dir = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_dir, 'urdf', 'rover.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'rover.rviz')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455',
        namespace='d455',
        output='screen',
        parameters=[{
            'serial_no': '',
            'enable_depth': True,
            'enable_color': True,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 'linear_interpolation',
            'use_sim_time': False
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        realsense_node,
        rviz_node,
    ])