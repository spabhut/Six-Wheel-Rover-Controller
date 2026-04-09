from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # RTAB-Map parameters
    parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'wait_for_transform': 0.2,
        'Grid/RangeMax': '5.0',
        'use_sim_time': True
    }]

    # Topic Remappings to match Gazebo D455 outputs
    remappings = [
        ('rgb/image', '/d455/image_raw'),
        ('rgb/camera_info', '/d455/camera_info'),
        ('depth/image', '/d455/depth/image_raw'),
        ('odom', '/odom')
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=remappings,
        arguments=['-d']
    )

    return LaunchDescription([
        rtabmap_node
    ])