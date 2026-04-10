from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'wait_for_transform': 0.5,
        'Grid/RangeMax': '5.0',
        'use_sim_time': False
    }]

    # Topics based on actual rostopic list output
    # Camera has double namespace /d455/d455/... due to name+namespace combo
    remappings = [
        ('rgb/image',        '/d455/d455/color/image_raw'),
        ('rgb/camera_info',  '/d455/d455/color/camera_info'),
        ('depth/image',      '/d455/d455/depth/image_rect_raw'),
        ('odom',             '/odom'),
        ('grid_map',         '/map')
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

    return LaunchDescription([rtabmap_node])