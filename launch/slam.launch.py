from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_odom': False,
        'approx_sync': True,
        'wait_for_transform': 0.5,
        'Grid/RangeMax': '5.0',
        'use_sim_time': False,
        'odom_frame_id': 'odom',           # FIX — must not be empty
        'publish_tf': True,                # rtabmap publishes odom->base_link tf itself
        'subscribe_odom_info': False,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'Mem/IncrementalMemory': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'Vis/EstimationType': '1',
    }]

    remappings = [
        ('rgb/image',        '/d455/d455/color/image_raw'),
        ('rgb/camera_info',  '/d455/d455/color/camera_info'),
        ('depth/image',      '/d455/d455/depth/image_rect_raw'),
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