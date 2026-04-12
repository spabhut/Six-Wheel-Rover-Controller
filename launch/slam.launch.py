from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    parameters = [{
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'wait_for_transform': 1.0,

        # Sim time
        'use_sim_time': True,

        # Depth camera fix — limo publishes 32-bit float in METERS
        # Far clip is 10m in xacro, so clamp to that
        'RGBD/DepthMax': '10.0',
        'RGBD/DepthMin': '0.1',
        'Mem/SaveDepth16Format': 'false',       # stay in 32-bit float, do NOT convert to 16-bit
        'Mem/DepthCompressionFormat': '.png',   # use png compression, not rvl

        # Map update thresholds — low so map builds even with small movement
        'RGBD/LinearUpdate': '0.01',
        'RGBD/AngularUpdate': '0.01',

        # Force 2D (ground navigation)
        'Reg/Force3DoF': 'true',

        # Grid map settings
        'Grid/RangeMax': '5.0',
        'Grid/FromDepth': 'true',
        'Grid/3D': 'false',

        # Odometry covariance override
        'Odom/ResetCountdown': '1',
        'Odom/GuessMotion': 'true',
    }]

    remappings = [
        ('rgb/image',        '/limo/depth_camera_link/image_raw'),
        ('rgb/camera_info',  '/limo/depth_camera_link/camera_info'),
        ('depth/image',      '/limo/depth_camera_link/depth/image_raw'),
        ('odom',             '/odom'),
        ('grid_map',         '/map'),
        ('scan',             '/scan'),
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

    delayed_rtabmap = TimerAction(
        period=5.0,
        actions=[rtabmap_node]
    )

    return LaunchDescription([delayed_rtabmap])
