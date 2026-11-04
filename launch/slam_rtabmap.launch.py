from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rtabmap_launch'), '/launch/rtabmap.launch.py'
        ]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'scan_topic': '/scan',
            'visual_odometry': 'false',
            'odom_topic': '/odom',
            'frame_id': 'base_footprint',
            'approx_sync': 'true',
            'qos': '1',
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        rtabmap_launch
    ])
