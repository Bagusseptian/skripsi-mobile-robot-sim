from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    delete_db = LaunchConfiguration('delete_db')

    # Launch RTAB-Map dengan mode Visual SLAM RGB-D (Tanpa LiDAR)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rtabmap_launch'), '/launch/rtabmap.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': '',
            'map_topic': '/map',
            'rtabmap_args': ('--delete_db_on_start ' if delete_db else '') + '--Grid/FromDepth true --Grid/RangeMax 5.0 --Grid/RayTracing true --Grid/CellSize 0.05 --Vis/FeatureType 6 --Vis/MaxFeatures 1000 --RGBD/AngularUpdate 0.05 --RGBD/LinearUpdate 0.05 --Rtabmap/DetectionRate 2.0 --Grid/3D true',
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            
            # Konfigurasi Input Kamera RGB-D
            'subscribe_depth': 'true',
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.05',
            'qos': '1',

            # Nonaktifkan LiDAR untuk pengujian Visual SLAM murni (Fase 4 Dokumen)
            'subscribe_scan': 'false',
            
            # Odometri roda virtual dari Gazebo
            'visual_odometry': 'false',
            'odom_topic': '/odom',

            # Visualisasi UI bawaan RTAB-Map
            'rtabmap_viz': rtabmap_viz,
            'rviz': 'false'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Gunakan clock waktu simulasi'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Buka antarmuka grafis rtabmap_viz'),
        DeclareLaunchArgument('delete_db', default_value='true', description='Hapus database peta sebelumnya saat mulai'),
        rtabmap_launch
    ])
