from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    delete_db = LaunchConfiguration('delete_db')

    # Launch RTAB-Map dengan mode Fusi Sensor Penuh (LiDAR 360° + RGB-D Camera + IMU/Odom)
    # Sesuai diagram arsitektur Bagian 3 & 7 pada Dokumen Perencanaan
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rtabmap_launch'), '/launch/rtabmap.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': '',
            'map_topic': '/map',
            'rtabmap_args': ('--delete_db_on_start ' if delete_db else '') + '--Grid/RayTracing true --Grid/CellSize 0.05',
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            
            # 1. Konfigurasi Input Sensor RGB-D
            'subscribe_depth': 'true',
            'rgb_topic': '/camera/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.05',
            'qos': '1',

            # 2. Konfigurasi Input Sensor 360 LiDAR (Aktif untuk Fusi)
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            
            # 3. Odometri roda virtual dari Gazebo
            'visual_odometry': 'false',
            'odom_topic': '/odom',

            # 4. Antarmuka Visualisasi RTAB-Map
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
