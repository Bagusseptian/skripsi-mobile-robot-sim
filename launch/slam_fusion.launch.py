import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    open_rviz = LaunchConfiguration('rviz')
    delete_db = LaunchConfiguration('delete_db')

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_fusion.rviz')

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
            'rtabmap_args': ('--delete_db_on_start ' if delete_db else '') + '--Grid/RayTracing true --Grid/CellSize 0.05 --Vis/FeatureType 6 --Vis/MaxFeatures 1000 --RGBD/AngularUpdate 0.05 --RGBD/LinearUpdate 0.05 --Rtabmap/DetectionRate 2.0 --Grid/3D true',
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

            # 4. Antarmuka Visualisasi
            'rtabmap_viz': rtabmap_viz,
            'rviz': 'false'
        }.items()
    )

    # Node: RViz2 untuk Visualisasi Fusi SLAM (Robot, LiDAR, PointCloud, Peta 2D/3D)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(open_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Gunakan clock waktu simulasi'),
        DeclareLaunchArgument('rviz', default_value='true', description='Buka RViz2 otomatis untuk visualisasi SLAM'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false', description='Buka GUI bawaan RTAB-Map (opsional)'),
        DeclareLaunchArgument('delete_db', default_value='true', description='Hapus database peta sebelumnya saat mulai'),
        rtabmap_launch,
        rviz_node
    ])
