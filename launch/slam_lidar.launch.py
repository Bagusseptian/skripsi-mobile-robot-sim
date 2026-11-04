import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    open_rviz = LaunchConfiguration('rviz')

    default_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_lidar.rviz')

    # Node: SLAM Toolbox (Mode Asinkron untuk Pemetaan 2D Real-Time)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Node: RViz2 untuk Visualisasi Peta dan Scan
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan clock waktu simulasi Gazebo'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path lengkap ke file parameter slam_toolbox'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Apakah membuka RViz2 secara otomatis'
        ),
        slam_toolbox_node,
        rviz_node
    ])
