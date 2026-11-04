import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'view_all_sensors.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sensor_viewer',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rviz_node
    ])
