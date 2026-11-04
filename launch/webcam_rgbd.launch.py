from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    webcam_node = Node(
        package='robot_sim',
        executable='webcam_rgbd_node.py',
        output='screen'
    )
    return LaunchDescription([
        webcam_node
    ])
