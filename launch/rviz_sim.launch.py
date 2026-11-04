import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'view_all_sensors.rviz')

    # Convert xacro to robot_description
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # 2. Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 3. Lightweight Kinematic & Sensor Simulator Node
    dummy_sim = Node(
        package='robot_sim',
        executable='dummy_robot.py',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 4. RViz2 with pre-configured sensors and camera detection
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        dummy_sim,
        rviz
    ])
