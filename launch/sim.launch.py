import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('robot_sim')
    
    # Resolusi nama world
    world_arg = LaunchConfiguration('world').perform(context)
    gui_arg = LaunchConfiguration('gui').perform(context)
    x_arg = LaunchConfiguration('x').perform(context)
    y_arg = LaunchConfiguration('y').perform(context)
    z_arg = LaunchConfiguration('z').perform(context)
    yaw_arg = LaunchConfiguration('yaw').perform(context)

    if world_arg in ['world_a', 'basic', 'a']:
        world_file = os.path.join(pkg_share, 'worlds', 'world_a_basic.world')
    elif world_arg in ['world_b', 'corridor', 'b']:
        world_file = os.path.join(pkg_share, 'worlds', 'world_b_corridor.world')
        # shortcut: spawn di lorong selatan agar tidak terjebak pilar tengah jika y tidak diatur
        if y_arg == '0.0':
            y_arg = '-3.8'
    elif world_arg in ['world_c', 'classroom', 'lab', 'c']:
        world_file = os.path.join(pkg_share, 'worlds', 'world_c_classroom.world')
    elif os.path.exists(world_arg):
        world_file = world_arg
    else:
        # Default fallback
        world_file = os.path.join(pkg_share, 'worlds', 'world_a_basic.world')

    # Convert Xacro ke Robot Description
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 2. Gazebo Simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': gui_arg, 'pause': 'false'}.items()
    )

    # 3. Spawn Robot Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'exploration_robot',
            '-x', x_arg,
            '-y', y_arg,
            '-z', z_arg,
            '-Y', yaw_arg
        ],
        output='screen'
    )

    # 4. PID Velocity Controller (Anti-Oversteering & Soft Motion)
    pid_controller = Node(
        package='robot_sim',
        executable='pid_velocity_controller.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return [
        robot_state_publisher,
        gazebo,
        spawn_entity,
        pid_controller
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='world_a', description='Pilihan dunia simulasi: world_a, world_b, world_c'),
        DeclareLaunchArgument('gui', default_value='true', description='Tampilkan Gazebo GUI'),
        DeclareLaunchArgument('x', default_value='0.0', description='Posisi awal spawn X (meter)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Posisi awal spawn Y (meter)'),
        DeclareLaunchArgument('z', default_value='0.05', description='Posisi awal spawn Z (meter)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Orientasi awal spawn Yaw (radian)'),
        OpaqueFunction(function=launch_setup)
    ])
