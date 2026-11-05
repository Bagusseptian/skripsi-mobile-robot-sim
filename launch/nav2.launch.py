import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('robot_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_path = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)
    open_rviz = LaunchConfiguration('rviz').perform(context)

    # Jika map tidak ditentukan, gunakan peta default jika ada, atau mode navigasi online (tanpa AMCL statis)
    default_map = os.path.join(pkg_share, 'maps', 'world_a_map.yaml')
    
    actions = []

    if map_path != '' and map_path != 'none':
        # Mode Navigasi Penuh dengan Peta Tersimpan + AMCL Lokalisasi
        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart
            }.items()
        )
        actions.append(bringup_cmd)
    else:
        # Mode Navigasi Bersamaan dengan Online SLAM (slam_toolbox / rtabmap menerbitkan /map)
        navigation_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart
            }.items()
        )
        actions.append(navigation_cmd)

    # Launch RViz2 dengan tampilan Nav2 kustom (Termasuk RGB-D Astra & 3D Pointcloud)
    custom_rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_view.rviz')
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': custom_rviz_config
        }.items(),
        condition=IfCondition(open_rviz)
    )
    actions.append(rviz_cmd)

    # Node: Interaktif Penginderaan Spasial 3D Kamera RGB-D (Publish Point & Obstacle Inspector)
    spatial_node = Node(
        package='robot_sim',
        executable='rgbd_spatial_interactive.py',
        name='rgbd_spatial_interactive',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time == 'true'}]
    )
    actions.append(spatial_node)

    return actions

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path ke file peta map.yaml. Kosongkan jika ingin navigasi langsung bersama SLAM online'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan clock simulasi Gazebo'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path lengkap ke file parameter nav2'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Otomatis memulai lifecycle node Nav2'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Buka RViz2 dengan konfigurasi Nav2'
        ),
        OpaqueFunction(function=launch_setup)
    ])
