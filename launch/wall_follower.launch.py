import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    try:
        pkg_share = get_package_share_directory('maze_simulation')
        sim_launch_path = os.path.join(pkg_share, 'launch', 'maze_sim.launch.py')
    except Exception:
        sim_launch_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), 'maze_sim.launch.py')
        )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()
    )

    wall_follower_node = Node(
        package='maze_simulation',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'forward_speed': 0.25,
            'turn_speed': 0.7,
            'target_wall_dist': 0.45,
            'front_threshold': 0.55,
            'kp_gain': 1.8
        }]
    )

    return LaunchDescription([
        rviz_arg,
        sim_launch,
        wall_follower_node
    ])
