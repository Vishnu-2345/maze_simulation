import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Attempt to locate package share, with fallback to local path
    try:
        pkg_share = get_package_share_directory('maze_simulation')
        default_world_path = os.path.join(pkg_share, 'worlds', 'maze.sdf')
    except Exception:
        default_world_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'worlds', 'maze.sdf')
        )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world SDF file to load into Gazebo Harmonic'
    )

    gz_sim_share = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items()
    )

    return LaunchDescription([
        world_arg,
        gz_sim
    ])

