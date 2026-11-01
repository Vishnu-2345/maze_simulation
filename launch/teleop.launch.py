import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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

    return LaunchDescription([
        rviz_arg,
        sim_launch
    ])
