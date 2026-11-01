import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    try:
        pkg_share = get_package_share_directory('maze_simulation')
        sim_launch_path = os.path.join(pkg_share, 'launch', 'maze_sim.launch.py')
        slam_params_path = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'nav2_view.rviz')
    except Exception:
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        sim_launch_path = os.path.join(base_dir, 'launch', 'maze_sim.launch.py')
        slam_params_path = os.path.join(base_dir, 'config', 'slam_toolbox_params.yaml')
        rviz_config_path = os.path.join(base_dir, 'rviz', 'nav2_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz2 visualization'
    )

    # Launch Gazebo Sim with Robot (disable its default rviz since SLAM opens nav2_view)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path),
        launch_arguments={'rviz': 'false', 'use_sim_time': use_sim_time}.items()
    )

    # SLAM Toolbox Asynchronous Mapping Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz2 with SLAM & Navigation layout
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        sim_launch,
        slam_node,
        rviz_node
    ])
