import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Resolve package paths with fallback for local workspace development
    try:
        pkg_share = get_package_share_directory('maze_simulation')
        default_world = os.path.join(pkg_share, 'worlds', 'maze.sdf')
        default_urdf = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
        default_bridge_cfg = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
        default_rviz_cfg = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    except Exception:
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        default_world = os.path.join(base_dir, 'worlds', 'maze.sdf')
        default_urdf = os.path.join(base_dir, 'urdf', 'robot.urdf.xacro')
        default_bridge_cfg = os.path.join(base_dir, 'config', 'ros_gz_bridge.yaml')
        default_rviz_cfg = os.path.join(base_dir, 'rviz', 'robot_view.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=default_world)
    x_pose = LaunchConfiguration('x_pose', default='-4.5')
    y_pose = LaunchConfiguration('y_pose', default='-4.5')
    z_pose = LaunchConfiguration('z_pose', default='0.05')
    yaw_pose = LaunchConfiguration('yaw', default='0.0')
    rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world, description='Gazebo world file'
    )
    declare_x = DeclareLaunchArgument('x_pose', default_value='-4.5', description='Spawn X coordinate')
    declare_y = DeclareLaunchArgument('y_pose', default_value='-4.5', description='Spawn Y coordinate')
    declare_z = DeclareLaunchArgument('z_pose', default_value='0.05', description='Spawn Z coordinate')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw orientation')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2')

    # Start Gazebo Sim directly with unpaused flag (-r)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # Robot State Publisher (Processes Xacro -> Robot Description & Static TF)
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', default_urdf
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # ROS-Gazebo Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': default_bridge_cfg,
            'expand_gz_topic_names': True,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn Robot into Gazebo Harmonic via raw string (eliminating topic wait race conditions)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'maze_bot',
            '-string', robot_description_content,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ]
    )

    # Delay spawner by 2.0s to ensure Gazebo world physics & plugins are initialized
    delayed_spawn_robot = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_rviz,
        gz_sim,
        robot_state_publisher,
        bridge,
        delayed_spawn_robot,
        rviz_node
    ])
