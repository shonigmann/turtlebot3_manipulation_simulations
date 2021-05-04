import os

import rclpy.publisher
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, \
    TimerAction, GroupAction, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command  # , TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    ld = LaunchDescription()

    # Get relevant system paths
    pkg_dir = get_package_share_directory('turtlebot3_manipulation_gazebo')
    launch_dir = os.path.join(pkg_dir, 'launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    world_dir = os.path.join(nav2_bringup_dir, 'worlds')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    nav_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                               'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    launch_configs = {
        'urdf_path': [os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'urdf',
                                   'turtlebot3_pi_manipulator.xacro'), 'Path to robot xacro or urdf'],
        'gui': ['True', ''],
        'paused': ['True', ''],
        'use_sim_time': ['True', ''],
        'use_robot_state_pub': ['True', ''],
        'use_rviz': ['True', ''],
        'run_slam': ['False', ''],
        'use_simulator': ['True', 'Whether to start the simulator'],
        'world': [os.path.join(world_dir, 'world_only.model'),
                  'Full path to world model file to load'],
        'x_pose': ['0.5', 'Initial x position of the robot'],
        'y_pose': ['0.0', 'Initial y position of the robot'],
        'z_pose': ['0.0', 'Initial z position of the robot'],
        'Y_pose': ['0.0', 'Initial Y position of the robot'],
        'use_namespace': ['false', 'Whether or not to apply a namespace to the navigation stack'],
        'namespace': ['', 'Top-level navigation stack namespace']
    }

    for c in launch_configs:
        launch_configs[c].append(LaunchConfiguration(c))
        ld.add_action(DeclareLaunchArgument(c, default_value=launch_configs[c][0], description=launch_configs[c][1]))

    gz_controller_yaml = os.path.join(pkg_dir, 'config',
                                      'gazebo_controller.yaml')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', launch_configs['world'][2]],
        output='screen')

    ld.add_action(gazebo)

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # (orduno) Substitute with `PushNodeRemapping` https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # start robot state publisher
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(launch_configs['use_robot_state_pub'][2]),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=launch_configs['namespace'][2],
        output='screen',
        parameters=[
            {'use_sim_time': launch_configs['use_sim_time'][2]},
            {'robot_description': Command(['xacro', ' ', launch_configs['urdf_path'][2], ' gazebo:=False'])}
        ],
        remappings=remappings,
    )

    spawn_model = Node(
        package='tb3_manipulation_gazebo_spawner',
        executable='tb3_manipulation_gazebo_spawner',
        output='screen',
        arguments=[
            '--robot_name', 'tb0',
            '--robot_namespace', launch_configs['namespace'][2],
            '--urdf', launch_configs['urdf_path'][2],
            '-x', launch_configs['x_pose'][2],
            '-y', launch_configs['y_pose'][2],
            '-z', launch_configs['z_pose'][2],
            # '-Y', launch_configs['Y_pose'][2],
            # '-p', gz_controller_yaml])
        ]
    )

    # Manipulator Control Items:
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller'],
        output='screen'
    )

    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'effort_controllers'],
        output='screen'
    )

    # RVIZ Launch Items:
    load_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'rviz_launch.py')),
        condition=IfCondition(launch_configs['use_rviz'][2]),
        launch_arguments=[
            ('namespace', launch_configs['namespace'][2]),
            ('use_namespace', launch_configs['use_namespace'][2]),
            ('rviz_config', rviz_config_file)
        ]
    )

    # Navigation 2:
    load_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments=[
            ('namespace', launch_configs['namespace'][2]),
            ('use_namespace', launch_configs['use_namespace'][2]),
            ('slam', launch_configs['run_slam'][2]),
            ('map', map_yaml_file),
            ('use_sim_time', launch_configs['use_sim_time'][2]),
            ('params_file', nav_params_file),
            ('default_bt_xml_filename', bt_xml_file),
            ('autostart', 'true'),
        ]
    )

    actions = [
        # start_robot_state_publisher_cmd,
        TimerAction(actions=[spawn_model], period=1.5),
        # RegisterEventHandler(event_handler=OnProcessExit(
        #     target_action=spawn_model,
        #     on_exit=[load_joint_state_controller]
        # )),
        # RegisterEventHandler(event_handler=OnProcessExit(
        #     target_action=load_joint_state_controller,
        #     on_exit=[load_joint_trajectory_controller, load_effort_controller]
        # )),
        # load_rviz,
        # load_nav2
    ]

    for action in actions:
        ld.add_action(action)

    return ld
