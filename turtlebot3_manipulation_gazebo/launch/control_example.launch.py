import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, \
    IncludeLaunchDescription  # , TimerAction, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command  # , TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    ld = LaunchDescription()

    # Get relevant system paths
    pkg_dir = get_package_share_directory('turtlebot3_manipulation_gazebo')
    autostart = 'True'

    launch_configs = {
        'urdf_path': [os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'urdf',
                                   'turtlebot3_pi_manipulator.xacro'), 'Path to robot xacro or urdf'],
        'gui': ['True', ''],
        'use_sim_time': ['True', ''],
        'use_robot_state_pub': ['True', ''],
        'use_simulator': ['True', 'Whether to start the simulator'],
        'x_pose': ['0.5', 'Initial x position of the robot'],
        'y_pose': ['0.0', 'Initial y position of the robot'],
        'z_pose': ['0.1', 'Initial z position of the robot'],
        'Y_pose': ['0.0', 'Initial Y position of the robot'],
        'namespace': ['', 'Top-level navigation stack namespace'],
        'log_level': ['WARN', 'Logging level']
    }

    for c in launch_configs:
        launch_configs[c].append(LaunchConfiguration(c))
        ld.add_action(DeclareLaunchArgument(c, default_value=launch_configs[c][0], description=launch_configs[c][1]))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
        launch_arguments=[('verbose', 'True'),
                          ]
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py']),
        launch_arguments=[('verbose', 'True')]
    )

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # start robot state publisher
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(launch_configs['use_robot_state_pub'][2]),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': launch_configs['use_sim_time'][2]},
            {'robot_description': Command(['xacro ',
                                           launch_configs['urdf_path'][2],
                                           ' gazebo:=True run_arm_control:=True '])}
        ],
        remappings=remappings,
        arguments=['--log-level', launch_configs['log_level'][2]]
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
            '-Y', launch_configs['Y_pose'][2],
            '--log-level', launch_configs['log_level'][2],
            '--xacro_args', 'gazebo:=True run_arm_control:=True'
        ]
    )

    # Manipulator Control Items:
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_broadcaster'],
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

    # bundling all the launch actions and adding to the launch description
    actions = [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_model,
            on_exit=[load_joint_state_broadcaster]
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_trajectory_controller, load_effort_controller]
        )),
        gzserver,
        gzclient,
        start_robot_state_publisher_cmd,
        spawn_model,
    ]

    for action in actions:
        ld.add_action(action)

    return ld
