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

    xacro_args = ' offroad_turtlebot:=True scale_factor:=2.0 flashlight:=True'

    ld = LaunchDescription()

    # Get relevant system paths
    pkg_dir = get_package_share_directory('turtlebot3_manipulation_gazebo')
    # apriltag_dir = get_package_share_directory('apriltag_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_system_test_dir = get_package_share_directory('nav2_system_tests')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    #spaceros_dir = get_package_share_directory('spaceros_gazebo')
    #world_path = os.path.join(spaceros_dir, 'worlds', 'turtlebot_world', 'turtlebot_world.world')

    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    nav_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                               'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    autostart = 'True'

    launch_configs = {
        'urdf_path': [os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'urdf',
                                   'turtlebot3_pi_manipulator.xacro'), 'Path to robot xacro or urdf'],
        'gui': ['True', ''],
        'use_sim_time': ['True', ''],
        'use_robot_state_pub': ['True', ''],
        'use_rviz': ['True', ''],
        'run_slam': ['True', ''],
        'use_simulator': ['True', 'Whether to start the simulator'],
       # 'world': [world_path, 'Full path to world model file to load'],
        'x_pose': ['0.5', 'Initial x position of the robot'],
        'y_pose': ['0.0', 'Initial y position of the robot'],
        'z_pose': ['0.1', 'Initial z position of the robot'],
        'Y_pose': ['0.0', 'Initial Y position of the robot'],
        'use_namespace': ['False', 'Whether or not to apply a namespace to the navigation stack'],
        'namespace': ['', 'Top-level navigation stack namespace'],
        'log_level': ['WARN', 'Logging level']
    }

    for c in launch_configs:
        launch_configs[c].append(LaunchConfiguration(c))
        ld.add_action(DeclareLaunchArgument(c, default_value=launch_configs[c][0], description=launch_configs[c][1]))

    if os.getenv('GAZEBO_MODEL_PATH') is not None:
        os.environ['GAZEBO_MODEL_PATH'] = os.path.join(nav2_system_test_dir, '/models', os.pathsep,
                                                       #spaceros_dir, '/models', os.pathsep,
                                                       os.getenv('GAZEBO_MODEL_PATH'))
    else:
        os.environ['GAZEBO_MODEL_PATH'] = os.path.join(nav2_system_test_dir, '/models', os.pathsep,
                                                       #spaceros_dir, '/models', os.pathsep)
							)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
        launch_arguments=[('verbose', 'True'),
                          ('pause', 'True'),
                          #('world', launch_configs['world'][2])
                          ]
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py']),
        launch_arguments=[('verbose', 'True')]
    )

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
        # namespace=launch_configs['namespace'][2],
        output='screen',
        parameters=[
            {'use_sim_time': launch_configs['use_sim_time'][2]},
            {'robot_description': Command(['xacro ',
                                           launch_configs['urdf_path'][2],
                                           ' gazebo:=False ' + xacro_args])}
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
            # '-p', gz_controller_yaml]),
            '--log-level', launch_configs['log_level'][2],
            '--xacro_args', ' gazebo:=True run_arm_control:=True ' + xacro_args
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
            ('autostart', autostart),
        ]
    )

    # April Tag
    #load_apriltag = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(os.path.join(apriltag_dir, 'launch', 'tag_25h9_all.launch.py'))
    #)

    # TODO: write node that randomly places AR cubes in the environment; being able to customize the size and modify
    #  the apriltag launch config and the urdf would be nice

    # TODO: write node that:
    #  load_commander = Node(
    #         package='spaceros_demo',
    #         executable='tb3m_commander',
    #         name='tb3m_commander',
    #         output='screen',
    #         remappings=remappings,
    #         arguments=['--log-level', launch_configs['log_level'][2]]
    #     )
    #  A) explores the environment, within some radius of the starting point
    #  B) listens for apriltag detections
    #  C) navigates to object when it is detected
    #  D) attempts to pickup the object once it is in the correct position
    #  D) navigates to bin
    #  E) places block in bin

    unpause_sim = ExecuteProcess(cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty'],
                                 output='screen')

    actions = [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_model,
            on_exit=[load_joint_state_controller, load_rviz, load_nav2, unpause_sim]  # , load_apriltag]
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
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
