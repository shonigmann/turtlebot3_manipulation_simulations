import os
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
    world_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_namespaced_view.rviz')
    map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    nav_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_multirobot_params_{}.yaml')
    bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                               'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Number of Robots to Simulate (work in progress)
    n_robots = 1
    # Set names and poses of the robots to be spawned. More parameters can be added later
    robots = [{'name': 'tb'+str(i), 'namespace': 'tb'+str(i), 'x_pose': 0.0, 'y_pose': (i - n_robots) / 2,
               'z_pose': 0.01, 'nav_params_file': nav_params_file.format(i)} for i in range(n_robots)]

    use_namespace = 'true'
    # if n_robots == 1:
    #     robots[0]['namespace'] = ''
    #     # use_namespace = 'false'

    # # TODO: add path to gazebo_ros2_control to plugin path for gazebo!
    # #  https://github.com/ros-simulation/gazebo_ros2_control
    # if os.getenv('GAZEBO_MODEL_PATH') is not None:
    #     os.environ['GAZEBO_MODEL_PATH'] = get_package_share_directory('turtlebot3_gazebo') + '/models' + os.pathsep + \
    #                                       pkg_dir + '/models' \
    #                                       + os.pathsep + os.getenv('GAZEBO_MODEL_PATH')
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] = get_package_share_directory('turtlebot3_gazebo') + '/models' + os.pathsep + \
    #                                       pkg_dir + '/models'
    #
    # if os.getenv('GAZEBO_PLUGIN_PATH') is not None:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = '/home/robo/dev_ws/build/gazebo_ros2_control' + \
    #                                        os.pathsep + os.getenv('GAZEBO_PLUGIN_PATH')
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = '/home/robo/dev_ws/build/gazebo_ros2_control'

    # TODO: consider reverting to more common approach; more lines of code, but probably more readable. Else consider
    #  creating a class with required fields rather than relying on numerical indexing
    #  The trouble with this method is that you don't know what is actually being used
    launch_configs = {'urdf_path': [os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'urdf',
                                                 'turtlebot3_pi_manipulator.xacro'), 'Path to robot xacro or urdf'],
                      'gui': ['True', ''],
                      'paused': ['True', ''],
                      'use_sim_time': ['True', ''],
                      'use_robot_state_pub': ['True', ''],
                      'use_rviz': ['True', ''],
                      'run_slam': ['True', ''],
                      'use_simulator': ['True', 'Whether to start the simulator'],
                      'world': [os.path.join(world_dir, 'empty_worlds', 'empty.model'),
                                'Full path to world model file to load'],
                      # 'x_pose': ['0.0', 'Initial x position of the robot'],
                      # 'y_pose': ['0.0', 'Initial y position of the robot'],
                      # 'z_pose': ['0.0', 'Initial z position of the robot'],
                      # 'Y_pose': ['0.0', 'Initial Y position of the robot'],
                      # 'J1_pose': ['0.0', 'Initial Joint 1 position of the robot'],
                      # 'J2_pose': ['0.0', 'Initial Joint 2 position of the robot'],
                      # 'J3_pose': ['0.0', 'Initial Joint 3 position of the robot'],
                      # 'J4_pose': ['0.0', 'Initial Joint 4 position of the robot'],
                      # 'G_pose': ['0.0', 'Initial Gripper position of the robot'],
                      # 'Gs_pose': ['0.0', 'Initial sub-Gripper position of the robot']
                      }

    for c in launch_configs:
        launch_configs[c].append(LaunchConfiguration(c))
        ld.add_action(DeclareLaunchArgument(c, default_value=launch_configs[c][0], description=launch_configs[c][1]))

    # for the time being, universal gains are being used for all robots. Would need new approach/yaml files for
    # heterogeneous control
    # TODO: find out how namespace affects joint names in yaml (e.g. does the yaml file need /robot1/joint1 or is
    #  joint1 sufficient
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

    for robot in robots:
        # start robot state publisher
        start_robot_state_publisher_cmd = Node(
            condition=IfCondition(launch_configs['use_robot_state_pub'][2]),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher ',  # _{}'.format(robot['name']),  TODO: SEE IF NODE NEEDS UNIQUE NAME OR IF GROUP ADDS NAMESPACE
            namespace=robot['namespace'],  # TODO: see if '/' is required
            output='screen',
            parameters=[
                {'use_sim_time': launch_configs['use_sim_time'][2]},
                {'robot_description': Command(['xacro', ' ', launch_configs['urdf_path'][2], ' gazebo:=False'])}
            ],
            remappings=remappings,
        )  # TODO: consider supporting heterogeneous robot groups, changing urdf path in robots instead of in configs

        spawn_model = Node(
            package='tb3_manipulation_gazebo_spawner',
            executable='tb3_manipulation_gazebo_spawner',
            output='screen',
            arguments=[
                '--robot_name', robot['name'],
                '--robot_namespace', robot['namespace'],
                '--urdf', launch_configs['urdf_path'][2],
                '-x', str(robot['x_pose']),  # TODO: consider adding commandline interface value... or removing altogether
                '-y', str(robot['y_pose']),
                '-z', str(robot['z_pose']),
                # '-Y', launch_configs['Y_pose'][2],
                # '-J1', launch_configs['J1_pose'][2],
                # '-J2', launch_configs['J2_pose'][2],
                # '-J3', launch_configs['J3_pose'][2],
                # '-J4', launch_configs['J4_pose'][2],
                # '-G', launch_configs['G_pose'][2],
                # '-Gs', launch_configs['Gs_pose'][2],
                '-p', gz_controller_yaml])
        #   <!-- controller utils -->
        #   <include file="$(find turtlebot3_manipulation_gazebo)/launch/controller_utils.launch"/>
        #   <!-- run controllers -->
        #   <include file="$(find turtlebot3_manipulation_gazebo)/launch/turtlebot3_manipulation_controller.launch"/>

        # TODO: probably safe to remove these
        # cmd_utils = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'controller_utils.launch.py')))  # TEST
        #
        # cmd_controller = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(launch_dir,
        #                                                'turtlebot3_manipulation_controller.launch.py')))  # TEST

        # add actions to launch description and return

        # Manipulator Control Items:
        # TODO: figure out controller namespaces
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

        # NAV2 Launch Items:
        load_rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'rviz_launch.py')),
            condition=IfCondition(launch_configs['use_rviz'][2]),
            launch_arguments=[
                ('namespace', robot['namespace']),
                ('use_namespace', use_namespace),
                ('rviz_config', rviz_config_file)
            ]
        )

        # TODO: launch nav2_slam with params, map_yaml, namespace, behaviour tree filename, autostart
        load_nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
            launch_arguments=[
                ('namespace', robot['namespace']),
                ('use_namespace', use_namespace),
                ('slam', launch_configs['run_slam'][2]),
                ('use_sim_time', launch_configs['use_sim_time'][2]),
                ('autostart', 'true'),
                ('map', map_yaml_file),
                ('params_file', robot['nav_params_file']),
                ('default_bt_xml_filename', bt_xml_file),
            ]
        )

        ld.add_action(LogInfo(msg="=================================="))
        ld.add_action(LogInfo(msg="========= Launching {} ========== ".format(robot['name'])))
        ld.add_action(LogInfo(msg="======== With Params {} ========= ".format(robot['nav_params_file'])))
        ld.add_action(LogInfo(msg="=================================="))
        actions = [
            start_robot_state_publisher_cmd,
            TimerAction(actions=[spawn_model], period=2.0),
            RegisterEventHandler(event_handler=OnProcessExit(
                target_action=spawn_model,
                on_exit=[load_joint_state_controller]
            )),
            RegisterEventHandler(event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller, load_effort_controller]
            )),
            load_rviz,
            load_nav2
        ]
        ld.add_action(GroupAction(actions))

        # if n_robots > 1:
        #     ld.add_action(GroupAction(actions))
        # else:
        #     for action in actions:
        #         ld.add_action(action)

    # TODO: probably safe to remove these:
    # ld.add_action(cmd_utils)
    # ld.add_action(cmd_controller)

    return ld
