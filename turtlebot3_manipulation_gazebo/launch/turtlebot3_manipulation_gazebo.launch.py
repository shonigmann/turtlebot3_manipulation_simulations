import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, \
    TimerAction, GroupAction
# LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    ld = LaunchDescription()

    # Get relevant system paths
    pkg_dir = get_package_share_directory('turtlebot3_manipulation_gazebo')
    launch_dir = os.path.join(pkg_dir, 'launch')
    world_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # TODO: revisit when looking at multi-robot collaboration
    n_robots = 1

    # Set names and poses of the robots to be spawned. More parameters can be added later
    robots = [{'name': 'tb'+str(i), 'namespace': '/tb'+str(i), 'x_pose': 0.0, 'y_pose': (i - n_robots) / 2,
               'z_pose': 0.01} for i in range(n_robots)]
    if n_robots == 1:
        robots[0]['namespace'] = ''
        # TODO: REVISIT NAMESPACING WHEN CONSIDERING MULTI BOT

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
                      # 'robot_name': ['', 'robot'],
                      # 'namespace': ['', ''],
                      'use_robot_state_pub': ['True', ''],
                      'use_rviz': ['False', ''],
                      'run_slam': ['False', ''],
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
            name='robot_state_publisher',
            namespace='/' + robot['name'],  # TODO: see if '/' is required
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
                '-x', robot['x'],  # TODO: consider adding commandline interface value... or removing altogether
                '-y', robot['y'],
                '-z', robot['z'],
                '-Y', launch_configs['Y_pose'][2],
                '-J1', launch_configs['J1_pose'][2],
                '-J2', launch_configs['J2_pose'][2],
                '-J3', launch_configs['J3_pose'][2],
                '-J4', launch_configs['J4_pose'][2],
                '-G', launch_configs['G_pose'][2],
                '-Gs', launch_configs['Gs_pose'][2],
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
        ld.add_action(start_robot_state_publisher_cmd)
        ld.add_action(TimerAction(actions=[spawn_model], period=2.0))

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

        ld.add_action(RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn_model,
            on_exit=[load_joint_state_controller]
        )))
        ld.add_action(RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller, load_effort_controller]
        )))


    # TODO: probably safe to remove these:
    # ld.add_action(cmd_utils)
    # ld.add_action(cmd_controller)

    return ld
