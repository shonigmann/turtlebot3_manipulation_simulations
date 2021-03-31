import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, \
    TimerAction
# LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
# , PythonExpression, TextSubstitution,
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
# import xacro


def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir = get_package_share_directory('turtlebot3_manipulation_gazebo')
    launch_dir = os.path.join(pkg_dir, 'launch')
    tb3_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds')

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

    launch_configs = {'urdf_path': [os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'urdf',
                                                 'turtlebot3_pi_manipulator.xacro'), 'Path to robot xacro, urdf, or sdf'],
                      # 'gui': ['True', ''],
                      'paused': ['True', ''],
                      'use_sim_time': ['True', ''],
                      'robot_name': ['', 'robot'],
                      'namespace': ['', ''],
                      'use_namespace': ['False', ''],
                      'use_robot_state_pub': ['True', ''],
                      'use_rviz': ['False', ''],
                      'use_simulator': ['True', 'Whether to start the simulator'],
                      'world': [os.path.join(tb3_dir, 'empty_worlds', 'empty.model'),
                                'Full path to world model file to load'],
                      'x_pose': ['0.0', 'Initial x position of the robot'],
                      'y_pose': ['0.0', 'Initial y position of the robot'],
                      'z_pose': ['0.0', 'Initial z position of the robot'],
                      'Y_pose': ['0.0', 'Initial Y position of the robot'],
                      'J1_pose': ['0.0', 'Initial Joint 1 position of the robot'],
                      'J2_pose': ['0.0', 'Initial Joint 2 position of the robot'],
                      'J3_pose': ['0.0', 'Initial Joint 3 position of the robot'],
                      'J4_pose': ['0.0', 'Initial Joint 4 position of the robot'],
                      'G_pose': ['0.0', 'Initial Gripper position of the robot'],
                      'Gs_pose': ['0.0', 'Initial sub-Gripper position of the robot']
                      }

    for c in launch_configs:
        launch_configs[c].append(LaunchConfiguration(c))
        ld.add_action(DeclareLaunchArgument(c, default_value=launch_configs[c][0], description=launch_configs[c][1]))

    gz_controller_yaml = os.path.join(pkg_dir, 'config',
                                      'gazebo_controller.yaml')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', launch_configs['world'][2]],
        output='screen')

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
            '--robot_name', launch_configs['robot_name'][2],
            '--robot_namespace', launch_configs['namespace'][2],
            '--urdf', launch_configs['urdf_path'][2],
            '-x', launch_configs['x_pose'][2],
            '-y', launch_configs['y_pose'][2],
            '-z', launch_configs['z_pose'][2],
            '-Y', launch_configs['Y_pose'][2],
            '-J1', launch_configs['J1_pose'][2],
            '-J2', launch_configs['J2_pose'][2],
            '-J3', launch_configs['J3_pose'][2],
            '-J4', launch_configs['J4_pose'][2],
            '-G', launch_configs['G_pose'][2],
            '-Gs', launch_configs['Gs_pose'][2],
            '-p', gz_controller_yaml])

    # NO LONGER NECESSARY WITH SPAWNER WORKAROUND... TODO: OR IS IT???
    #   <!-- send robot urdf to param server -->
    #   <include file="$(find turtlebot3_manipulation_description)/launch/turtlebot3_manipulation_upload.launch">
    #     <arg name="model" value="$(arg model)"/>
    #   </include>

    #   <!-- controller utils -->
    #   <include file="$(find turtlebot3_manipulation_gazebo)/launch/controller_utils.launch"/>
    #   <!-- run controllers -->
    #   <include file="$(find turtlebot3_manipulation_gazebo)/launch/turtlebot3_manipulation_controller.launch"/>

    # cmd_utils = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'controller_utils.launch.py')))  # TEST
    #
    # cmd_controller = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir,
    #                                                'turtlebot3_manipulation_controller.launch.py')))  # TEST

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

    # # TODO: RE-ADD?
    # ld.add_action(RegisterEventHandler(event_handler=OnProcessExit(
    #     target_action=gazebo,
    #     on_exit=[spawn_model]
    # )))
    # ld.add_action(RegisterEventHandler(event_handler=OnProcessExit(
    #     target_action=spawn_model,
    #     on_exit=[load_joint_state_controller]
    # )))
    # ld.add_action(RegisterEventHandler(event_handler=OnProcessExit(
    #     target_action=load_joint_state_controller,
    #     on_exit=[load_joint_trajectory_controller, load_effort_controller]
    # )))

    # add actions to launch description and return
    ld.add_action(gazebo)
    ld.add_action(start_robot_state_publisher_cmd)

    ld.add_action(TimerAction(actions=[spawn_model], period=3.0))
    # ld.add_action(spawn_model)

    # for dbm in debug_msg:
    #     ld.add_action(dbm)
    # TODO: Determine if Timer Actions are needed... Also, review nav2_bringup and other examples to see how they are
    #  doing staged launches of dependent nodes (e.g. robot_spawner after gz_factory)
    # TODO: RE-ADD
    # ld.add_action(TimerAction(actions=[load_joint_state_controller], period=1.0))
    # ld.add_action(TimerAction(actions=[load_joint_trajectory_controller], period=2.0))
    # ld.add_action(TimerAction(actions=[load_effort_controller], period=2.0))
    # ld.add_action(load_joint_state_controller)
    # ld.add_action(load_joint_trajectory_controller)
    # ld.add_action(load_effort_controller)

    # ld.add_action(cmd_utils)
    # ld.add_action(cmd_controller)
    return ld
