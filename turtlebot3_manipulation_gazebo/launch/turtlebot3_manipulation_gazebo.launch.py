import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    launch_dir = os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'launch')
    model_dir = os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'models')
    tb3_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds')

    # TODO: add path to gazebo_ros2_control to plugin path for gazebo!
    #  https://github.com/ros-simulation/gazebo_ros2_control
    if os.getenv('GAZEBO_MODEL_PATH') is not None:
        os.environ['GAZEBO_MODEL_PATH'] = get_package_share_directory('turtlebot3_gazebo') + '/models' + os.pathsep + \
                                          get_package_share_directory('turtlebot3_manipulation_gazebo') + '/models' \
                                          + os.pathsep + os.getenv('GAZEBO_MODEL_PATH')
    else:
        os.environ['GAZEBO_MODEL_PATH'] = get_package_share_directory('turtlebot3_gazebo') + '/models' + os.pathsep + \
                                          get_package_share_directory('turtlebot3_manipulation_gazebo') + '/models'

            #   <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [waffle, waffle_pi]"/>
    #   <arg name="gui" default="true"/>
    #   <arg name="paused" default="true"/>
    #   <arg name="use_sim_time" default="true"/>
    launch_configs = {'model': [os.getenv('TURTLEBOT3_MODEL'), 'model type [waffle, waffle_pi]'],
                      # 'gui': ['True', ''],
                      'paused': ['True', ''],
                      'use_sim_time': ['True', ''],
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

    #   <rosparam file="$(find turtlebot3_manipulation_gazebo)/config/gazebo_controller.yaml" command="load"/>
    gz_controller_yaml = os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'config',
                                      'gazebo_controller.yaml')

    if os.getenv('GAZEBO_MODEL_PATH') is not None:
        os.environ['GAZEBO_MODEL_PATH'] = get_package_share_directory('turtlebot3_gazebo') + '/models' + os.pathsep + \
                                          get_package_share_directory('turtlebot3_manipulation_gazebo') + '/models' + \
                                          os.pathsep + \
                                          os.getenv('GAZEBO_MODEL_PATH')
                                          # get_package_share_directory('spaceros_gazebo') + '/models' + os.pathsep + \
    else:
        os.environ['GAZEBO_MODEL_PATH'] = get_package_share_directory('turtlebot3_gazebo') + '/models' + os.pathsep + \
                                          get_package_share_directory('turtlebot3_manipulation_gazebo') + '/models'
                                          # get_package_share_directory('spaceros_gazebo') + '/models' + os.pathsep + \


    #   <!-- startup simulated world -->
    #   <include file="$(find gazebo_ros)/launch/empty_world.launch">
    #     <arg name="paused" value="$(arg paused)"/>
    #     <arg name="gui" value="$(arg gui)"/>
    #     <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    #   </include>
    #
    # start_gazebo_server_cmd = ExecuteProcess(
    #     condition=IfCondition(launch_configs['use_simulator'][2]),
    #     cmd=['gzserver', '-s', 'libgazebo_ros_init.so', launch_configs['world'][2]],
    #     output='screen')  # , cwd=[launch_dir],
    #     # arguments=['-pause', launch_configs['paused'][2]])
    #

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', launch_configs['world'][2]],
        output='screen')

    # start_gazebo_client_cmd = ExecuteProcess(
    #     condition=IfCondition(PythonExpression([launch_configs['use_simulator'][2]])),
    #     cmd=['gzclient'],
    #     output='screen')  # cwd=[launch_dir],

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # (orduno) Substitute with `PushNodeRemapping` https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # TODO: Allow for "none" option (e.g. only the manipulator?)
    urdf = os.path.join(model_dir, 'turtlebot3_waffle_manipulator', 'model.urdf')

    # TODO: readd when working
    # start robot state publisher
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(launch_configs['use_robot_state_pub'][2]),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=launch_configs['namespace'][2],
        output='screen',
        parameters=[{'use_sim_time': launch_configs['use_sim_time'][2]}],
        remappings=remappings,
        arguments=[urdf])

    #   <!-- push robot_description to factory and spawn robot in gazebo -->
    #   <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    #     args="-urdf -param robot_description -model robot -x 0.0 -y 0.0 -Y 0.0 -J joint1 0.0 -J joint2 0.0 -J joint3 0.0 -J joint4 0.0 -J gripper 0.0 -J gripper_sub 0.0"/>

    spawn_model = Node(
        package='tb3_manipulation_gazebo_spawner',
        executable='tb3_manipulation_gazebo_spawner',
        output='screen',
        arguments=[
            '--robot_name', launch_configs['model'][2],
            '--robot_namespace', launch_configs['namespace'][2],
            '--turtlebot_type', launch_configs['model'][2],
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

    # spawn_model = Node(
    #         package='nav2_gazebo_spawner',
    #         executable='nav2_gazebo_spawner',
    #         output='screen',
    #         arguments=[
    #             '--robot_name', "waffle",
    #             '--robot_namespace', "",
    #             '--turtlebot_type', "waffle",
    #             '-x', '0.0',
    #             '-y', '0.0',
    #             '-z', '0.0'])

    # NO LONGER NECESSARY WITH SPAWNER WORKAROUND... TODO: OR IS IT???
    #   <!-- send robot urdf to param server -->
    #   <include file="$(find turtlebot3_manipulation_description)/launch/turtlebot3_manipulation_upload.launch">
    #     <arg name="model" value="$(arg model)"/>
    #   </include>

    #   <!-- controller utils -->
    #   <include file="$(find turtlebot3_manipulation_gazebo)/launch/controller_utils.launch"/>
    #   <!-- run controllers -->
    #   <include file="$(find turtlebot3_manipulation_gazebo)/launch/turtlebot3_manipulation_controller.launch"/>

    cmd_utils = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'controller_utils.launch.py')))     # TEST

    cmd_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                   'turtlebot3_manipulation_controller.launch.py')))  # TEST



    # add actions to launch description and return
    ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_model)
    #
    # ld.add_action(cmd_utils)
    # ld.add_action(cmd_controller)
    return ld


