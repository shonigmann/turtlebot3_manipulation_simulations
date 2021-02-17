import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os


def generate_launch_description():
    ld = launch.LaunchDescription()
    arm_yaml_path = os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'config',
                                 'arm_controller.yaml')
    gripper_yaml_path = os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'config',
                                     'arm_controller.yaml')
    arm_controller_node = launch_ros.actions.Node(
        package='controller_manager',
        name='arm_controller_spawner',
        executable='controller_manager',
        parameters=[arm_yaml_path],
        arguments="spawn arm_controller",
    )  # TODO: TEST
    gripper_controller_node = launch_ros.actions.Node(
        package='controller_manager',
        name='gripper_controller_spawner',
        executable='controller_manager',
        parameters=[gripper_yaml_path],
        arguments="spawn gripper_controller",
    )  # TODO: TEST

    ld.add_action(arm_controller_node)
    ld.add_action(gripper_controller_node)

    return ld

# <launch>
#   <!-- arm controller -->
#   <rosparam file="$(find turtlebot3_manipulation_gazebo)/config/arm_controller.yaml" command="load"/>
#   <node name="arm_controller_spawner" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="spawn arm_controller"/>
#
#   <!-- gripper controller -->
#   <rosparam file="$(find turtlebot3_manipulation_gazebo)/config/gripper_controller.yaml" command="load"/>
#   <node name="gripper_controller_spawner" pkg="controller_manager" type="controller_manager" respawn="false" output="screen" args="spawn gripper_controller"/>
# </launch>