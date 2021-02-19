import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription()
    yaml_path = os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'config',
                             'joint_state_controller.yaml')
    node = launch_ros.actions.Node(
        package='controller_manager',
        name='joint_state_controller_spawner',
        executable='ros2_control_node',
        parameters=[yaml_path],
        arguments="spawn joint_state_controller",
    )  # TODO: Figure out how to pass robot_description (urdf or just filepath?)

    ld.add_action(node)

    return ld

# <launch>
#   <!-- start joint state controller -->
#   <rosparam file="$(find turtlebot3_manipulation_gazebo)/config/joint_state_controller.yaml" command="load"/>
#   <node name="joint_state_controller_spawner" pkg="controller_manager" type="controller_manager" output="screen"
#     args="spawn joint_state_controller" respawn="false"/>
# </launch>