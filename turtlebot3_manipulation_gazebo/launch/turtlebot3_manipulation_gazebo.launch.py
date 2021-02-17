import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    ld = launch.LaunchDescription()

    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('gui')

    declare_model_cmd = DeclareLaunchArgument(
        'model',
        default_value=os.getenv('TURTLEBOT3_MODEL'),
        description='model type [waffle, waffle_pi]')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='')

    declare_paused_cmd = DeclareLaunchArgument(
        'paused',
        default_value='True',
        description='')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='')

    gz_controller_yaml = os.path.join(get_package_share_directory('turtlebot3_manipulation_gazebo'), 'config',
                                      'gazebo_controller.yaml')

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_paused_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_gui_cmd)

    return ld

# <launch>
#   <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [waffle, waffle_pi]"/>
#   <arg name="gui" default="true"/>
#   <arg name="paused" default="true"/>
#   <arg name="use_sim_time" default="true"/>
#
#   <rosparam file="$(find turtlebot3_manipulation_gazebo)/config/gazebo_controller.yaml" command="load"/>
#
#   <!-- startup simulated world -->
#   <include file="$(find gazebo_ros)/launch/empty_world.launch">
#     <arg name="paused" value="$(arg paused)"/>
#     <arg name="gui" value="$(arg gui)"/>
#     <arg name="use_sim_time" value="$(arg use_sim_time)"/>
#   </include>
#
#   <!-- send robot urdf to param server -->
#   <include file="$(find turtlebot3_manipulation_description)/launch/turtlebot3_manipulation_upload.launch">
#     <arg name="model" value="$(arg model)"/>
#   </include>
#
#   <!-- push robot_description to factory and spawn robot in gazebo -->
#   <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
#     args="-urdf -param robot_description -model robot -x 0.0 -y 0.0 -Y 0.0 -J joint1 0.0 -J joint2 0.0 -J joint3 0.0 -J joint4 0.0 -J gripper 0.0 -J gripper_sub 0.0"/>
#
#   <!-- controller utils -->
#   <include file="$(find turtlebot3_manipulation_gazebo)/launch/controller_utils.launch"/>
#
#   <!-- run controllers -->
#   <include file="$(find turtlebot3_manipulation_gazebo)/launch/turtlebot3_manipulation_controller.launch"/>
#
# </launch>