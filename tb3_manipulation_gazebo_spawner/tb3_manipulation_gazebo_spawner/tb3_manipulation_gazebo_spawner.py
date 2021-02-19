#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Script used to spawn a robot in a generic position."""

import argparse
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy


def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Robot into Gazebo with navigation2')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-Y', type=float, default=0,
                        help='the initial robot yaw position [radians]')
    parser.add_argument('-J1', type=float, default=0,
                        help='the initial manipulator joint-1 position [radians]')
    parser.add_argument('-J2', type=float, default=0,
                        help='the initial manipulator joint-2 position [radians]')
    parser.add_argument('-J3', type=float, default=0,
                        help='the initial manipulator joint-3 position [radians]')
    parser.add_argument('-J4', type=float, default=0,
                        help='the initial manipulator joint-4 position [radians]')
    parser.add_argument('-G', type=float, default=0,
                        help='the initial gripper position [radians]')
    parser.add_argument('-Gs', type=float, default=0,
                        help='the initial sub-gripper position [radians]')
    parser.add_argument('-p', type=str, default='',
                        help='full path location of the desired yaml config file for the robot\'s PID joint '
                             'controllers')  # TODO: do something with this...
    parser.add_argument('-k', '--timeout', type=float, default=10.0,
                        help="Seconds to wait. Block until the future is complete if negative. Don't wait if 0.")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-t', '--turtlebot_type', type=str,
                       choices=['none', 'waffle'])  # TODO: see if i can get just the manipulator spawned before trying to do both manipulator and robot   
    group.add_argument('-s', '--sdf', type=str,
                       help="the path to the robot's model file (sdf)")

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}, {}, {}, {}, {}, {}, {}, {}'.format(
        args.robot_name, args.robot_namespace, args.x, args.y, args.z, args.Y, args.J1, args.J2, args.J3, args.J4,
        args.G, args.Gs))

    # Get path to the robot's sdf file
    if args.turtlebot_type is not None:
        # sdf_file_path = os.path.join(
        #     get_package_share_directory('turtlebot3_manipulation_gazebo'), 'models',
        #     'turtlebot3_{}_manipulator'.format(args.turtlebot_type), 'model.urdf')
        sdf_file_path = '/home/blue-robotics/dev_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_' \
                        'gazebo/models/turtlebot3_waffle_manipulator/model.urdf'  # TODO: REVERT, just testing
    else:
        sdf_file_path = args.sdf

    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the sdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    for plugin in root.iter('plugin'):
        # TODO(orduno) Handle case if an sdf file from non-turtlebot is provided
        # TODO: see if turtlebot3_joint_state or gazebo_ros_control also require remapping...
        if 'turtlebot3_diff_drive' in plugin.attrib.values():  #  or 'gazebo_ros_control' in plugin.attrib.values():
            # The only plugin we care for now is 'diff_drive' which is
            # broadcasting a transform between`odom` and `base_footprint`
            # break

            ros_params = plugin.find('ros')
            ros_tf_remap = ET.SubElement(ros_params, 'remapping')
            ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'


    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')
    request.robot_namespace = args.robot_namespace
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    # TODO: add support for robot's initial yaw
    # request.initial_pose.orientation.x,y,z,w = ...
    # TODO: could also add urdf modifications to allow for initial joint angles to be set and control gains to be set
    
    node.get_logger().info('Sending service request to `/spawn_entity`')
    future = client.call_async(request)

    # TODO: DEBUGGING (tried adding timeout_sec="; might have broken things. Trying to remove altogether)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
