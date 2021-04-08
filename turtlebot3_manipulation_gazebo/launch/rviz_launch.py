# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command  # , TextSubstitution


def generate_launch_description():
    config_path = '/home/robo/.rviz/launchtest.rviz'
    config_path2 = '/home/robo/dev_ws/src/navigation2/nav2_bringup/bringup/rviz/nav2_default_view.rviz'

    # Launch rviz node
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_path2],
        output='screen')

    # # run rviz command:
    # os.system('ros2 run rviz2 rviz2 -d ' + config_path +
    #                                    ' --ros-args --remap __node:=rviz2_test2')
    run_rviz_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', config_path2, '--ros-args', '--remap', '__node:=rviz2_test2'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription([launch_rviz_node, run_rviz_cmd])

    return ld
