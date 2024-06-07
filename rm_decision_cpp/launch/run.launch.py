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

"""
Example for spawning multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks.
"""

import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node,LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_decision_cpp'), 'config', 'node_params.yaml')


    # Create the launch description and populate
    ld = LaunchDescription()
    demo_cmd = Node(
        package='rm_decision_cpp',
        name='tree_exec',
        executable='tree_exec_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen')
    ld.add_action(demo_cmd)

    return ld