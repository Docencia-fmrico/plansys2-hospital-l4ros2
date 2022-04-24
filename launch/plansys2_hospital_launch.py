# Copyright 2019 Intelligent Robotics Lab
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
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    hospital_dir = get_package_share_directory('plansys2_hospital_l4ros2')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': hospital_dir + '/pddl/hospital_domain.pddl'}.items()
        )
    """
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('br2_navigation'),
            'launch',
            'tiago_navigation.launch.py')),
        launch_arguments={
            'map': os.path.join(hospital_dir, 'maps', 'hospital_map.yaml')
        }.items())
    """
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            hospital_dir,
            'launch',
            'sim_hospital.launch.py'))
            )
    """
    fake_nav2_cmd = Node(
        package='plansys2_hospital_l4ros2',
        executable='nav2_sim_node',
        name='nav2_sim_node',
        output='screen',
        parameters=[])
    """
    # Specify the actions
    move_cmd = Node(
        package='plansys2_hospital_l4ros2',
        executable='move_action_node',
        name='move_action_node',
        output='screen',
        parameters=[os.path.join(hospital_dir, 'config', 'coords.yaml')])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(gazebo_cmd)
    # ld.add_action(nav2_cmd)
    ld.add_action(move_cmd)

    return ld
