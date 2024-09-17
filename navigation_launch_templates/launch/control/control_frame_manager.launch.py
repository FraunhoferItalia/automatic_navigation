#
# Copyright 2024 Fraunhofer Italia Research
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
#
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from fhi_ros2.launch import get_absolute_file_path


def launch_setup(context, *args, **kwargs):
    parent_frame = LaunchConfiguration("parent_frame").perform(context)
    x = LaunchConfiguration("initial_x_transform").perform(context)
    y = LaunchConfiguration("initial_y_transform").perform(context)
    theta = LaunchConfiguration("initial_theta_transform").perform(context)

    control_frame_manager = Node(
        package="navigation_launch_templates",
        executable="control_frame_manager.py",
        name="control_frame_manager",
        parameters=[
            {"parent_frame": parent_frame},
            {"initial_x_transform": x},
            {"initial_y_transform": y},
            {"initial_theta_transform": theta},
        ],
        output="screen",
    )

    return [control_frame_manager]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parent_frame",
                default_value="",
                description="Name of the parent frame",
            ),
            DeclareLaunchArgument(
                "initial_x_transform",
                default_value="0.0",
                description="Relative x coordinate of the control frame",
            ),
            DeclareLaunchArgument(
                "initial_y_transform",
                default_value="0.0",
                description="Relative y coordinate of the control frame",
            ),
            DeclareLaunchArgument(
                "initial_theta_transform",
                default_value="0.0",
                description="Relative theta coordinate of the control frame",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
