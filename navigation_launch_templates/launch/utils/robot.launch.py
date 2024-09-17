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
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    controllers = LaunchConfiguration("controllers").perform(context)
    simulation = LaunchConfiguration("simulation").perform(context)
    urdf = LaunchConfiguration("urdf").perform(context)

    use_sim_time = simulation == "true"
    robot_description = xacro.process_file(urdf).toxml()
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return [robot_state_publisher, joint_state_broadcaster]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf",
                default_value="",
                description="Allow to select the urdf of the robot",
            ),
            DeclareLaunchArgument(
                "simulation",
                default_value="true",
                choices=["true", "false"],
                description="Allow to use the simulation",
            ),
            DeclareLaunchArgument(
                "controllers",
                default_value="",
                description="Allow to select the controllers configuration file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
