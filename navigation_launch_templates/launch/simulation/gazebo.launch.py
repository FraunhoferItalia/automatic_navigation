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
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from fhi_ros2.launch import get_launcher


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context)

    gzserver = IncludeLaunchDescription(
        get_launcher("gazebo_ros", "gzserver"),
        launch_arguments={
            "world": world,
            "verbose": "true",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        get_launcher("gazebo_ros", "gzclient"),
    )

    robot_spawner = Node(
        name="spawner",
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "robot",
        ],
    )
    if gui == "true":
        return [
            gzserver,
            gzclient,
            robot_spawner,
        ]
    else:
        return [
            gzserver,
            robot_spawner,
        ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="",
                description="Allow to select the world description file",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Allow to select the world description file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
