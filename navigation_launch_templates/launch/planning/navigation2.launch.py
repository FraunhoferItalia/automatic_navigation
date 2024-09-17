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
from typing import Any, Dict
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    simulation = LaunchConfiguration("simulation").perform(context)
    params = LaunchConfiguration("planning_params").perform(context)

    use_sim_time = simulation == "true"

    # Needed because is not possible to change the local and global costmap topics
    footprint_manager = Node(
        package="navigation_launch_templates",
        executable="footprint_manager.py",
        name="footprint_manager_node",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_plan_transformer = Node(
        package="navigation_launch_templates",
        executable="nav2_plan_transformer.py",
        name="nav2_plan_transformer_node",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_switch_manager = Node(
        package="navigation_launch_templates",
        executable="nav2_switch_manager.py",
        name="nav2_switch_manager_node",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Cmd Vel Collision checker
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="nav2_collision_monitor_node",
        output="screen",
        parameters=[
            params,
            {"cmd_vel_in_topic": "local_planner/cmd_vel"},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Local Planner
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="nav2_controller_server",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
        remappings={
            "cmd_vel": "local_planner/cmd_vel",
        }.items(),
    )

    # Global Planner
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="nav2_planner_server",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Nav2 Behavior Tree Part
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="nav2_behavior_server",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
        remappings={"cmd_vel": "local_planner/cmd_vel"}.items(),
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="nav2_waypoint_follower",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="nav2_bt_navigator",
        output="screen",
        parameters=[
            params,
            {"use_sim_time": use_sim_time},
        ],
    )

    tools = [
        "nav2_controller_server",
        "nav2_planner_server",
        "nav2_behavior_server",
        "nav2_bt_navigator",
        "nav2_waypoint_follower",
        "nav2_collision_monitor_node",
    ]
    return [
        footprint_manager,
        collision_monitor,
        controller_server,
        planner_server,
        behavior_server,
        waypoint_follower,
        bt_navigator,
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="nav2_lifecycle_manager",
            output="screen",
            respawn=True,
            parameters=[
                {"autostart": True},
                {"node_names": tools},
                # {"attempt_respawn_reconnection": tools},
                {"use_sim_time": use_sim_time},
            ],
        ),
        TimerAction(
            period=10.0,
            actions=[nav2_plan_transformer, nav2_switch_manager],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulation",
                default_value="true",
                choices=["true", "false"],
                description="Allow to use the simulation",
            ),
            DeclareLaunchArgument(
                name="planning_params",
                default_value="",
                description="Allow to select the nao nav2 configuration file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
