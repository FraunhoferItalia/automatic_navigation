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
import copy
import os
from . import PlanningConfiguratorBase
from typing import Dict, Any, List, Tuple
from vehicle_recognition import Vehicle, SensorType
import yaml


class Navigation2Configurator(PlanningConfiguratorBase):
    def __init__(self, info: Dict[str, Any], vehicle: Vehicle, file: str):
        super().__init__(info, vehicle, file)

    def configure(self) -> bool:
        configuration = {}
        with open(self.file, "w") as f:
            has_laser = False
            has_pointcloud = False
            lasers = []
            pointclouds = []
            for sensor in self.vehicle.sensors:
                if sensor.type is SensorType.LASER:
                    lasers += [sensor.name]
                    has_laser = True
                if sensor.type is SensorType.POINTCLOUD:
                    pointclouds += [sensor.name]
                    has_pointcloud = True

            (
                local_costmap,
                global_costmap,
                footprint_forwarder,
            ) = self.generate_nav2_local_and_global_costmap_configuration(lasers)

            planning_tools = [
                self.generate_nav2_collision_monitor_configuration(lasers, pointclouds),
                self.generate_nav2_controller_server_configuration(),
                self.generate_nav2_planner_server_configuration(),
                self.generate_navigation_plan_transformer(),
                ("local_costmap", local_costmap),
                ("global_costmap", global_costmap),
                footprint_forwarder,
                self.generate_nav2_behaviors_configuration(),
                self.generate_nav2_bt_navigator_configuration(),
                self.generate_nav2_waypoint_follower_configuration(),
            ]

            for tool in planning_tools:
                configuration[tool[0]] = tool[1]

            configuration = {
                "**/": configuration,
            }

            yaml.dump(configuration, f, sort_keys=False)

        return True

    def generate_nav2_collision_monitor_configuration(
        self, lasers: List[str] = [], pointclouds: List[str] = []
    ) -> Tuple[str, Any]:
        observation_sources = {}

        if lasers:
            laserscan_topic = lasers[0]
            try:
                laserscan_topic = self.info["laserscan_topic"]
            except KeyError:
                print(
                    "PLANNING WARNING: Laser topic not specified. First laser name will be taken"
                )
                pass
            observation_sources["scan"] = {
                "type": "scan",
                "topic": laserscan_topic,
            }

        if pointclouds:
            pointcloud_topic = pointclouds[0]
            try:
                pointcloud_topic = self.info["pointcloud_topic"]
            except KeyError:
                print(
                    "PLANNING WARNING: Pointcloud merge topic not specified. First pointcloud name will be taken"
                )
                pass
            observation_sources["pointcloud"] = {
                "type": "pointcloud",
                "topic": pointcloud_topic,
            }

        if not lasers and not pointclouds and "laserscan_topic" in self.info.keys():
            print(
                f"PLANNING WARNING: No lasers or pointclouds found, using laserscan topic {self.info['laserscan_topic']}"
            )
            observation_sources["scan"] = {
                "type": "scan",
                "topic": self.info["laserscan_topic"],
            }

        if len(observation_sources.keys()) == 0:
            print(
                f"PLANNING ERROR: The nav2 collision monitor package does not work without any observation sources."
            )

        collision_monitor = {
            "ros__parameters": {
                "base_frame_id": self.info["control_frame"],
                "odom_frame_id": self.info["odom_frame"],
                "cmd_vel_out_topic": self.info["planning_cmd_vel_topic"],
                "state_topic": "~/state",
                "transform_tolerance": 0.1,
                "source_timeout": 1.0,
                "stop_pub_timeout": 1.0,
                "polygons": ["PolygonStop"],
                "PolygonStop": {
                    "type": "polygon",
                    "footprint_topic": "local_costmap/published_footprint",
                    "action_type": "approach",
                    "min_points": 4,
                    "time_before_collision": 3.0,
                    "simulation_time_step": 0.1,
                    "enabled": True,
                },
                "observation_sources": list(observation_sources.keys()),
            },
        }
        collision_monitor["ros__parameters"].update(observation_sources)

        return ("nav2_collision_monitor_node", collision_monitor)

    def generate_nav2_local_and_global_costmap_configuration(
        self, lasers
    ) -> Tuple[Any, Any, Any]:
        observation_source = {}
        laserscan_topic = ""
        if lasers:
            laserscan_topic: str = lasers[0]
        else:
            try:
                laserscan_topic = self.info["laserscan_topic"]
            except KeyError:
                pass
        if not laserscan_topic == "":
            if not laserscan_topic.startswith("/"):
                laserscan_topic = "/" + self.info["namespace"] + "/" + laserscan_topic
        else:
            print(
                f"PLANNING ERROR: The nav2 local and global costmaps do not work without any observation sources."
            )
            return

        observation_source["source"] = {
            "data_type": "LaserScan",
            "topic": laserscan_topic,
            "obstacle_max_range": 4.0,
            "raytrace_max_range": 4.0,
            "max_obstacle_height": 2.0,
            "clearing": True,
            "marking": True,
        }
        footprint = str(self.vehicle.estimate_footprint(self.info["footprint_padding"]))
        local_costmap = {
            "local_costmap": {
                "ros__parameters": {
                    "update_frequency": 10.0,
                    "publish_frequency": 1.0,
                    "footprint": footprint,
                    "global_frame": self.info["odom_frame"],
                    "robot_base_frame": self.info["control_frame"],
                    "rolling_window": True,
                    "width": 12,
                    "height": 12,
                    "resolution": 0.05,
                    "always_send_full_costmap": True,
                    "plugins": ["obstacle_layer"],
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "enabled": True,
                        "observation_sources": "source",
                    },
                },
            },
        }
        local_costmap["local_costmap"]["ros__parameters"]["obstacle_layer"].update(
            copy.deepcopy(observation_source)
        )

        global_costmap = {
            "global_costmap": {
                "ros__parameters": {
                    "update_frequency": 10.0,
                    "publish_frequency": 1.0,
                    "footprint": footprint,
                    "global_frame": self.info["map_frame"],
                    "robot_base_frame": self.info["control_frame"],
                    "track_unknown_space": True,
                    "width": 12,
                    "height": 12,
                    "resolution": 0.05,
                    "always_send_full_costmap": True,
                    "plugins": [
                        "static_layer",
                        "obstacle_layer",
                        "inflation_layer",
                    ],
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "enabled": True,
                        "observation_sources": "source",
                    },
                    "static_layer": {
                        "plugin": "nav2_costmap_2d::StaticLayer",
                        "map_subscribe_transient_local": True,
                    },
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "cost_scaling_factor": 1.20,
                        "inflation_radius": 5.0,
                    },
                },
            },
        }
        global_costmap["global_costmap"]["ros__parameters"]["obstacle_layer"].update(
            copy.deepcopy(observation_source)
        )

        footprint_manager = (
            "footprint_manager_node",
            {
                "ros__parameters": {
                    "footprint_subscription_topic": self.info["footprint_topic"],
                    "footprint_publishers_topics": [
                        "local_costmap/footprint",
                        "global_costmap/footprint",
                    ],
                    "footprint_stamped_publishers_topics": [
                        self.info["footprint_stamped_topic"],
                    ],
                    "first_value_frame": self.vehicle.root.name,
                    "first_value": footprint,
                    "publish_frame": self.info["control_frame"],
                    "frequency": 1.0,
                }
            },
        )

        return local_costmap, global_costmap, footprint_manager

    def generate_nav2_planner_server_configuration(self) -> Tuple[str, Any]:
        nav2_planner_configuration = (
            "nav2_planner_server",
            {
                "ros__parameters": {
                    "expected_planner_frequency": 5.0,
                    "planner_plugins": ["GridBased"],
                    "GridBased": {
                        "plugin": "nav2_navfn_planner/NavfnPlanner",
                        "tolerance": 0.5,
                        "use_astar": False,
                        "allow_unknown": True,
                    },
                }
            },
        )
        return nav2_planner_configuration

    def generate_navigation_plan_transformer(self) -> Tuple[str, Any]:
        plan_frame = self.vehicle.root.name
        try:
            plan_frame = self.info["plan_frame"]
        except KeyError:
            print(f"Key 'plan_frame' not specified {plan_frame} will be used")

        navigation_plan_transformer_configuration = (
            "nav2_plan_transformer_node",
            {
                "ros__parameters": {
                    "plan_client_topic": "navigate_to_pose",
                    "plan_server_topic": self.info["plan_topic"],
                    "goal_pose_subscription_topic": self.info["goal_pose_topic"],
                    "goal_pose_publisher_topic": "goal_pose",
                    "plan_server_topic": self.info["plan_topic"],
                    "path_publisher_topic": self.info["path_topic"],
                    "path_subscription_topic": "plan",
                    "input_reference_frame": plan_frame,
                    "output_reference_frame": self.info["control_frame"],
                }
            },
        )
        return navigation_plan_transformer_configuration

    def generate_nav2_switch_manager(self) -> Tuple[str, Any]:
        folder = os.path.join(os.path.dirname(self.file), "nav2_behavior_trees")
        file = os.path.join(folder, "tf_following.xml")
        if not os.path.isdir(folder):
            os.mkdir(folder)

        follow_frame = "nav2_tf_following_frame"
        try:
            follow_frame = self.info["follow_frame"]
        except KeyError:
            print(f"Key 'follow_frame' not specified {follow_frame} will be used")

        nav2_switch_manager_tf_following = (
            "nav2_switch_manager_node",
            {
                "ros__parameters": {
                    "tf_frame": follow_frame,
                    "behavior_tree_path": os.path.join(
                        file,
                    ),
                    "nav_to_action_topic": self.info["plan_topic"],
                }
            },
        )
        if not os.path.exists(file):
            with open(file, "w"):
                pass

        return nav2_switch_manager_tf_following

    def generate_nav2_controller_server_configuration(self) -> Tuple[str, Any]:
        differential_drive = True
        if not any(
            [
                kinematic.type == "omnidirectional"
                for kinematic in self.vehicle.kinematics
            ]
        ):
            differential_drive = False

        nav2_controller_server = (
            "nav2_controller_server",
            {
                "ros__parameters": {
                    "odom_topic": self.info["odom_topic"],
                    "controller_plugins": ["FollowPath"],
                    "controller_frequency": 300.0,
                    "controller_plugin_types": ["neo_local_planner::NeoLocalPlanner"],
                    "progress_checker": {
                        "plugin": "nav2_controller::SimpleProgressChecker",
                        "required_movement_radius": 0.5,  # TODO
                        "movement_time_allowance": 100.0,  # TODO
                    },
                    "goal_checker": {
                        "plugin": "nav2_controller::SimpleGoalChecker",
                        "xy_goal_tolerance": 0.05,  # TODO
                        "yaw_goal_tolerance": 0.05,  # TODO
                        "stateful": True,
                    },
                    "FollowPath": {
                        "plugin": "neo_local_planner::NeoLocalPlanner",
                        "base_frame": self.info["control_frame"],
                        "local_frame": self.info["odom_frame"],
                        "odom_topic": self.info["odom_topic"],
                        "acc_lim_x": 0.25,  # TODO
                        "acc_lim_y": 0.25 if not differential_drive else 0.0,  # TODO
                        "acc_lim_theta": 0.8,  # TODO
                        "max_vel_x": 0.8,  # TODO
                        "min_vel_x": -0.2,  # TODO
                        "max_vel_y": 0.5 if not differential_drive else 0.0,  # TODO
                        "min_vel_y": -0.5 if not differential_drive else 0.0,  # TODO
                        "max_rot_vel": 0.8,  # TODO
                        "min_rot_vel": -0.8,  # TODO
                        "max_trans_vel": 0.8,  # TODO
                        "min_trans_vel": 0.1,  # TODO
                        "yaw_goal_tolerance": 0.005,  # TODO
                        "xy_goal_tolerance": 0.01,  # TODO
                        "goal_tune_time": 0.5,  # TODO
                        "lookahead_time": 0.4,  # TODO
                        "lookahead_dist": 1.0,  # TODO
                        "start_yaw_error": 0.5,  # TODO
                        "pos_x_gain": 1.0,  # TODO
                        "pos_y_gain": 1.0 if not differential_drive else 0.0,  # TODO
                        "static_yaw_gain": 3.0,  # TODO
                        "cost_x_gain": 0.1,  # TODO
                        "cost_y_gain": 0.1 if not differential_drive else 0.0,  # TODO
                        "cost_y_lookahead_dist": (
                            0.3 if not differential_drive else 0.0
                        ),  # TODO
                        "cost_y_lookahead_time": (
                            0.3 if not differential_drive else 0.0
                        ),  # TODO
                        "cost_yaw_gain": 2.0,  # TODO
                        "low_pass_gain": 0.2,  # TODO
                        "max_cost": 0.95,  # TODO
                        "max_curve_vel": 0.4,  # TODO
                        "max_goal_dist": 0.5,  # TODO
                        "max_backup_dist": 1.0,  # TODO
                        "min_stop_dist": 0.2,  # TODO
                        "differential_drive": differential_drive,  # TODO
                        "allow_reversing": True,  # TODO
                    },
                },
            },
        )
        return nav2_controller_server

    def generate_nav2_behaviors_configuration(self) -> Tuple[str, Any]:
        nav2_behaviors_configuration = (
            "nav2_behavior_server",
            {
                "ros__parameters": {
                    "costmap_topic": "local_costmap/costmap_raw",
                    "footprint_topic": "local_costmap/published_footprint",
                    "cycle_frequency": 10.0,
                    "behavior_plugins": ["spin", "backup", "wait"],
                    "spin": {"plugin": "nav2_behaviors/Spin"},
                    "backup": {"plugin": "nav2_behaviors/BackUp"},
                    "wait": {"plugin": "nav2_behaviors/Wait"},
                    "global_frame": self.info["odom_frame"],
                    "robot_base_frame": self.info["control_frame"],
                    "transform_tolerance": 0.1,  # TODO: understand what it means
                    "simulate_ahead_time": 2.0,  # TODO: understand what it means
                    "max_rotational_vel": 0.5,  # TODO: understand what it means
                    "min_rotational_vel": 0.3,  # TODO: understand what it means
                    "rotational_acc_lim": 0.8,  # TODO: understand what it means
                }
            },
        )
        return nav2_behaviors_configuration

    def generate_nav2_bt_navigator_configuration(self) -> Tuple[str, Any]:
        nav2_bt_navigator_configuration = (
            "nav2_bt_navigator",
            {
                "ros__parameters": {
                    "global_frame": self.info["map_frame"],
                    "robot_base_frame": self.info["control_frame"],
                    "odom_topic": self.info["odom_topic"],
                    "bt_loop_duration": 10,
                    "default_server_timeout": 20,
                    "groot_zmq_publisher_port": 1666,
                    "groot_zmq_server_port": 1667,
                    "navigate_to_pose": {
                        "plugin": "nav2_bt_navigator/NavigateToPoseNavigator"
                    },
                    "navigate_through_poses": {
                        "plugin": "nav2_bt_navigator/NavigateThroughPosesNavigator"
                    },
                    "plugin_lib_names": [
                        "nav2_compute_path_to_pose_action_bt_node",
                        "nav2_compute_path_through_poses_action_bt_node",
                        "nav2_smooth_path_action_bt_node",
                        "nav2_follow_path_action_bt_node",
                        "nav2_spin_action_bt_node",
                        "nav2_wait_action_bt_node",
                        "nav2_back_up_action_bt_node",
                        "nav2_drive_on_heading_bt_node",
                        "nav2_clear_costmap_service_bt_node",
                        "nav2_is_stuck_condition_bt_node",
                        "nav2_goal_reached_condition_bt_node",
                        "nav2_goal_updated_condition_bt_node",
                        "nav2_globally_updated_goal_condition_bt_node",
                        "nav2_is_path_valid_condition_bt_node",
                        "nav2_initial_pose_received_condition_bt_node",
                        "nav2_reinitialize_global_localization_service_bt_node",
                        "nav2_rate_controller_bt_node",
                        "nav2_distance_controller_bt_node",
                        "nav2_speed_controller_bt_node",
                        "nav2_truncate_path_action_bt_node",
                        "nav2_truncate_path_local_action_bt_node",
                        "nav2_goal_updater_node_bt_node",
                        "nav2_recovery_node_bt_node",
                        "nav2_pipeline_sequence_bt_node",
                        "nav2_round_robin_node_bt_node",
                        "nav2_transform_available_condition_bt_node",
                        "nav2_time_expired_condition_bt_node",
                        "nav2_path_expiring_timer_condition",
                        "nav2_distance_traveled_condition_bt_node",
                        "nav2_single_trigger_bt_node",
                        "nav2_is_battery_low_condition_bt_node",
                        "nav2_navigate_through_poses_action_bt_node",
                        "nav2_navigate_to_pose_action_bt_node",
                        "nav2_remove_passed_goals_action_bt_node",
                        "nav2_planner_selector_bt_node",
                        "nav2_controller_selector_bt_node",
                        "nav2_goal_checker_selector_bt_node",
                        "nav2_controller_cancel_bt_node",
                        "nav2_path_longer_on_approach_bt_node",
                        "nav2_wait_cancel_bt_node",
                        "nav2_spin_cancel_bt_node",
                        "nav2_back_up_cancel_bt_node",
                        "nav2_drive_on_heading_cancel_bt_node",
                    ],
                }
            },
        )
        return nav2_bt_navigator_configuration

    def generate_nav2_waypoint_follower_configuration(self) -> Tuple[str, Any]:
        nav2_waypoint_follower_configuration = (
            "nav2_waypoint_follower",
            {
                "ros__parameters": {
                    "loop_rate": 20,
                    "stop_on_failure": False,
                    "waypoint_task_executor_plugin": "wait_at_waypoint",
                    "wait_at_waypoint": {
                        "plugin": "nav2_waypoint_follower::WaitAtWaypoint",
                        "enabled": True,
                        "waypoint_pause_duration": 200,
                    },
                }
            },
        )
        return nav2_waypoint_follower_configuration
