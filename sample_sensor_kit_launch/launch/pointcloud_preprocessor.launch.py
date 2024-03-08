# Copyright 2020 Tier IV, Inc. All rights reserved.
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


import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # set concat filter as a component

    # separate components for backward compatibility
    separate_pointcloud_sync_and_concatenate_nodes_str: str = LaunchConfiguration(
        "concatenate_data__separate_pointcloud_sync_and_concatenate_nodes"
    ).perform(context)
    separate_pointcloud_sync_and_concatenate_nodes: bool = (
        separate_pointcloud_sync_and_concatenate_nodes_str.lower() == "true"
    )

    common_input_parameter = {
        "input_topics": [
            "/sensing/lidar/top/pointcloud_before_sync",
            "/sensing/lidar/left/pointcloud_before_sync",
            "/sensing/lidar/right/pointcloud_before_sync",
        ],
        "input_twist_topic_type": "twist",
    }
    if not separate_pointcloud_sync_and_concatenate_nodes:
        # legacy mode for backward compatibility. Not used in default.
        sync_and_concat_component = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_data",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("output", "concatenated/pointcloud"),
            ],
            parameters=[
                common_input_parameter,
                {
                    "output_frame": LaunchConfiguration("base_frame"),
                    "publish_synchronized_pointcloud": True,
                },
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
        concat_components = [sync_and_concat_component]
    else:
        time_sync_component = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudDataSynchronizerComponent",
            name="synchronizer_filter",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ],
            parameters=[
                common_input_parameter,
                {
                    "output_frame": LaunchConfiguration("base_frame"),
                    "approximate_sync": True,
                },
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

        concat_component = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::PointCloudConcatenationComponent",
            name="concatenate_filter",
            remappings=[("output", "concatenated/pointcloud")],
            parameters=[
                {
                    # actual input topics becomes + "_synchronized"
                    "input_topics": [
                        "/sensing/lidar/top/pointcloud",
                        "/sensing/lidar/left/pointcloud",
                        "/sensing/lidar/right/pointcloud",
                    ],
                    "output_frame": LaunchConfiguration("base_frame"),
                    "approximate_sync": True,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
        concat_components = [time_sync_component, concat_component]

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=concat_components,
        # target_container=target_container,
        target_container=LaunchConfiguration("pointcloud_container_name"),
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [concat_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("concatenate_data__separate_pointcloud_sync_and_concatenate_nodes", "False")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
