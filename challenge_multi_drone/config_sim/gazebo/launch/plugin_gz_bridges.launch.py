"""drone_bridges.py."""

# Copyright 2022 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def drone_bridges(context):
    """Return drone bridges."""
    namespace = LaunchConfiguration('namespace').perform(context)

    # [fix13] direction: '@' bidirectional, '[' Gazebo->ROS, ']' ROS->Gazebo.
    # A bidirectional obstacle bridge is a feedback loop (gz -> ROS -> gz), and
    # this file runs once per drone, which amplified the topic to ~5k msgs/s.
    want_obstacle = LaunchConfiguration('obstacle_bridge').perform(context).lower() \
        in ('true', '1', 'yes')

    bridges = [
        # (gz_topic, ros_topic, gz_type, ros_type, direction)
        (f"/model/{namespace}/leds/control", "leds/control",
         "ignition.msgs.Color", "std_msgs/msg/ColorRGBA", "]"),
    ]
    if want_obstacle:
        bridges.append(("/dynamic_obstacles/locations", "/dynamic_obstacles/locations",
                        "ignition.msgs.Pose", "geometry_msgs/msg/PoseStamped", "["))

    nodes = []
    for gz_topic, ros_topic, gz_type, ros_type, direction in bridges:
        n = Node(package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=namespace,
                output='screen',
                arguments=[f'{gz_topic}@{ros_type}{direction}{gz_type}'],
                remappings=[(gz_topic, ros_topic)]
            )
        nodes.append(n)
    return nodes


def generate_launch_description():
    """Generate Launch description with world bridges."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            description='Drone ID to create bridges'
        ),
        # [fix13] only the base launch creates the (single) obstacle bridge
        DeclareLaunchArgument(
            'obstacle_bridge',
            default_value='true',
            description='create the GZ->ROS bridge for /dynamic_obstacles/locations'
        ),
        OpaqueFunction(function=drone_bridges)
    ])
