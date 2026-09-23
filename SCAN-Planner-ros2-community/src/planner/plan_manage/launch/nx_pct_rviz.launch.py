# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""RViz2 client only; run this on the operator's local ROS 2 computer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('scan_planner'),
        'config',
        'nx_pct_navigation.rviz',
    )
    return LaunchDescription(
        [
            Node(
                package='rviz2',
                executable='rviz2',
                name='nx_pct_rviz',
                output='screen',
                arguments=['-d', config],
            )
        ]
    )