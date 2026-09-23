# Copyright information
#
# © [2026] LimX Dynamics Technology Co., Ltd. All rights reserved.

"""NX real-robot PCT + SCAN runtime for ROS 2 Humble."""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_ROBOT_ACCID_PATTERN = re.compile(r'^WF_[A-Za-z0-9]+_[A-Za-z0-9]+$')
_NO_OBSTACLE_MAX_LINEAR_SPEED = 0.60


def _valid_robot_accid(value):
    return len(value) <= 64 and _ROBOT_ACCID_PATTERN.fullmatch(value) is not None


def _launch_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in {
        '1', 'true', 'yes', 'on'
    }


def _validate_chassis_outputs(context):
    websocket_enabled = _launch_bool(context, 'start_websocket_bridge')
    cloud_subscription_enabled = _launch_bool(
        context, 'enable_cloud_subscription'
    )
    start_paused = _launch_bool(context, 'start_paused')
    websocket_url = LaunchConfiguration(
        'websocket_url'
    ).perform(context).strip()
    websocket_protocol = LaunchConfiguration(
        'websocket_protocol'
    ).perform(context).strip().lower()
    websocket_robot_accid = LaunchConfiguration(
        'websocket_robot_accid'
    ).perform(context).strip()
    if websocket_protocol != 'legacy':
        raise RuntimeError(
            'nx_pct_navigation supports only websocket_protocol=legacy'
        )
    if websocket_enabled and not websocket_url.startswith(('ws://', 'wss://')):
        raise RuntimeError(
            'websocket_url must be explicitly configured as ws://... or wss://...'
        )
    if websocket_enabled and not _valid_robot_accid(websocket_robot_accid):
        raise RuntimeError(
            'websocket_robot_accid must match WF_<MODEL>_<ID> and be at most '
            '64 characters'
        )
    if not cloud_subscription_enabled:
        if not start_paused:
            raise RuntimeError(
                'Disabling the obstacle cloud requires start_paused=true'
            )
        speed_arguments = ['controller_max_linear_speed']
        if websocket_enabled:
            speed_arguments.extend([
                'websocket_max_linear_speed',
                'websocket_max_lateral_speed',
            ])
        for name in speed_arguments:
            try:
                value = float(LaunchConfiguration(name).perform(context))
            except ValueError as error:
                raise RuntimeError(f'{name} must be numeric') from error
            if value <= 0.0 or value > _NO_OBSTACLE_MAX_LINEAR_SPEED:
                raise RuntimeError(
                    f'{name} must be in (0, '
                    f'{_NO_OBSTACLE_MAX_LINEAR_SPEED:.2f}] when the obstacle '
                    'cloud is disabled'
                )
    return []


def generate_launch_description():
    package_share = get_package_share_directory('scan_planner')
    planner_config = os.path.join(
        package_share, 'config', 'nx_pct_planner.yaml'
    )
    controller_config = os.path.join(
        package_share, 'config', 'nx_pct_controllers.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_id = LaunchConfiguration('frame_id')
    pose_topic = LaunchConfiguration('pose_topic')
    body_odom_topic = LaunchConfiguration('body_odom_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    enable_cloud_subscription = LaunchConfiguration(
        'enable_cloud_subscription'
    )
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    reference_path_topic = LaunchConfiguration('reference_path_topic')
    start_paused = LaunchConfiguration('start_paused')
    start_websocket_bridge = LaunchConfiguration('start_websocket_bridge')
    websocket_url = LaunchConfiguration('websocket_url')
    websocket_protocol = LaunchConfiguration('websocket_protocol')
    websocket_robot_accid = LaunchConfiguration('websocket_robot_accid')
    websocket_send_rate = LaunchConfiguration('websocket_send_rate')
    websocket_command_timeout = LaunchConfiguration(
        'websocket_command_timeout'
    )
    websocket_max_linear_speed = LaunchConfiguration(
        'websocket_max_linear_speed'
    )
    websocket_max_lateral_speed = LaunchConfiguration(
        'websocket_max_lateral_speed'
    )
    websocket_max_angular_speed = LaunchConfiguration(
        'websocket_max_angular_speed'
    )
    controller_max_linear_speed = LaunchConfiguration(
        'controller_max_linear_speed'
    )
    controller_max_angular_speed = LaunchConfiguration(
        'controller_max_angular_speed'
    )

    common_parameters = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('frame_id', default_value='map'),
            DeclareLaunchArgument('pose_topic', default_value='/pose_stamped'),
            DeclareLaunchArgument(
                'body_odom_topic', default_value='/scan/body_odom'
            ),
            DeclareLaunchArgument(
                'cloud_topic', default_value='/corrected_current_pcd'
            ),
            DeclareLaunchArgument(
                'enable_cloud_subscription', default_value='true'
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic', default_value='/sdk_cmd_vel'
            ),
            DeclareLaunchArgument(
                'reference_path_topic', default_value='/pct_path'
            ),
            DeclareLaunchArgument('start_paused', default_value='true'),
            DeclareLaunchArgument(
                'start_websocket_bridge', default_value='false'
            ),
            DeclareLaunchArgument(
                'websocket_url', default_value=''
            ),
            DeclareLaunchArgument(
                'websocket_protocol', default_value='legacy'
            ),
            DeclareLaunchArgument(
                'websocket_robot_accid', default_value=''
            ),
            DeclareLaunchArgument(
                'websocket_send_rate', default_value='10.0'
            ),
            DeclareLaunchArgument(
                'websocket_command_timeout', default_value='0.3'
            ),
            DeclareLaunchArgument(
                'websocket_max_linear_speed', default_value='0.60'
            ),
            DeclareLaunchArgument(
                'websocket_max_lateral_speed', default_value='0.60'
            ),
            DeclareLaunchArgument(
                'websocket_max_angular_speed', default_value='0.20'
            ),
            DeclareLaunchArgument(
                'controller_max_linear_speed', default_value='0.60'
            ),
            DeclareLaunchArgument(
                'controller_max_angular_speed', default_value='0.20'
            ),
            OpaqueFunction(function=_validate_chassis_outputs),
            Node(
                package='scan_planner',
                executable='limx_websocket_cmd_bridge',
                name='limx_websocket_cmd_bridge',
                output='screen',
                condition=IfCondition(start_websocket_bridge),
                parameters=[
                    common_parameters,
                    {
                        'sdk_url': websocket_url,
                        'protocol': ParameterValue(
                            websocket_protocol, value_type=str
                        ),
                        'robot_accid': ParameterValue(
                            websocket_robot_accid, value_type=str
                        ),
                        'cmd_vel_topic': cmd_vel_topic,
                        'pause_topic': '/scan_planner/pause',
                        'start_paused': ParameterValue(
                            start_paused, value_type=bool
                        ),
                        'send_rate': ParameterValue(
                            websocket_send_rate, value_type=float
                        ),
                        'command_timeout': ParameterValue(
                            websocket_command_timeout, value_type=float
                        ),
                        'max_linear_speed': ParameterValue(
                            websocket_max_linear_speed, value_type=float
                        ),
                        'max_lateral_speed': ParameterValue(
                            websocket_max_lateral_speed, value_type=float
                        ),
                        'max_angular_speed': ParameterValue(
                            websocket_max_angular_speed, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package='scan_planner',
                executable='pose_stamped_to_odometry',
                name='pose_stamped_to_odometry',
                output='screen',
                parameters=[
                    common_parameters,
                    {
                        'pose_topic': pose_topic,
                        'odom_topic': body_odom_topic,
                        'frame_id': frame_id,
                        'child_frame_id': 'body',
                        'broadcast_sensor_tf': False,
                        'z_offset': 0.0,
                        'velocity_filter_alpha': 0.35,
                        'max_pose_gap': 0.5,
                        'max_linear_speed': 2.0,
                    },
                ],
            ),
            Node(
                package='scan_planner',
                executable='scan_planner_node',
                name='scan_planner_node',
                output='screen',
                parameters=[
                    planner_config,
                    common_parameters,
                    {
                        'grid_map.frame_id': frame_id,
                        'grid_map.enable_cloud_subscription': ParameterValue(
                            enable_cloud_subscription, value_type=bool
                        ),
                        'fsm.reference_path_topic': reference_path_topic,
                    },
                ],
                remappings=[
                    ('body_pose', body_odom_topic),
                    ('sensor_pose', body_odom_topic),
                    ('cloud', cloud_topic),
                ],
            ),
            Node(
                package='scan_planner',
                executable='closed_loop_controller',
                name='closed_loop_controller',
                output='screen',
                parameters=[
                    controller_config,
                    common_parameters,
                    {
                        'body_pose_topic': body_odom_topic,
                        'cmd_vel_topic': cmd_vel_topic,
                        'start_paused': ParameterValue(
                            start_paused, value_type=bool
                        ),
                        'max_vx': ParameterValue(
                            controller_max_linear_speed, value_type=float
                        ),
                        'max_vyaw': ParameterValue(
                            controller_max_angular_speed, value_type=float
                        ),
                    },
                ],
            ),
        ]
    )