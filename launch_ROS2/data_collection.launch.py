# Copyright (c) 2026，D-Robotics.
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
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default_value'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():

    node_params = [
        {'name':'log_level', 'default_value':'warn', 'description': 'log_level'},

        {'name':'output_dir', 'default_value':'/userdata/lidar_img_data_collect', 'description': 'output_dir'},
        {'name':'imu_topic', 'default_value':'/livox/imu', 'description': 'imu_topic'},
        {'name':'lidar_topic', 'default_value':'/livox/lidar', 'description': 'lidar_topic'},
        {'name':'image_topic', 'default_value':'/husq_stereo_cam_node/image_combine_rgb', 'description': 'image_topic'},

        {'name':'snap_shot', 'default_value':'False', 'description': 'snap_shot'},
        {'name':'enable_pause', 'default_value':'False', 'description': 'enable_pause'},

        {'name':'save_thread_num', 'default_value':'4', 'description': 'save_thread_num'},
        {'name':'gravity', 'default_value':'9.81', 'description': 'gravity'},
        {'name':'image_gap_mode', 'default_value':'0', 'description': 'image_gap_mode'},
        {'name':'lidar_gap_mode', 'default_value':'0', 'description': 'lidar_gap_mode'},

        {'name':'motion_detect', 'default_value':'False', 'description': 'motion_detect'},
        {'name':'motion_imu_window_size', 'default_value':'200', 'description': 'imu_window_size'},
        {'name':'motion_accel_th', 'default_value':'0.7', 'description': 'motion_accel_th'},
        {'name':'motion_gyro_th', 'default_value':'0.0', 'description': '0.0'},
        {'name':'pcd_static_collect_count', 'default_value':'65', 'description': 'pcd_static_collect_count'},
        {'name':'img_static_collect_count', 'default_value':'2', 'description': 'img_static_collect_count'},

        {'name':'save_pcd_bin', 'default_value':'True', 'description': 'save_pcd_bin'},

        {'name':'check_ext_driver', 'default_value':'True', 'description': 'check_ext_driver'},
        {'name':'check_camera_sync', 'default_value':'True', 'description': 'check_camera_sync'},
        {'name':'check_lidar_exist', 'default_value':'True', 'description': 'check_lidar_exist'},

        {'name':'image_format', 'default_value':'png', 'description': 'image_format'},
        {'name':'is_ir', 'default_value':'False', 'description': 'is_ir'},

        {'name':'calib_file', 'default_value':'', 'description': 'calib_file'},

        {'name':'dc_web_ch', 'default_value': '0', 'description': 'data_collection web viusal channel'},
    ]

    launch = declare_configurable_parameters(node_params)

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/ROS2DataCollection/status_image_combine',
            'codec_pub_topic': '/ROS2DataCollection/status_image_combine_jpeg'
        }.items()
    )
    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': 'ROS2DataCollection/status_image_combine_jpeg',
            'websocket_only_show_image': 'True',
            'websocket_channel': LaunchConfiguration('dc_web_ch'),
        }.items()
    )

    launch.append(jpeg_codec_node)
    launch.append(web_node)

    launch.append(Node(
        package='livox_ros_driver2',
        executable='data_collection_node',
        output='screen',
        parameters=[set_configurable_parameters(node_params)],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    ))

    return LaunchDescription(launch)
