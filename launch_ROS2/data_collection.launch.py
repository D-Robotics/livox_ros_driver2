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


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default_value'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():

    node_params = [
        {'name':'log_level', 'default_value':'info', 'description': 'log_level'},

        {'name':'data_collect_node_name', 'default_value':'LidarImgDataCollectNode', 'description': 'data_collect_node_name'},

        {'name':'output_dir', 'default_value':'/userdata/lidar_img_data_collect', 'description': 'output_dir'},
        {'name':'imu_topic', 'default_value':'/livox/imu', 'description': 'imu_topic'},
        {'name':'lidar_topic', 'default_value':'/livox/lidar', 'description': 'lidar_topic'},
        {'name':'image_topic', 'default_value':'/husq_stereo_cam_node/image_combine_rgb', 'description': 'image_topic'},

        {'name':'image_collect_fps', 'default_value':'0.5', 'description': 'image_collect_fps'},
        {'name':'image_format', 'default_value':'jpg', 'description': 'image_format'},
        {'name':'save_motion_image', 'default_value':'True', 'description': 'save_motion_image'},

        {'name':'target_pcd_count', 'default_value':'200', 'description': 'target_pcd_count'},
        {'name':'save_pcd_binary', 'default_value':'True', 'description': 'save_pcd_binary'},

        {'name':'gravity', 'default_value':'9.81', 'description': 'gravity'},
        {'name':'imu_window_size', 'default_value':'200', 'description': 'imu_window_size'},
        {'name':'accel_std_th', 'default_value':'0.03', 'description': 'accel_std_th'},
        {'name':'gyro_std_th', 'default_value':'0.01', 'description': 'gyro_std_th'},
        {'name':'max_imu_age_sec', 'default_value':'0.2', 'description': 'max_imu_age_sec'},

        {'name':'static_confirm_count_th', 'default_value':'10', 'description': 'static_confirm_count_th'},
        {'name':'motion_confirm_count_th', 'default_value':'3', 'description': 'motion_confirm_count_th'},

        {'name':'enable_image_motion_check', 'default_value':'True', 'description': 'enable_image_motion_check'},
        {'name':'image_diff_ratio_th', 'default_value':'0.02', 'description': 'image_diff_ratio_th'},
        {'name':'image_diff_gray_th', 'default_value':'25', 'description': 'image_diff_gray_th'},

        {'name':'save_data_flag', 'default_value':'False', 'description': 'save_data_flag'},
    ]

    launch = declare_configurable_parameters(node_params)
    launch.append(Node(
        package='lidar_img_data_collect',
        executable='data_collection_node',
        name=LaunchConfiguration('data_collect_node_name'),
        output='screen',
        parameters=[set_configurable_parameters(node_params)],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    ))

    return LaunchDescription(launch)
