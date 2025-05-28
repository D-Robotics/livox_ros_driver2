#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from rclpy.time import Time

def convert_nv12_to_bgr(nv12_data, width, height):
    # NV12 to BGR conversion
    yuv_image = np.frombuffer(nv12_data, dtype=np.uint8)
    yuv_image = yuv_image.reshape((height * 3 // 2, width))
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV12)
    return bgr_image

def process_ros2_bag(ros2_bag_path, output_dir):
    # Create output directories
    imu_dir = os.path.join(output_dir, 'imu')
    image_dir = os.path.join(output_dir, 'images')
    os.makedirs(imu_dir, exist_ok=True)
    os.makedirs(image_dir, exist_ok=True)

    # Open ROS2 bag
    storage_options = rosbag2_py.StorageOptions(uri=ros2_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    imu_data = []
    while reader.has_next():
        topic, data, _ = reader.read_next()

        if 'imu' in topic.lower():
            msg = deserialize_message(data, Imu)
            # Save IMU data to list (we'll write to file later)
            imu_data.append({
                'timestamp': f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}',
                'orientation': [msg.orientation.x, msg.orientation.y, 
                               msg.orientation.z, msg.orientation.w],
                'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y,
                                   msg.angular_velocity.z],
                'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y,
                                      msg.linear_acceleration.z]
            })

        elif 'image' in topic.lower():
            msg = deserialize_message(data, Image)
            if msg.encoding == 'nv12':
                # Convert NV12 to BGR
                bgr_image = convert_nv12_to_bgr(msg.data, msg.width, msg.height)
                # Save image with timestamp in filename
                image_filename = f"{image_dir}/{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}.png"
                cv2.imwrite(image_filename, bgr_image)

    # Save IMU data to TXT file
    with open(f"{imu_dir}/imu_data.txt", 'w') as f:
        f.write("# timestamp(sec), orientation(x,y,z,w), angular_velocity(x,y,z), linear_acceleration(x,y,z)\n")
        for imu in imu_data:
            f.write(f"{imu['timestamp']}, "
                   f"{' '.join(map(str, imu['orientation']))}, "
                   f"{' '.join(map(str, imu['angular_velocity']))}, "
                   f"{' '.join(map(str, imu['linear_acceleration']))}\n")

    print(f"Processing complete. Data saved to {output_dir}")

if __name__ == '__main__':
    rclpy.init()
    ros2_bag_path = '/mnt/VirtualBoxShare/stereo_imu_calib/3_stereo_imu_calib/stereo_livox_imu_bag_2'  # Replace with your ROS2 bag path
    output_dir = '/mnt/VirtualBoxShare/stereo_imu_calib/3_stereo_imu_calib/stereo_livox_imu_bag_2_split'
    process_ros2_bag(ros2_bag_path, output_dir)
    rclpy.shutdown()