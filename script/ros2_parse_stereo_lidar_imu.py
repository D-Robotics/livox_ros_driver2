#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Image, Imu, PointCloud2
# from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from rclpy.time import Time
import open3d as o3d
from sensor_msgs_py import point_cloud2
import struct

def convert_nv12_to_bgr(nv12_data, width, height):
    # NV12 to BGR conversion
    yuv_image = np.frombuffer(nv12_data, dtype=np.uint8)
    yuv_image = yuv_image.reshape((height * 3 // 2, width))
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_NV12)
    return bgr_image

def parse_pointcloud2_xyz_intensity(msg):
    fields = {f.name: f.offset for f in msg.fields}
    point_step = msg.point_step
    data = msg.data
    num_points = msg.width * msg.height

    points = []

    for i in range(num_points):
        base = i * point_step  # 这是 offset

        # 提取 x, y, z, intensity（float32）
        x = struct.unpack_from('f', data, offset=base + fields['x'])[0]
        y = struct.unpack_from('f', data, offset=base + fields['y'])[0]
        z = struct.unpack_from('f', data, offset=base + fields['z'])[0]
        intensity = struct.unpack_from('f', data, offset=base + fields['intensity'])[0]

        if any(map(np.isnan, [x, y, z])):
            continue

        points.append([x, y, z, intensity])

    return np.array(points, dtype=np.float32)


def process_ros2_bag(ros2_bag_path, output_dir):
    # Create output directories
    imu_dir = os.path.join(output_dir, 'imu')
    image_dir = os.path.join(output_dir, 'images')
    lidar_dir = os.path.join(output_dir, 'lidar')
    os.makedirs(imu_dir, exist_ok=True)
    os.makedirs(image_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)
    
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
        
        elif 'lidar' in topic.lower():
            msg = deserialize_message(data, PointCloud2)
            # Convert PointCloud2 to numpy array
            pts = parse_pointcloud2_xyz_intensity(msg)
            pcd_filename = os.path.join(lidar_dir, f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}.pcd")
            with open(pcd_filename, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\n')
                f.write('FIELDS x y z intensity\n')
                f.write('SIZE 4 4 4 4\n')
                f.write('TYPE F F F F\n')
                f.write('COUNT 1 1 1 1\n')
                f.write(f'WIDTH {pts.shape[0]}\n')
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {pts.shape[0]}\n')
                f.write('DATA ascii\n')
                for pt in pts:
                    f.write(f'{pt[0]} {pt[1]} {pt[2]} {pt[3]}\n')

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
    ros2_bag_path = '/mnt/VirtualBoxShare/2_stereo_imu_calib_20250529/4_stereo_lidar_calib/stereo_livox_lidar_imu_bag_20250529_1'  # Replace with your ROS2 bag path
    output_dir = '/mnt/VirtualBoxShare/2_stereo_imu_calib_20250529/4_stereo_lidar_calib/stereo_livox_lidar_imu_bag_20250529_1_spilt'
    process_ros2_bag(ros2_bag_path, output_dir)
    rclpy.shutdown()