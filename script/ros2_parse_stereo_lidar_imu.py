      
#!/usr/bin/env python3

import os
import argparse
import struct

import cv2
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu, PointCloud2
import rosbag2_py


def convert_nv12_to_bgr(nv12_data, width, height):
    yuv = np.frombuffer(nv12_data, dtype=np.uint8).reshape((height * 3 // 2, width))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def parse_pointcloud2_xyz_intensity_offsettime(msg):
    fields = {f.name: f.offset for f in msg.fields}
    point_step = msg.point_step
    num_points = msg.width * msg.height
    data = msg.data

    points = []

    for i in range(num_points):
        base = i * point_step
        x = struct.unpack_from('f', data, base + fields['x'])[0]
        y = struct.unpack_from('f', data, base + fields['y'])[0]
        z = struct.unpack_from('f', data, base + fields['z'])[0]
        intensity = struct.unpack_from('f', data, base + fields['intensity'])[0]
        offset_time = struct.unpack_from('I', data, base + fields['offset_time'])[0]

        if not any(map(np.isnan, [x, y, z])):
            points.append([x, y, z, intensity, offset_time])

    return np.array(points, dtype=object)


def save_pcd_ascii(filename, points):
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity offset_time\n")
        f.write("SIZE 4 4 4 4 4\n")
        f.write("TYPE F F F F U\n")
        f.write("COUNT 1 1 1 1 1\n")
        f.write(f"WIDTH {points.shape[0]}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {points.shape[0]}\n")
        f.write("DATA ascii\n")
        for pt in points:
            f.write(f"{pt[0]} {pt[1]} {pt[2]} {pt[3]} {pt[4]}\n")


def process_ros2_bag(bag_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    imu_dir = os.path.join(output_dir, 'imu')
    image_dir = os.path.join(output_dir, 'stereo/cam_combine/')
    lidar_dir = os.path.join(output_dir, 'lidar')
    os.makedirs(imu_dir, exist_ok=True)
    os.makedirs(image_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic info from metadata
    topic_types = {}
    for topic_info in reader.get_all_topics_and_types():
        topic_types[topic_info.name] = topic_info.type

    imu_data = []

    while reader.has_next():
        topic, data, _ = reader.read_next()

        msg_type = topic_types.get(topic, '')

        timestamp_ns = None

        if msg_type == 'sensor_msgs/msg/Imu':
            msg = deserialize_message(data, Imu)
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            imu_data.append({
                'timestamp': str(timestamp_ns),
                'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            })

        elif msg_type == 'sensor_msgs/msg/Image':
            msg = deserialize_message(data, Image)
            if msg.encoding.lower() == 'nv12':
                bgr = convert_nv12_to_bgr(msg.data, msg.width, msg.height)
                timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                filename = os.path.join(image_dir, f"{timestamp_ns}.png")
                cv2.imwrite(filename, bgr)

        elif msg_type == 'sensor_msgs/msg/PointCloud2':
            msg = deserialize_message(data, PointCloud2)
            pts = parse_pointcloud2_xyz_intensity(msg)
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            filename = os.path.join(lidar_dir, f"{timestamp_ns}.pcd")
            save_pcd_ascii(filename, pts)

    # Save IMU data
    imu_file = os.path.join(imu_dir, 'data.csv')
    with open(imu_file, 'w') as f:
        f.write("# timestamp(ns), orientation(x y z w), angular_velocity(x y z), linear_acceleration(x y z)\n")
        for imu in imu_data:
            f.write(f"{imu['timestamp']},"
                    f"{','.join(map(str, imu['orientation']))},"
                    f"{','.join(map(str, imu['angular_velocity']))},"
                    f"{','.join(map(str, imu['linear_acceleration']))}\n")

    print(f"[Done] Data has been saved to: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description="Extract IMU, images, and point clouds from a ROS 2 bag by message type. This script must be execued on ROS2 Humble.")
    parser.add_argument("--bag", type=str, required=True, help="Path to the ROS2 bag directory (must contain metadata.yaml)")

    args = parser.parse_args()
    bag_path = os.path.abspath(args.bag)
    bag_dir_name = os.path.basename(bag_path.rstrip('/'))
    output_dir = os.path.join(os.path.dirname(bag_path), f"decode_{bag_dir_name}")

    rclpy.init()
    process_ros2_bag(bag_path, output_dir)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
