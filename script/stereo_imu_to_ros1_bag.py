#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from datetime import datetime

def create_ros1_bag(data_dir, output_bag_path):
    # Initialize ROS1 node (not strictly necessary but good practice)
    rospy.init_node('ros1_bag_creator', anonymous=True)

    # Open output bag
    bag = rosbag.Bag(output_bag_path, 'w')

    bridge = CvBridge()

    # Process IMU data
    cnt = 0
    imu_dir = os.path.join(data_dir, 'imu')
    with open(os.path.join(imu_dir, 'imu_data.txt'), 'r') as f:
        # Skip header
        next(f)
        for line in f:
            parts = line.strip().split(', ')
            timestamp = float(parts[0])

            # Parse IMU data
            orientation = list(map(float, parts[1].split()))
            angular_velocity = list(map(float, parts[2].split()))
            linear_acceleration = list(map(float, parts[3].split()))

            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.from_sec(timestamp)
            print(imu_msg.header.stamp)
            imu_msg.orientation.x = orientation[0]
            imu_msg.orientation.y = orientation[1]
            imu_msg.orientation.z = orientation[2]
            imu_msg.orientation.w = orientation[3]

            imu_msg.angular_velocity.x = angular_velocity[0]
            imu_msg.angular_velocity.y = angular_velocity[1]
            imu_msg.angular_velocity.z = angular_velocity[2]

            gravity = 9.7887
            imu_msg.linear_acceleration.x = linear_acceleration[0] * gravity
            imu_msg.linear_acceleration.y = linear_acceleration[1] * gravity
            imu_msg.linear_acceleration.z = linear_acceleration[2] * gravity

            # Write to bag
            bag.write('/imu/data', imu_msg, imu_msg.header.stamp)
            cnt += 1
            # if cnt == 10:
                # break

    # Process image data
    print("=======")
    cnt = 0
    image_dir = os.path.join(data_dir, 'images')
    for image_file in sorted(os.listdir(image_dir)):
        if image_file.endswith('.png'):
            timestamp = float(os.path.splitext(image_file)[0])

            # Read image
            img_path = os.path.join(image_dir, image_file)
            bgr_image = cv2.imread(img_path, cv2.IMREAD_COLOR)
            height, _, _ = bgr_image.shape

            # Convert to ROS message
            image_msg_left = bridge.cv2_to_imgmsg(bgr_image[:height // 2, :], encoding='bgr8')
            image_msg_right = bridge.cv2_to_imgmsg(bgr_image[height // 2:, :], encoding='bgr8')
            image_msg_left.header.stamp = rospy.Time.from_sec(timestamp)
            image_msg_right.header.stamp = rospy.Time.from_sec(timestamp)
            print(image_msg_left.header.stamp)

            # Write to bag
            bag.write('/left_camera/image_raw', image_msg_left, image_msg_left.header.stamp)
            bag.write('/right_camera/image_raw', image_msg_right, image_msg_right.header.stamp)
            cnt += 1
            # if cnt == 10:
                # break

    bag.close()

if __name__ == '__main__':
    data_dir = '/mnt/VirtualBoxShare/stereo_imu_calib/3_stereo_imu_calib/stereo_livox_imu_bag_2_split'
    output_bag_path = '/mnt/VirtualBoxShare/stereo_imu_calib/3_stereo_imu_calib/stereo_livox_imu_bag_2_ros1.bag'
    create_ros1_bag(data_dir, output_bag_path)