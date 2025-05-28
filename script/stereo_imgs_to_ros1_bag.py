#!/usr/bin/env python
import roslib; roslib.load_manifest('sensor_msgs')
import rosbag
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2
from glob import glob

def create_stereo_bag(left_dir, right_dir, output_bag, frame_rate=4):
    bridge = CvBridge()
    bag = rosbag.Bag(output_bag, 'w')

    left_images = sorted(glob(os.path.join(left_dir, '*')))
    right_images = sorted(glob(os.path.join(right_dir, '*')))

    if len(left_images) != len(right_images):
        print("Warning: Left and right image counts don't match!")

    interval = 1.0 / frame_rate

    for i, (left_path, right_path) in enumerate(zip(left_images, right_images)):
        print("=> ", left_path, right_path)
        left_img = cv2.imread(left_path, cv2.IMREAD_COLOR)
        right_img = cv2.imread(right_path, cv2.IMREAD_COLOR)

        if left_img is None or right_img is None:
            print("Warning: Could not read image pair ", left_images, right_images)
            continue

        timestamp = rospy.Time.from_sec(i * interval)

        left_msg = bridge.cv2_to_imgmsg(left_img, encoding="bgr8")
        right_msg = bridge.cv2_to_imgmsg(right_img, encoding="bgr8")

        left_msg.header.stamp = timestamp
        right_msg.header.stamp = timestamp
        left_msg.header.frame_id = "left_camera"
        right_msg.header.frame_id = "right_camera"

        bag.write("/left_camera/image_raw", left_msg, timestamp)
        bag.write("/right_camera/image_raw", right_msg, timestamp)

    bag.close()
    # print(f"Created bag file with {len(left_images)} stereo pairs at {frame_rate}Hz")

if __name__ == '__main__':
    left_dir = "/mnt/VirtualBoxShare/stereo_imu_calib/2_stereo_calib/chessboard_images/left"
    right_dir = "/mnt/VirtualBoxShare/stereo_imu_calib/2_stereo_calib/chessboard_images/right"
    output_bag = "/mnt/VirtualBoxShare/stereo_imu_calib/2_stereo_calib/stereo_images_ros1.bag"

    create_stereo_bag(left_dir, right_dir, output_bag, frame_rate=4)