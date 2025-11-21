Script Descriptions
IMU Intrinsic Calibration

ros2_parse_imu_to_txt.py: In a ROS2 environment, parse IMU data into a TXT file.

imu_txt_to_ros1_bag.py: In a ROS1 environment, convert the parsed IMU TXT file back into a ROS1 bag file.
Note: Pay attention to the gravity constant here!!!

After obtaining the ROS1 bag file, you can use the imu_utils package to perform IMU intrinsic calibration.

Stereo Camera Calibration

stereo_imgs_to_ros1_bag.py: Convert stereo image sequences into a ROS1 bag file.

After obtaining the ROS1 bag file, you can use the kalibr package to calibrate the stereo cameras.

Stereo + IMU Joint Calibration

ros2_parse_stereo_imu.py: In a ROS2 environment, parse IMU data into a TXT file and parse stereo images into PNG files.
The PNG filenames are the timestamps of the images.

stereo_imu_to_ros1_bag.py: In a ROS1 environment, convert the parsed IMU TXT file and stereo PNG images back into a ROS1 bag file.
Note: Pay attention to the gravity constant here!!!

After obtaining the ROS1 bag file, you can use the kalibr package for joint calibration of the stereo cameras and IMU.

Lidar Projection onto Images

ros2_parse_stereo_lidar_imu.py: In a ROS2 environment, parse IMU data into a TXT file, parse stereo images into PNG files (each filename is the image timestamp), and parse lidar data into PCD files (each filename is the point cloud timestamp).
