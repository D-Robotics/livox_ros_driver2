# 脚本说明

## IMU内参标定

- `ros2_parse_imu_to_txt.py` : ros2环境下，将imu数据解析为txt文件
- `imu_txt_to_ros1_bag.py` : ros1环境下，将解析的imu数据txt文件还原为ros1bag文件，注意这里的重力加速度常数！！！
- 得到ros1bag数据后，可以用imu_utils功能包对imu进行内参标定

## 双目标定

- `stereo_imgs_to_ros1_bag.py` : 将双目图像序列转为ros1bag文件
- 得到ros1bag数据后，可以用kalibr功能包对双目相机进行标定


## 双目+IMU联合标定

- `ros2_parse_stereo_imu.py` : ros2环境下，将imu数据解析为txt文件，将双目图像解析为png图像，png图像的文件名为图像的时间戳
- `stereo_imu_to_ros1_bag.py` : ros1环境下，将解析的imu数据txt文件和双目png图像还原为ros1bag文件，注意这里的重力加速度常数！！！
- 得到ros1bag数据后，可以用kalibr功能包对双目相机+IMU进行联合标定

