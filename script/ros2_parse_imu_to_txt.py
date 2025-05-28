# ros2_parse_to_txt.py
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

def main():
    # 配置bag文件路径
    bag_path = "./livix_imu_bag_2"
    output_txt = "./livox_imu_2.txt"

    # 设置ROS2 reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # 获取topic和类型映射
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # 只处理IMU数据（根据实际topic调整）
    imu_topic = "/livox/imu"  # 替换为你的实际topic

    with open(output_txt, 'w') as f:
        # 写入CSV头部（根据IMU消息结构调整）
        f.write("timestamp,orientation_x,orientation_y,orientation_z,orientation_w,"
                "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
                "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z\n")

        # 读取消息
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic == imu_topic:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)

                # 写入IMU数据（sensor_msgs/msg/Imu格式）
                f.write(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d},")
                f.write(f"{msg.orientation.x},{msg.orientation.y},{msg.orientation.z},{msg.orientation.w},")
                f.write(f"{msg.angular_velocity.x},{msg.angular_velocity.y},{msg.angular_velocity.z},")
                f.write(f"{msg.linear_acceleration.x},{msg.linear_acceleration.y},{msg.linear_acceleration.z}\n")

if __name__ == '__main__':
    main()